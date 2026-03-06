#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import base64
import binascii
import json
from io import BytesIO
import logging
import os
import threading
import time
from typing import Any, Dict, Iterator, List, Optional, Union

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from PIL import Image
from pydantic import BaseModel, Field
import requests
import torch
from transformers import AutoModelForCausalLM, AutoProcessor, AutoTokenizer, TextIteratorStreamer

try:
    from transformers import AutoModelForVision2Seq
except Exception:  # pragma: no cover - depends on transformers version
    AutoModelForVision2Seq = None

router = APIRouter()
logger = logging.getLogger(__name__)
class LogFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        record.msg = f"[MASAKI] {record.msg}"
        return super().format(record)

logger.addHandler(logging.StreamHandler())
logger.handlers[0].setFormatter(LogFormatter())



class GenerateRequest(BaseModel):
    prompt: str
    images: List[str] = Field(default_factory=list)
    system_prompt: Optional[str] = None
    model: Optional[str] = None
    max_new_tokens: int = 256
    temperature: float = 0.2
    top_p: float = 0.95
    do_sample: bool = False


class GenerateResponse(BaseModel):
    text: str
    model: str
    elapsed_sec: float


class LoadModelRequest(BaseModel):
    model: Optional[str] = None


class ChatMessage(BaseModel):
    role: str
    content: Union[str, List[Dict[str, Any]]]


class ChatCompletionsRequest(BaseModel):
    model: Optional[str] = None
    messages: List[ChatMessage]
    max_tokens: int = 256
    temperature: float = 0.2
    top_p: float = 0.95
    stream: bool = False


def _as_bool(value: str, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _decode_base64_image(image_data: str) -> Image.Image:
    if "," in image_data and image_data.startswith("data:"):
        _, b64 = image_data.split(",", 1)
    else:
        b64 = image_data
    try:
        raw = base64.b64decode(b64)
    except (binascii.Error, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"Invalid base64 image: {exc}") from exc
    try:
        return Image.open(BytesIO(raw)).convert("RGB")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Failed to decode image: {exc}") from exc


def _load_image(image_data: str) -> Image.Image:
    if isinstance(image_data, str) and image_data.startswith(("http://", "https://")):
        timeout = float(os.environ.get("LOCAL_LLM_IMAGE_FETCH_TIMEOUT_SEC", "30"))
        try:
            response = requests.get(image_data, timeout=timeout)
            response.raise_for_status()
            return Image.open(BytesIO(response.content)).convert("RGB")
        except Exception as exc:
            raise HTTPException(status_code=400, detail=f"Failed to fetch image URL: {exc}") from exc
    return _decode_base64_image(image_data)


def _torch_dtype_from_env() -> Optional[torch.dtype]:
    dtype_name = os.environ.get("LOCAL_LLM_DTYPE", "auto").strip().lower()
    if dtype_name == "auto":
        return None
    mapping = {
        "float16": torch.float16,
        "fp16": torch.float16,
        "bfloat16": torch.bfloat16,
        "bf16": torch.bfloat16,
        "float32": torch.float32,
        "fp32": torch.float32,
    }
    if dtype_name not in mapping:
        raise RuntimeError(f"Unsupported LOCAL_LLM_DTYPE={dtype_name}")
    return mapping[dtype_name]


def _local_files_only() -> bool:
    return _as_bool(os.environ.get("LOCAL_LLM_LOCAL_FILES_ONLY"), False)


def _device_kwargs() -> Dict[str, Any]:
    device_map = os.environ.get("LOCAL_LLM_DEVICE_MAP", "auto")
    if device_map:
        return {"device_map": device_map}
    return {}


class LocalLLMEngine:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.model = None
        self.processor = None
        self.tokenizer = None
        self.model_id = None
        self.is_vision_model = False

    def _default_model_id(self) -> str:
        return os.environ.get("LOCAL_LLM_MODEL_ID", "sbintuitions/sarashina2.2-vision-3b")

    def status(self) -> Dict[str, Any]:
        return {
            "loaded": self.model is not None,
            "model": self.model_id,
            "is_vision_model": self.is_vision_model,
        }

    def _startup_test_image_data_uri(self) -> str:
        image = Image.new("RGB", (32, 32), (255, 255, 255))
        buffer = BytesIO()
        image.save(buffer, format="PNG")
        encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
        return f"data:image/png;base64,{encoded}"

    def _run_startup_test_inference(self) -> GenerateResponse:
        test_prompt = os.environ.get(
            "LOCAL_LLM_STARTUP_TEST_PROMPT",
            "Describe this image in one short sentence.",
        )
        return self.generate(
            prompt=test_prompt,
            images=[self._startup_test_image_data_uri()],
            max_new_tokens=32,
            temperature=0.0,
            top_p=1.0,
            do_sample=False,
        )

    def initialize_on_startup(self) -> Dict[str, Any]:
        model_id = self._default_model_id()
        status = self.load(model_id)
        result = self._run_startup_test_inference()
        logger.info(
            "Startup local LLM image inference completed. model=%s elapsed_sec=%s text=%s",
            result.model,
            result.elapsed_sec,
            result.text[:200],
        )
        return status

    def _processor_has_image_support(self) -> bool:
        return self.processor is not None and getattr(self.processor, "image_processor", None) is not None

    def load(self, model_id: Optional[str] = None) -> Dict[str, Any]:
        target_model = model_id or self._default_model_id()
        logger.info("[MASAKI] Loading local LLM model: %s", target_model)
        with self._lock:
            if self.model is not None and self.model_id == target_model:
                return self.status()

            trust_remote_code = _as_bool(os.environ.get("LOCAL_LLM_TRUST_REMOTE_CODE"), True)
            local_files_only = _local_files_only()
            load_kwargs: Dict[str, Any] = {
                "trust_remote_code": trust_remote_code,
                "low_cpu_mem_usage": True,
                "local_files_only": local_files_only,
            }
            load_kwargs.update(_device_kwargs())
            torch_dtype = _torch_dtype_from_env()
            if torch_dtype is not None:
                load_kwargs["torch_dtype"] = torch_dtype

            last_exc: Optional[Exception] = None

            self.processor = None
            self.tokenizer = None
            self.model = None
            self.is_vision_model = False

            try:
                    self.processor = AutoProcessor.from_pretrained(
                        target_model,
                        trust_remote_code=trust_remote_code,
                        local_files_only=local_files_only,
                    )
            except Exception as exc:
                last_exc = exc
                logger.info("Processor load failed for %s, continuing without processor: %s", target_model, exc)
                self.processor = None

            # Prefer AutoModelForCausalLM for multimodal causal models (e.g. Sarashina vision).
            try:
                self.model = AutoModelForCausalLM.from_pretrained(target_model, **load_kwargs)
                self.is_vision_model = self._processor_has_image_support()
            except Exception as exc:
                last_exc = exc
                logger.warning("Causal model load failed for %s: %s", target_model, exc)
                self.model = None

            if self.model is None and AutoModelForVision2Seq is not None:
                try:
                    if self.processor is None:
                        self.processor = AutoProcessor.from_pretrained(
                            target_model,
                            trust_remote_code=trust_remote_code,
                            local_files_only=local_files_only,
                        )
                    self.model = AutoModelForVision2Seq.from_pretrained(target_model, **load_kwargs)
                    self.is_vision_model = True
                except Exception as exc:
                    last_exc = exc
                    logger.warning("Vision2Seq model load failed for %s: %s", target_model, exc)
                    self.model = None

            if self.model is None:
                logger.exception("Model load failed for %s", target_model)
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to load model {target_model}: {last_exc}",
                )

            if self.processor is not None and hasattr(self.processor, "tokenizer"):
                self.tokenizer = self.processor.tokenizer
            if self.tokenizer is None:
                try:
                    self.tokenizer = AutoTokenizer.from_pretrained(
                        target_model,
                        trust_remote_code=trust_remote_code,
                        local_files_only=local_files_only,
                    )
                except Exception as exc:
                    logger.warning(
                        "Tokenizer load failed for %s; streaming may be unavailable: %s",
                        target_model,
                        exc,
                    )

            self.model.eval()
            self.model_id = target_model

            if self.tokenizer is not None and self.tokenizer.pad_token_id is None and self.tokenizer.eos_token is not None:
                self.tokenizer.pad_token = self.tokenizer.eos_token

            if last_exc is not None:
                logger.info("Model %s loaded after fallback attempts: %s", target_model, last_exc)
            return self.status()

    def _format_prompt(self, prompt: str, system_prompt: Optional[str]) -> str:
        if system_prompt:
            return f"{system_prompt.strip()}\n\n{prompt.strip()}"
        return prompt

    def build_chat_prompt(self, messages: List[Dict[str, Any]], fallback_prompt: str) -> str:
        if self.processor is None or not hasattr(self.processor, "apply_chat_template"):
            return fallback_prompt
        try:
            prompt = self.processor.apply_chat_template(
                messages,
                add_generation_prompt=True,
                tokenize=False,
            )
            if isinstance(prompt, str) and prompt.strip():
                return prompt
        except TypeError:
            # Some processors do not accept tokenize=...
            try:
                prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
                if isinstance(prompt, str) and prompt.strip():
                    return prompt
            except Exception as exc:
                logger.warning("apply_chat_template failed; using fallback prompt: %s", exc)
        except Exception as exc:
            logger.warning("apply_chat_template failed; using fallback prompt: %s", exc)
        return fallback_prompt

    def _build_inputs(self, prompt: str, images: List[str]) -> Dict[str, Any]:
        pil_images = [_load_image(image) for image in images]
        if self._processor_has_image_support() and pil_images:
            image_token = os.environ.get("LOCAL_LLM_IMAGE_TOKEN", "").strip()
            text_prompt = prompt
            if image_token:
                prefix = " ".join([image_token] * len(pil_images))
                text_prompt = f"{prefix}\n{text_prompt}"
            return self.processor(
                text=[text_prompt],
                images=pil_images,
                padding=True,
                return_tensors="pt",
            )

        if images and not self._processor_has_image_support():
            raise HTTPException(status_code=400, detail="Images were provided but loaded model is text-only")

        if self.tokenizer is None:
            raise HTTPException(status_code=500, detail="Tokenizer is not loaded")
        return self.tokenizer(prompt, return_tensors="pt")

    def _move_inputs_to_model_device(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        model_device = next(self.model.parameters()).device
        return {k: v.to(model_device) if hasattr(v, "to") else v for k, v in inputs.items()}

    def _build_generate_kwargs(
        self,
        max_new_tokens: int,
        temperature: float,
        top_p: float,
        do_sample: bool,
    ) -> Dict[str, Any]:
        kwargs = {
            "max_new_tokens": max_new_tokens,
            "do_sample": do_sample,
            "pad_token_id": getattr(self.tokenizer, "pad_token_id", None),
            "eos_token_id": getattr(self.tokenizer, "eos_token_id", None),
        }
        if do_sample:
            kwargs["temperature"] = temperature
            kwargs["top_p"] = top_p
        return {k: v for k, v in kwargs.items() if v is not None}

    def _decode_generated_ids(self, inputs: Dict[str, Any], outputs: torch.Tensor) -> str:
        generated_ids = outputs
        if "input_ids" in inputs and outputs.shape[1] >= inputs["input_ids"].shape[1]:
            generated_ids = outputs[:, inputs["input_ids"].shape[1] :]

        decoder = self.tokenizer if self.tokenizer is not None else self.processor
        return decoder.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=True,
        )[0].strip()

    def generate(
        self,
        prompt: str,
        images: Optional[List[str]] = None,
        system_prompt: Optional[str] = None,
        model_id: Optional[str] = None,
        max_new_tokens: int = 256,
        temperature: float = 0.2,
        top_p: float = 0.95,
        do_sample: bool = False,
    ) -> GenerateResponse:
        self.load(model_id)
        images = images or []
        full_prompt = self._format_prompt(prompt, system_prompt)

        start = time.time()
        with self._lock:
            inputs = self._build_inputs(full_prompt, images)
            inputs = self._move_inputs_to_model_device(inputs)
            generate_kwargs = self._build_generate_kwargs(
                max_new_tokens=max_new_tokens,
                temperature=temperature,
                top_p=top_p,
                do_sample=do_sample,
            )
            with torch.no_grad():
                outputs = self.model.generate(**inputs, **generate_kwargs)
            text = self._decode_generated_ids(inputs, outputs)

        return GenerateResponse(
            text=text,
            model=self.model_id,
            elapsed_sec=round(time.time() - start, 3),
        )

    def stream_generate(
        self,
        prompt: str,
        images: Optional[List[str]] = None,
        system_prompt: Optional[str] = None,
        model_id: Optional[str] = None,
        max_new_tokens: int = 256,
        temperature: float = 0.2,
        top_p: float = 0.95,
        do_sample: bool = False,
    ) -> Iterator[str]:
        self.load(model_id)
        if self.tokenizer is None:
            raise HTTPException(status_code=500, detail="Tokenizer is required for streaming")

        images = images or []
        full_prompt = self._format_prompt(prompt, system_prompt)
        streamer = TextIteratorStreamer(
            self.tokenizer,
            skip_prompt=True,
            skip_special_tokens=True,
        )
        error_holder: Dict[str, Exception] = {}

        def _worker() -> None:
            try:
                with self._lock:
                    inputs = self._build_inputs(full_prompt, images)
                    inputs = self._move_inputs_to_model_device(inputs)
                    generate_kwargs = self._build_generate_kwargs(
                        max_new_tokens=max_new_tokens,
                        temperature=temperature,
                        top_p=top_p,
                        do_sample=do_sample,
                    )
                    generate_kwargs["streamer"] = streamer
                    with torch.no_grad():
                        self.model.generate(**inputs, **generate_kwargs)
            except Exception as exc:  # pragma: no cover - background thread path
                error_holder["error"] = exc
                logger.exception("Streaming generation failed")
                streamer.end()

        thread = threading.Thread(target=_worker, daemon=True)
        thread.start()

        for text in streamer:
            if text:
                yield text

        thread.join()
        if "error" in error_holder:
            raise HTTPException(status_code=500, detail=f"Streaming generation failed: {error_holder['error']}")


engine = LocalLLMEngine()


def _parse_openai_messages(messages: List[ChatMessage]) -> Dict[str, Any]:
    lines: List[str] = []
    images: List[str] = []
    processor_messages: List[Dict[str, Any]] = []

    for message in messages:
        content = message.content
        role = message.role or "user"

        if isinstance(content, str):
            text = content.strip()
            if text:
                lines.append(f"{role}: {text}")
                processor_messages.append(
                    {"role": role, "content": [{"type": "text", "text": text}]}
                )
            continue

        text_parts: List[str] = []
        normalized_content: List[Dict[str, Any]] = []
        for item in content:
            item_type = item.get("type")
            if item_type == "text":
                text_value = item.get("text", "")
                if text_value:
                    text_value = str(text_value)
                    text_parts.append(text_value)
                    normalized_content.append({"type": "text", "text": text_value})
            elif item_type in {"image_url", "image"}:
                if item_type == "image_url":
                    image_url = item.get("image_url")
                    if isinstance(image_url, dict):
                        url = image_url.get("url")
                    else:
                        url = image_url
                else:
                    url = item.get("image")
                if url:
                    url = str(url)
                    images.append(url)
                    normalized_content.append({"type": "image", "image": url})

        if text_parts:
            lines.append(f"{role}: {' '.join(text_parts).strip()}")
        if normalized_content:
            processor_messages.append({"role": role, "content": normalized_content})

    prompt = "\n".join(lines).strip()
    if prompt and not prompt.endswith("\nassistant:"):
        prompt = f"{prompt}\nassistant:"
    return {
        "prompt": prompt,
        "images": images,
        "processor_messages": processor_messages,
    }


def _sse_data(payload: Union[str, Dict[str, Any]]) -> str:
    if isinstance(payload, str):
        return f"data: {payload}\n\n"
    return f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"


def _stream_chat_completion(
    request: ChatCompletionsRequest,
    prompt: str,
    images: List[str],
    model_id: Optional[str],
) -> StreamingResponse:
    now = int(time.time())
    completion_id = f"chatcmpl-local-{now}"
    created = now

    def _event_stream() -> Iterator[str]:
        model_name = engine.model_id or model_id or engine._default_model_id()
        yield _sse_data(
            {
                "id": completion_id,
                "object": "chat.completion.chunk",
                "created": created,
                "model": model_name,
                "choices": [
                    {"index": 0, "delta": {"role": "assistant"}, "finish_reason": None}
                ],
            }
        )

        for chunk in engine.stream_generate(
            prompt=prompt,
            images=images,
            model_id=model_id,
            max_new_tokens=request.max_tokens,
            temperature=request.temperature,
            top_p=request.top_p,
            do_sample=request.temperature > 0.0,
        ):
            yield _sse_data(
                {
                    "id": completion_id,
                    "object": "chat.completion.chunk",
                    "created": created,
                    "model": model_name,
                    "choices": [
                        {"index": 0, "delta": {"content": chunk}, "finish_reason": None}
                    ],
                }
            )

        yield _sse_data(
            {
                "id": completion_id,
                "object": "chat.completion.chunk",
                "created": created,
                "model": model_name,
                "choices": [{"index": 0, "delta": {}, "finish_reason": "stop"}],
            }
        )
        yield _sse_data("[DONE]")

    return StreamingResponse(_event_stream(), media_type="text/event-stream")


@router.get("/healthz")
def healthz():
    return {"status": "ok", **engine.status()}


@router.get("/model_status")
def model_status():
    return engine.status()


@router.post("/load_model")
def load_model(request: Optional[LoadModelRequest] = None):
    return engine.load(request.model if request else None)


@router.post("/generate", response_model=GenerateResponse)
def generate(request: GenerateRequest):
    return engine.generate(
        prompt=request.prompt,
        images=request.images,
        system_prompt=request.system_prompt,
        model_id=request.model,
        max_new_tokens=request.max_new_tokens,
        temperature=request.temperature,
        top_p=request.top_p,
        do_sample=request.do_sample,
    )


@router.post("/v1/chat/completions")
def chat_completions(request: ChatCompletionsRequest):
    if not request.messages:
        raise HTTPException(status_code=400, detail="messages is required")

    parsed = _parse_openai_messages(request.messages)
    if not parsed["prompt"] and not parsed["images"]:
        raise HTTPException(status_code=400, detail="No text prompt found in messages")

    # engine.load(request.model)
    prompt = engine.build_chat_prompt(parsed["processor_messages"], parsed["prompt"])

    if request.stream:
        return _stream_chat_completion(
            request=request,
            prompt=prompt,
            images=parsed["images"],
            model_id=request.model,
        )

    result = engine.generate(
        prompt=prompt,
        images=parsed["images"],
        model_id=request.model,
        max_new_tokens=request.max_tokens,
        temperature=request.temperature,
        top_p=request.top_p,
        do_sample=request.temperature > 0.0,
    )

    now = int(time.time())
    return {
        "id": f"chatcmpl-local-{now}",
        "object": "chat.completion",
        "created": now,
        "model": result.model,
        "choices": [
            {
                "index": 0,
                "finish_reason": "stop",
                "message": {
                    "role": "assistant",
                    "content": result.text,
                },
            }
        ],
        "usage": {
            "prompt_tokens": None,
            "completion_tokens": None,
            "total_tokens": None,
        },
        "elapsed_sec": result.elapsed_sec,
    }
