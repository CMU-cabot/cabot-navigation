
import requests
from PIL import Image
from transformers import AutoModelForCausalLM, AutoProcessor


class BaseLocalVLM:
    def __init__(self, model_name: str, logger=None):
        self.model_name = model_name
        self.logger = logger
        self.message = []
        self.ready = False

    def inference(self, image: Image.Image = None, prompt: str = None):
        raise NotImplementedError("Subclasses should implement this method.")
    
    def get_sample_image(self):
        sample_image_url = "https://huggingface.co/sbintuitions/sarashina2-vision-8b/resolve/main/sample.jpg"
        return Image.open(requests.get(sample_image_url, stream=True).raw).convert("RGB")
    
    def heat_up(self):
        self.logger.info("Heating up the model...")
        sample_image = self.get_sample_image()
        sample_inference = self.inference(sample_image, "Explain this image in concisely.")
        self.logger.info(F"Local model sample inference: {sample_inference}")
        self.logger.info(F"Local model loaded: {self.model_name}")
        self.ready = True

    def clear_message(self):
        self.message = []
        self.logger.info("Message history cleared.")

    def add_message(self, role: str, content: str):
        if role not in ["user", "assistant"]:
            self.logger.error("Role must be either 'user' or 'assistant'.")
            return
        self.message.append({"role": role, "content": content})
        

class Sarashina(BaseLocalVLM):
    def __init__(self, logger=None):
        super().__init__(model_name="sbintuitions/sarashina2-vision-8b", logger=logger)
        self.processor = AutoProcessor.from_pretrained(self.model_name, trust_remote_code=True)
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            device_map="cuda",
            torch_dtype="auto",
            trust_remote_code=True,
        )
        self.heat_up()
        

    def inference(self, image: Image.Image = None, prompt: str = None, add_message: bool = False) -> str:
        if not self.ready:
            self.logger.error("Model is not ready. Please heat up the model first.")
            return

        if image is None or prompt is None:
            self.logger.error("Image and prompt must be provided for inference.")
            return 

        self.message.append({"role": "user", "content": prompt})
        text_prompt = self.processor.apply_chat_template(self.message, add_generation_prompt=True)

        inputs = self.processor(
            text=[text_prompt],
            images=[image],
            padding=True,
            return_tensors="pt",
        )
        inputs = inputs.to("cuda")
        stopping_criteria = self.processor.get_stopping_criteria(["\n###"])

        # Inference: Generation of the output
        output_ids = self.model.generate(
            **inputs,
            max_new_tokens=1024,
            temperature=0.5,
            do_sample=True,
            stopping_criteria=stopping_criteria,
        )
        generated_ids = [
            output_ids[len(input_ids) :] for input_ids, output_ids in zip(inputs.input_ids, output_ids)
        ]
        output_text = self.processor.batch_decode(
            generated_ids, skip_special_tokens=True, clean_up_tokenization_spaces=True
        )

        output = output_text[0]
        
        if add_message:
            self.add_message("assistant", output)

        self.logger.info(f"Inference output: {output}")

        return output
