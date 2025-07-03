
import requests
from PIL import Image
# from transformers import AutoModelForCausalLM, AutoProcessor
from openai import OpenAI
import io
import base64
import os
import datetime
from rclpy.logging import get_logger



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
        # self.logger.info("Heating up the model...")
        # sample_image = self.get_sample_image()
        # sample_inference = self.inference(sample_image, "Explain this image in concisely.")
        # self.logger.info(F"Local model sample inference: {sample_inference}")
        # self.logger.info(F"Local model loaded: {self.model_name}")
        self.ready = True

    def clear_message(self):
        self.message = []
        self.logger.info("Message history cleared.")

    def add_message(self, role: str, content: str):
        if role not in ["user", "assistant"]:
            self.logger.error("Role must be either 'user' or 'assistant'.")
            return
        self.message.append({"role": role, "content": content})
        
class GPT4oVLM(BaseLocalVLM):
    def __init__(self, openai_api_key, logger=None):
        super().__init__(model_name="gpt-4o", logger=logger)
        self.client = OpenAI(api_key=openai_api_key)
        try:
            base_log_dir = os.path.expanduser("~/.ros/log")
            timestamp = datetime.datetime.now().strftime("vlm-%Y%m%d_%H%M%S")
            self.images_dir = os.path.join(base_log_dir, timestamp)
            os.makedirs(self.images_dir, exist_ok=True)
            self.logger.info(f"Created VLM output folder: {self.images_dir}")
        except Exception as e:
            self.logger.error(f"Failed to create VLM output folder: {e}")
            raise
        self.heat_up()


    def inference(self, image: Image.Image = None, prompt: str = None, add_message: bool = False) -> str:
        if image is None or prompt is None:
            if self.logger:
                self.logger.error("Image and prompt must be provided for inference.")
            return None




       

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(self.images_dir, f"gpt4o_image_{timestamp}.png")
        image.save(image_path)
        if self.logger:
            self.logger.info(f"Saved debug image to {image_path}")

        # Prepare image for GPT-4o
        buffered = io.BytesIO()
        image.save(buffered, format="PNG")
        image_bytes = buffered.getvalue()
        image_b64 = base64.b64encode(image_bytes).decode()

        try:
            self.logger.info(f"requesting image")
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a helpful robot that describes surroundings in Japanese.",
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/png;base64,{image_b64}"},
                            },
                        ],
                    },
                ],
                max_tokens=1024,
                temperature=0.5,
            )
            self.logger.info(f"got response")
            output = response.choices[0].message.content


            # Save GPT output to matching .txt file
            output_path = os.path.join(self.images_dir, f"gpt4o_image_{timestamp}.txt")

            
            with open(output_path, "w") as f:
                f.write(output)
            if self.logger:
                self.logger.info(f"Saved GPT-4o output to {output_path}")

            if add_message:
                self.add_message("assistant", output)

            self.logger.info(f"GPT-4o output: {output}")
            return output

        except Exception as e:
            import traceback 
            self.logger.error(traceback.print_exc())
            if self.logger:
                self.logger.error(f"Error calling GPT-4o: {e}")
            return None
            
# class Sarashina(BaseLocalVLM):
#     def __init__(self, logger=None):
#         super().__init__(model_name="sbintuitions/sarashina2-vision-8b", logger=logger)
#         self.processor=AutoProcessor.from_pretrained(self.model_name,trust_remote_code=True)
#         self.model = AutoModelForCausalLM.from_pretrained(
#             self.model_name,
#             device_map="cpu",
#             torch_dtype="auto",
#             trust_remote_code=True,
#         )
#         self.heat_up()
        

#     def inference(self, image: Image.Image = None, prompt: str = None, add_message: bool = False) -> str:
#         if not self.ready:
#             self.logger.error("Model is not ready. Please heat up the model first.")
#             return

#         if image is None or prompt is None:
#             self.logger.error("Image and prompt must be provided for inference.")
            
        

#         self.message.append({"role": "user", "content": prompt})
#         text_prompt = self.processor.apply_chat_template(self.message, add_generation_prompt=True)

#         inputs = self.processor(
#             text=[text_prompt],
#             images=[image],
#             padding=True,
#             return_tensors="pt",
#         )
#         #inputs = inputs.to("cuda")
#         inputs = inputs.to("cpu")
       
#         stopping_criteria = self.processor.get_stopping_criteria(["\n###"])

#         # Inference: Generation of the output
#         output_ids = self.model.generate(
#             **inputs,
#             max_new_tokens=1024,
#             temperature=0.5,
#             do_sample=True,
#             stopping_criteria=stopping_criteria,
#         )
#         generated_ids = [
#             output_ids[len(input_ids) :] for input_ids, output_ids in zip(inputs.input_ids, output_ids)
#         ]
#         output_text = self.processor.batch_decode(
#             generated_ids, skip_special_tokens=True, clean_up_tokenization_spaces=True
#         )

#         output = output_text[0]
        
#         if add_message:
#             self.add_message("assistant", output)

#         self.logger.info(f"Inference output: {output}")

#         return output