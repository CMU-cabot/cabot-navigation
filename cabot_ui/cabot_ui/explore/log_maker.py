import cv2
from PIL import Image, ImageDraw, ImageFont
import datetime
import os

def log_image_and_gpt_response(frames, string, save_path="logs"):
    # Convert frames to PIL Images
    images = [Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) for frame in frames]

    # Determine the size for the final image
    width, height = images[0].size
    total_width = width * len(images)

    if len(images) == 1:
        font_size = 10
    else:
        font_size = 15

    # Load a font with a specified size
    font = ImageFont.truetype("/usr/share/fonts/truetype/fonts-japanese-gothic.ttf", font_size)  # Use a font path available on your system

    # Calculate the total height required for the text
    draw = ImageDraw.Draw(Image.new('RGB', (1, 1)))

    lines = string.split('\n')
    text_height = sum(draw.textbbox((0, 0), line, font=font)[3] - draw.textbbox((0, 0), line, font=font)[1] for line in lines)
    total_height = height + text_height + 20  # Add some padding

    # Create a new blank image
    combined_image = Image.new('RGB', (total_width, total_height), "white")

    # Paste the frames into the combined image
    for i, img in enumerate(images):
        combined_image.paste(img, (i * width, 0))

    # Add the string below the combined image
    draw = ImageDraw.Draw(combined_image)
    current_height = height + 10  # Start drawing text below the images with some padding

    for line in lines:
        text_bbox = draw.textbbox((0, 0), line, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_x = (total_width - text_width) // 2
        draw.text((text_x, current_height), line, font=font, fill="black")
        current_height += text_bbox[3] - text_bbox[1]

    # Save the final image as PNG
    # get current time until sec and make it into a image name
    current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    output_path = os.path.join(save_path, f"{current_time}.png")
    combined_image.save(output_path)

def convert_json_to_string(json_data):
    return "\n".join([f"{key}: {value}" for key, value in json_data.items()])


# Example usage
if __name__ == "__main__":
    # Capture frames from video
    cap = cv2.VideoCapture('videos/front.mp4')  # Change this to your video source

    frames = []
    for _ in range(3):
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)

    cap.release()

    test_json = {"description": "前方には展示物があり、丸いベンチが並んでいます。右手には白いカーブした壁があり、左手には開けたスペースが広がっています。", "left": True, "front": True, "right": False}
    # make test_json into a string with a new line separator
    test_string = convert_json_to_string(test_json)
    # Call the function with the captured frames and a string
    log_image_and_gpt_response(frames, test_string)