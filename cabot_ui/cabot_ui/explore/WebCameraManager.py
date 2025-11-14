import cv2
import os
import time

class WebCameraManager:
    def __init__(self, id=0, logger=None, log_dir=None, resolution="4k"):
        # Initialize the camera

        self.cap = cv2.VideoCapture(id)  # 0 is usually the default camera
        if logger is not None:
            self.logger = logger
        # set to 4K
        if resolution == "4k":
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        elif resolution == "1080p":
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        else:
            self.logger.error("Resolution not supported")
            self.cap.release()
            return

        if self.is_open():
            self.logger.info(f"Camera {id} is opened")
        else:
            self.logger.error(f"Camera {id} is not opened")

        frame = self.get_frame()
        try:
            cv2.imwrite(os.path.join(log_dir, f"test_frame.jpg"), frame)
        except Exception as e:
            pass

    def get_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # if self.logger is not None:
                # self.logger.info("Frame read successfully")
            return frame
        else:
            # if self.logger is not None:
                # self.logger.error("Frame not read")
            return None
        
    def is_open(self):
        return self.cap.isOpened()
