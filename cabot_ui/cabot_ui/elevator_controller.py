# Copyright (c) 2025  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import json
import os
import requests
import threading
try:
    from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil as logger
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)


class ElevatorController:
    def __init__(self):
        self._enabled = True
        self._in_control = False
        self._allow_door_hold = False
        self._client = ElevatorClient()

    @property
    def enabled(self):
        return self._client.valid and self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        logger.info(f"ElevatorController: enabled = {value}")
        self._enabled = value

    @property
    def in_control(self):
        return self.enabled and self._in_control

    @in_control.setter
    def in_control(self, value: bool):
        logger.info(f"ElevatorController: in_control = {value}")
        self._in_control = value

    def call_elevator(self, from_floor, to_floor):
        if self._enabled:
            logger.info(f"ElevatorController: call elevator from {from_floor} to {to_floor}")
            if self._client.call_elevator(from_floor, to_floor):
                self._in_control = True
                self._allow_door_hold = False

    def open_door(self, duration=10):
        if self._enabled and self._in_control:
            logger.info(f"ElevatorController: open door for {duration} seconds")
            self._allow_door_hold = True
            self._hold_door(duration)

    def _hold_door(self, duration):
        if self._enabled and self._in_control and self._allow_door_hold:
            logger.info(f"ElevatorController: hold door for {duration} seconds")
            if self._client.open_door(duration):
                threading.Timer(duration/2, self._hold_door, args=(duration,)).start()

    def close_door(self, after=5):
        if self._enabled and self._in_control:
            logger.info(f"ElevatorController: close door after {after} seconds")
            self._allow_door_hold = False
            self._client.open_door(after)


class ElevatorClient:
    def __init__(self):
        self._auth_header = None
        self._endpoint = os.getenv("ELEVATOR_ENDPOINT")
        self._client_id = os.getenv("ELEVATOR_CLIENT_ID")
        self._client_secret = os.getenv("ELEVATOR_CLIENT_SECRET")

    @property
    def valid(self):
        return self._endpoint and self._client_id and self._client_secret

    def authorize(self):
        self._auth_header = None
        if self.valid:
            try:
                res = requests.post(
                    f"{self._endpoint}/auth/oauth/token",
                    headers={"Content-Type": "application/json"},
                    data=json.dumps({
                        "client_id": self._client_id,
                        "client_secret": self._client_secret,
                    }),
                    timeout=10,
                )
                res.raise_for_status()
                access_token = res.json().get("access_token")
                if access_token:
                    self._auth_header = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {access_token}"
                    }

            except Exception as err:
                logger.error(f"error: {err}")
        return self._auth_header is not None

    def retry_request(self, res, retry_count):
        if res.status_code == 401 and retry_count[0] > 0:
            retry_count[0] -= 1
            return self.authorize()
        return False

    def call_elevator(self, from_floor, to_floor):
        logger.info(f"ElevatorClient: call elevator from {from_floor} to {to_floor}")
        retry_count = [1]
        while True:
            try:
                res = requests.post(
                    f"{self._endpoint}/api/elevator/call",
                    headers=self._auth_header,
                    data=json.dumps({"id": "dummy", "from": from_floor, "to": to_floor}),
                    timeout=10,
                )
                logger.info(f"call_elevator result: {res.json()}")
                if res.status_code == 200:
                    return True
                if not self.retry_request(res, retry_count):
                    break
            except Exception as err:
                logger.error(f"error: {err}")
                break

    def open_door(self, duration=10):
        logger.info(f"ElevatorClient: open door for {duration} seconds")
        retry_count = [1]
        while True:
            try:
                res = requests.post(
                    f"{self._endpoint}/api/elevator/open",
                    headers=self._auth_header,
                    data=json.dumps({"duration": duration}),
                    timeout=10,
                )
                logger.info(f"open_door result: {res.json()}")
                if res.status_code == 200:
                    return True
                if not self.retry_request(res, retry_count):
                    break
            except Exception as err:
                logger.error(f"error: {err}")
                break


elevator_controller = ElevatorController()

if __name__ == "__main__":
    elevator_controller.call_elevator(5, 3)
    elevator_controller.open_door()
    threading.Timer(20, elevator_controller.close_door).start()
    threading.Timer(40, elevator_controller.close_door).start()
