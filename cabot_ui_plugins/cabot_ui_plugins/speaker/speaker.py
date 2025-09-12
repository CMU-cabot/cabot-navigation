import os
from pydub import AudioSegment
import pydub.playback
import subprocess

from ament_index_python.packages import get_package_share_directory
import std_msgs.msg

from cabot_ui.event import NavigationEvent
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil


class Speaker:
    def __init__(self, node_manager):
        self.node = node_manager.get_node("speaker", True)
        # Initialize other attributes and setup as needed
        self.delegate = None
        self._enable_speaker = False
        self._audio_dir = os.path.join(get_package_share_directory(os.environ.get("CABOT_SITE")), "sound")
        self._audio_file_path = None
        self._speaker_volume = 0.0
        if os.path.isdir(self._audio_dir):
            wav_files = [
                f
                for f in os.listdir(self._audio_dir)
                if f.endswith(".wav") and os.path.isfile(os.path.join(self._audio_dir, f))
            ]
        else:
            wav_files = []
        self.audio_files = ",".join(wav_files)
        self.send_speaker_audio_files()

    @property
    def ready(self) -> bool:
        return True

    def process_event(self, event) -> bool:
        if event.subtype == "getspeakeraudiofiles":
            return True

        if event.subtype == "speaker_enable":
            self.enable_speaker(event.param)
            return

        if event.subtype == "speaker_alert":
            self.speaker_alert()
            return

        if event.subtype == "speaker_audio_file":
            self.set_audio_file(event.param)
            return

        if event.subtype == "speaker_volume":
            self.set_speaker_volume(event.param)
            return

    def enable_speaker(self, enable_speaker: str):
        self._enable_speaker = enable_speaker.lower() == "true"
        self._activity_log("change speaker config", "enable speaker", str(self._enable_speaker), visualize=True)

    def set_audio_file(self, filename):
        self._audio_file_path = os.path.join(self._audio_dir, filename)
        self._activity_log("change speaker config", "audio file", str(self._audio_file_path), visualize=True)

    def set_speaker_volume(self, volume):
        try:
            speaker_volume = float(volume)
        except Exception:
            CaBotRclpyUtil.error(f"Invalid volume value: {volume}")
            return

        speaker_volume = max(0, min(100, speaker_volume))
        self._speaker_volume = speaker_volume

        try:
            if speaker_volume == 0:
                subprocess.run(
                    ["pactl", "set-sink-mute", "@DEFAULT_SINK@", "on"],
                    check=True
                )
            else:
                subprocess.run(
                    ["pactl", "set-sink-mute", "@DEFAULT_SINK@", "off"],
                    check=True
                )
                subprocess.run(
                    ["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{int(speaker_volume)}%"],
                    check=True
                )
        except subprocess.CalledProcessError as e:
            CaBotRclpyUtil.error(f"Failed to adjust volume via pactl: {e}")
            return

        self._activity_log("change speaker config", "volume (%)", str(self._speaker_volume), visualize=True)

    def speaker_alert(self):
        if not self._enable_speaker:
            CaBotRclpyUtil.error("Speaker is disabled")
            return

        if not os.path.isfile(self._audio_file_path):
            CaBotRclpyUtil.error(F"Audio file not found: {self._audio_file_path}")
            return

        if self._speaker_volume == 0:
            CaBotRclpyUtil.info("Speaker is muted")
            return

        try:
            sound = AudioSegment.from_wav(self._audio_file_path)
            CaBotRclpyUtil.info(F"Playing {self._audio_file_path} (volume: {self._speaker_volume}%)")
            pydub.playback.play(sound)
        except Exception as e:
            CaBotRclpyUtil.error(F"Playback failed: {e}")

    def send_speaker_audio_files(self):
        if self.audio_files:
            e = NavigationEvent("getspeakeraudiofiles", self._interface.audio_files)
            msg = std_msgs.msg.String()
            msg.data = str(e)
            self._eventPub.publish(msg)
        else:
            return
