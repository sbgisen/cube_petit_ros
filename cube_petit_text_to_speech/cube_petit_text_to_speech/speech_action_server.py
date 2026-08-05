#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time

from action_msgs.msg import GoalStatus
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import server
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from scipy.signal import resample_poly
import sounddevice as sd
import soundfile as sf
from std_msgs.msg import Bool

from cube_petit_speech_msgs.action import Speech
from cube_petit_speech_msgs.msg import AudioDataStamped
from cube_petit_speech_msgs.msg import AudioInfo
from cube_petit_text_to_speech.utils.jtalk import check_goal
from cube_petit_text_to_speech.utils.jtalk import generate_jtalk_file
from cube_petit_text_to_speech.utils.jtalk import resolve_voice_params
from cube_petit_text_to_speech.utils.speaking_state import SpeakingState


class SpeechActionServer(Node):

    def __init__(self) -> None:
        super().__init__('speech_action_server')

        ActionServer(self,
                     Speech,
                     'speech_action_server',
                     handle_accepted_callback=self.handle_accepted_callback,
                     execute_callback=self.call_speech,
                     cancel_callback=self.cancel_callback,
                     callback_group=ReentrantCallbackGroup())

        self.audio_stampled_publisher = self.create_publisher(AudioDataStamped, 'audio_stamped', 10)
        info_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.audio_info_publisher = self.create_publisher(AudioInfo, 'audio_info', info_qos)

        # Lip-sync flag for the facial frontend (private-namespaced ~/speaking, std_msgs/Bool).
        # TRANSIENT_LOCAL so a frontend that (re)connects after startup still gets the
        # current state instead of waiting for the next speech goal.
        # (口パク同期用の発話中フラグ。private topicの~/speakingでpublishし、後から接続した
        # frontendにも直近の状態が届くようlatchする)
        speaking_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.__speaking_publisher = self.create_publisher(Bool, '~/speaking', speaking_qos)
        self.__speaking_state = SpeakingState(self.__publish_speaking)
        self.__publish_speaking(False)  # Announce the initial (not speaking) state explicitly.

        # Per-robot voice (ROSConJP conversation demo, 2026-08-04). Defaults
        # reproduce the pre-existing voice exactly (preset='default',
        # semitone_shift/speed_scale params neutral at 0.0/1.0). See
        # cube_petit_text_to_speech/README.md for how to pick/retune these,
        # and cube_petit_text_to_jtalk.launch.py for how they're wired per robot.
        self.declare_parameter('voice_preset', 'default')
        self.declare_parameter('voice_semitone_shift', 0.0)
        self.declare_parameter('voice_speed_scale', 1.0)
        self.declare_parameter('voice_name', 'mei')
        self.__voice_semitone_shift, self.__voice_speed_scale = resolve_voice_params(
            self.get_parameter('voice_preset').value,
            float(self.get_parameter('voice_semitone_shift').value),
            float(self.get_parameter('voice_speed_scale').value),
        )
        self.__voice_name = self.get_parameter('voice_name').value
        self.get_logger().info(f'Voice: preset={self.get_parameter("voice_preset").value}, '
                               f'semitone_shift={self.__voice_semitone_shift}, '
                               f'speed_scale={self.__voice_speed_scale}, voice_name={self.__voice_name}')

        self.__sampling_rate = 16000  # [TODO] get ros param

        info_msg = AudioInfo()
        info_msg.sample_rate = self.__sampling_rate  # [TODO] get ros param
        info_msg.channels = 1
        info_msg.sample_format = 'S16LE'
        info_msg.bitrate = self.__sampling_rate * 16
        info_msg.coding_format = 'raw'
        self.audio_info_publisher.publish(info_msg)

        self.__lock = threading.Lock()
        self.__is_running = False
        self.get_logger().info('Speech Action Server is ready.')

    def handle_accepted_callback(self, goal_handle: server.ServerGoalHandle) -> None:
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'Canceled phrase before execution: {goal_handle.request.text}')
                goal_handle.canceled()
                return
            with self.__lock:
                if not self.__is_running:
                    self.__is_running = True
                    break
            time.sleep(0.1)

        goal_handle.execute()

    def cancel_callback(self, goal_handle: server.ServerGoalHandle) -> CancelResponse:
        self.get_logger().info(f'Received cancel request for {goal_handle.request.text}')
        return CancelResponse.ACCEPT

    def __publish_speaking(self, speaking: bool) -> None:
        """Publish the current speaking state (used as the SpeakingState callback)."""
        msg = Bool()
        msg.data = speaking
        self.__speaking_publisher.publish(msg)

    def call_speech(self, goal_handle: server.ServerGoalHandle) -> Speech.Result:

        try:
            res = Speech.Result()
            request = goal_handle.request
            if not check_goal(request.text, request.emotion, request.emotion_level, request.pitch, request.speed,
                              request.volume):
                self.get_logger().error('Invalid speech goal received.')
                goal_handle.abort()
                res.result = False
                return res

            feedback = Speech.Feedback()
            goal = goal_handle.request
            speech_file = None
            start_t = self.get_clock().now()
            # goal.speed is validated by check_goal() above; the per-robot
            # speed_scale is applied on top of it here, after validation, so a
            # preset can never make an otherwise-invalid goal pass the check.
            effective_speed = round(goal.speed * self.__voice_speed_scale)
            speech_file = generate_jtalk_file(goal.text,
                                              goal.emotion,
                                              goal.pitch,
                                              effective_speed,
                                              speech_file,
                                              semitone_shift=self.__voice_semitone_shift,
                                              voice_name=self.__voice_name)
            data, sr = sf.read(speech_file, dtype='float32')
            if data.ndim != 1:
                data = np.mean(data, axis=1)
            if sr != self.__sampling_rate:

                def _get_up_down(orig_sr: int, target_sr: int) -> tuple[int, int]:
                    gcd = np.gcd(orig_sr, target_sr)
                    return target_sr // gcd, orig_sr // gcd

                up, down = _get_up_down(sr, self.__sampling_rate)
                data = resample_poly(data, up, down)
                sr = self.__sampling_rate
            data = data * (goal_handle.request.volume / 100.0)
            current_index = 0

            def callback(outdata: np.ndarray, frames: int, time: dict, status: sd.CallbackFlags) -> None:
                nonlocal current_index
                chunk = data[current_index:current_index + frames]
                if len(chunk) < outdata.shape[0]:
                    outdata[:len(chunk), 0] = chunk
                    outdata[len(chunk):, 0] = 0
                    raise sd.CallbackStop()
                outdata[:, 0] = chunk

                msg = AudioDataStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.audio.data = (chunk * np.iinfo(np.int16).max).astype('int16').tobytes()
                self.audio_stampled_publisher.publish(msg)
                current_index += len(chunk)

            try:
                device = None
                # Mark speaking as started only once actual audio playback begins, so the
                # frontend's lip sync lines up with what is actually heard.
                self.__speaking_state.start()
                with sd.OutputStream(sr, int(sr * 0.01), device, 1, callback=callback) as stream:
                    while rclpy.ok() and stream.active:
                        feedback.elapsed_time = (self.get_clock().now() - start_t).to_msg()
                        goal_handle.publish_feedback(feedback)
                        if goal_handle.is_cancel_requested:
                            stream.stop()
                        time.sleep(0.2)
            except Exception as e:
                self.get_logger().error(f'Audio playback error: {e}')
                goal_handle.abort()
                res.result = False
                return res

            if goal_handle.status == GoalStatus.STATUS_CANCELING:
                goal_handle.canceled()
                res.result = False
            else:
                self.get_logger().info(f'Speech Log: {goal.text}')
                goal_handle.succeed()
                res.result = True
            return res

        finally:
            # Guaranteed to run on every exit path (success, cancel, abort, exception), so the
            # frontend never gets stuck thinking the robot is still speaking.
            self.__speaking_state.stop()
            with self.__lock:
                self.__is_running = False


def main() -> None:
    """Entry point for the speech action server."""
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    node = SpeechActionServer()
    executor.add_node(node)  # 必須
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ERROR in speech_action_server]: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
