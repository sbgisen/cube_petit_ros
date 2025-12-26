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

import os
import pathlib
import re
import subprocess
import sys

from ament_index_python import get_package_share_directory
from beartype import beartype

from cube_petit_speech_msgs.action import Speech

__all__ = ['LIB_ROUTE', 'simple_jtalk', 'generate_jtalk_command', 'adjust_text']

# Jtalk Library Path
LIB_ROUTE = f'{get_package_share_directory("cube_petit_text_to_speech")}/speech_lib/'

OUTPUT_FILE = pathlib.Path('/tmp/jtalk_output.wav')


@beartype
def simple_jtalk(text: str) -> None:
    """Execute jtalk speech without specifying parameters aside the speech phrase.

    Args:
        text: Phrase to be said.
    """
    if text == '':
        return
    generate_jtalk_file(text, Speech.Goal.EMOTION_DEFAULT, 100, 100)
    subprocess.Popen(generate_jtalk_command(OUTPUT_FILE),
                     stdin=subprocess.PIPE,
                     stdout=subprocess.PIPE,
                     shell=False,
                     preexec_fn=os.setsid)


@beartype
def check_goal(goal: Speech.Goal) -> bool:
    # text
    if not goal.text:
        return False
    valid_emotions = {'happy', 'normal', 'angry', 'bashful', 'sad'}
    if goal.emotion not in valid_emotions:
        return False
    if not (1 <= goal.emotion_level <= 5):
        return False
    if not (50 <= goal.pitch < 200):
        return False
    if not (50 <= goal.speed < 300):
        return False
    if not (1 <= goal.volume <= 100):
        return False

    return True


@beartype
def generate_jtalk_file(text: str,
                        emotion: str,
                        pitch: int,
                        speed: int,
                        file_path: pathlib.Path | None = None) -> pathlib.Path:
    """Generate jtalk audio file.

    Args:
        text: Phrase to be said.
        emotion: Emotion of the phrase to be played.
        pitch: Pitch of the phrase to be played. Value between 50 to 200.
        speed: Speed of the phrase to be played. Value between 50 and 400.
        file_path: Path to save mp3 audio file.

    Returns:
        Path to the generated audio file.
    """
    text = adjust_text(text)
    echo = f'echo {text} | '
    open_jtalk = f'{LIB_ROUTE}open_jtalk-1.11/bin/open_jtalk '
    dic = f'-x {LIB_ROUTE}open_jtalk_dic_utf_8-1.11 '
    htsvoice = f'-m {LIB_ROUTE}MMDAgent_Example-1.6/Voice/mei/mei_{emotion}.htsvoice '
    speed_param = f'-r {float(speed) / 100} '
    intonation = f'-jf {float(pitch) / 100} '
    if file_path is None:
        file_path = OUTPUT_FILE
    sox = f'sox -t wav - -p silence 1 0.1 0.1% reverse | sox -p -t wav {str(file_path)} silence 1 0.1 0.1% reverse'
    outwav = f'-ow /dev/stdout | {sox}'
    subprocess.run(echo + open_jtalk + dic + htsvoice + speed_param + intonation + outwav,
                   stdin=subprocess.PIPE,
                   stdout=subprocess.PIPE,
                   shell=True)
    return file_path


@beartype
def generate_jtalk_command(file_path: pathlib.Path | None = None, options: list[str] = []) -> list[str]:
    """Generate shell command to play audio file.

    Args:
        file_path: Path to wav audio file.
        options: Additional options for the command.

    Returns:
        Command and arguments for subprocess.
    """
    return [
        'pw-play',
        str(file_path if file_path is not None else OUTPUT_FILE),
    ] + options


@beartype
def adjust_text(text: str) -> str:
    """Change unreadable characters inside a text to readable ones.

    Args:
        text: Text (Japanese) to be adjusted.

    Returns:
        A text that the Open Jtalk Engine can read.
    """
    text = re.sub(r' ', '、', text)  # 半角スペース
    text = re.sub(r'　', '、', text)  # 全角スペース
    text = re.sub(r'\n', '。', text)  # 改行
    text = re.sub(r'ゝ', 'ー、', text)  # 繰り返し記号
    text = re.sub(r'」', '、', text)  # かぎかっこ
    text = re.sub(r'「', '、', text)  # かぎかっこ
    text = re.sub(r'！', 'っ。', text)  # びっくり
    text = re.sub(r'!', 'っ。', text)  # びっくり
    text = re.sub(r'。、', '。', text)  # その他
    text = re.sub(r'。。', '。', text)  # その他
    text = re.sub(r'。 ', '。', text)  # その他
    text = re.sub(r'。　', '。', text)  # その他
    text = re.sub(r'―', '', text)  # その他
    text = re.sub(r'\.', '。', text)  # 句読点
    text = re.sub(r'．', '。', text)  # 句読点
    text = re.sub(r',', '、', text)  # 句読点
    text = re.sub(r'，', '、', text)  # 句読点
    text = re.sub(r'。\\1', '。', text)  # 複数連続
    text = re.sub(r'[!-/:-@[-`{-~]', '、', text)  # 半角記号,数字,英字
    text = re.sub(r'[︰-＠]', '、', text)  # 全角記号
    text = re.sub(r'、\\1', '、', text)  # 複数連続
    text += '〜っ、。'
    return text


def main() -> None:
    """Test speech via command line."""
    print('文字を入力してください。(Ctrl + d で入力完了)')
    text = sys.stdin.read()
    simple_jtalk(text)


if __name__ == '__main__':
    main()
