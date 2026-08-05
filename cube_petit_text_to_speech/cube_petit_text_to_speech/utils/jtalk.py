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

import functools
import os
import pathlib
import re
import subprocess
import sys

from beartype import beartype

__all__ = ['LIB_ROUTE', 'simple_jtalk', 'generate_jtalk_command', 'adjust_text']  # noqa: F822 (LIB_ROUTE is lazy)

OUTPUT_FILE = pathlib.Path('/tmp/jtalk_output.wav')

# Per-robot voice presets for the ROSConJP conversation demo (2026-08-04), so a
# listener can tell orange/pink/violet apart by voice alone. Each entry is
# (semitone_shift, speed_scale):
#   - semitone_shift: additional half-tone pitch shift, fed to open_jtalk's
#     `-fm` option (0.0 = unchanged). This is a real average-pitch shift, unlike
#     the Speech.action `pitch` field, which actually controls `-jf` (GV weight
#     for log F0 / intonation dynamics), not average pitch -- see
#     resolve_voice_params() and generate_jtalk_file() below.
#   - speed_scale: multiplier applied on top of each utterance's own speed
#     (1.0 = unchanged).
# 'default' is orange's baseline and is byte-for-byte identical to the
# pre-2026-08 behavior. pink/violet values are chosen to match each
# individual's personality.yaml (2026-08-03, ありさん指示): pink is
# おっとりマイペース -> a bit higher & slower (soft); violet is
# いたずら好き・やんちゃ -> higher & a bit faster (playful/brisk). These are a
# starting point to be confirmed and tuned by ear on real hardware; see
# cube_petit_text_to_speech/README.md for how to retune them.
VOICE_PRESETS: dict[str, tuple[float, float]] = {
    'default': (0.0, 1.0),
    'pink': (1.5, 0.90),
    'violet': (3.0, 1.10),
}

# Speech.action宣言のEMOTION_*定数(happiness/default/anger/shout/sadness)は、
# 実際のjtalk音声ファイル(speech_lib/.../Voice/mei/mei_<name>.htsvoice、
# happy/normal/angry/bashful/sadのみ存在)の語彙と一致していない。ここで変換する。
# shoutに対応する音声ファイルは無いため、一番近いangryにフォールバックする。
# Speech.action's declared EMOTION_* constants (happiness/default/anger/shout/
# sadness) don't match the jtalk voice files' own vocabulary (speech_lib/.../
# Voice/mei/mei_<name>.htsvoice only has happy/normal/angry/bashful/sad).
# Bridge the two here. shout has no dedicated voice file, so it falls back to
# the closest match, angry.
_EMOTION_ALIASES: dict = {
    'happiness': 'happy',
    'default': 'normal',
    'anger': 'angry',
    'sadness': 'sad',
    'shout': 'angry',
}


def normalize_emotion(emotion: str) -> str:
    """Map a Speech.action EMOTION_* constant to the jtalk voice file vocabulary.

    Args:
        emotion: Emotion string from a speech goal (either a Speech.action
            EMOTION_* constant, e.g. ``happiness``, or a jtalk voice file name
            already, e.g. ``happy``).

    Returns:
        The jtalk voice file vocabulary name (happy/normal/angry/bashful/sad).
        Values not found in the alias table pass through unchanged, so
        :func:`check_goal` can still reject genuinely unknown emotions.
    """
    return _EMOTION_ALIASES.get(emotion, emotion)


@beartype
def resolve_voice_params(preset: str, semitone_shift_param: float, speed_scale_param: float) -> tuple[float, float]:
    """Resolve the effective (semitone_shift, speed_scale) for a robot's voice.

    Combines a named preset (see VOICE_PRESETS) with additive/multiplicative
    node parameters, so per-utterance tuning ("nudge pink a bit higher") is
    possible without editing the preset table. Both parameters are neutral at
    their defaults (0.0 / 1.0), so preset='default' with default params
    reproduces the pre-existing (unshifted, unscaled) behavior exactly.

    Args:
        preset: Name from VOICE_PRESETS. Unknown names fall back to 'default'.
        semitone_shift_param: Extra half-tone shift added on top of the preset's
            own semitone_shift. 0.0 = no adjustment.
        speed_scale_param: Extra multiplier applied on top of the preset's own
            speed_scale. 1.0 = no adjustment.

    Returns:
        (semitone_shift, speed_scale) to use for this robot's speech.
    """
    preset_semitone_shift, preset_speed_scale = VOICE_PRESETS.get(preset, VOICE_PRESETS['default'])
    return preset_semitone_shift + semitone_shift_param, preset_speed_scale * speed_scale_param


@functools.lru_cache(maxsize=1)
def _lib_route() -> str:
    """Return the jtalk library path lazily (requires a ROS environment)."""
    from ament_index_python import get_package_share_directory
    return f'{get_package_share_directory("cube_petit_text_to_speech")}/speech_lib/'


def __getattr__(name: str) -> str:
    """Provide the legacy module-level ``LIB_ROUTE`` constant lazily."""
    if name == 'LIB_ROUTE':
        return _lib_route()
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


@beartype
def simple_jtalk(text: str) -> None:
    """Execute jtalk speech without specifying parameters aside the speech phrase.

    Args:
        text: Phrase to be said.
    """
    if text == '':
        return
    from cube_petit_speech_msgs.action import Speech
    generate_jtalk_file(text, Speech.Goal.EMOTION_DEFAULT, 100, 100)
    subprocess.Popen(generate_jtalk_command(OUTPUT_FILE),
                     stdin=subprocess.PIPE,
                     stdout=subprocess.PIPE,
                     shell=False,
                     preexec_fn=os.setsid)


@beartype
def check_goal(text: str, emotion: str, emotion_level: int, pitch: int, speed: int, volume: int) -> bool:
    """Validate speech goal values.

    Args:
        text: Phrase to be said.
        emotion: Emotion of the phrase to be played.
        emotion_level: Emotion level. Value between 1 and 5.
        pitch: Pitch of the phrase to be played. Value between 50 and 199.
        speed: Speed of the phrase to be played. Value between 50 and 299.
        volume: Volume of the phrase to be played. Value between 1 and 100.

    Returns:
        True if all the values are valid.
    """
    # text
    if not text:
        return False
    valid_emotions = {'happy', 'normal', 'angry', 'bashful', 'sad'}
    if normalize_emotion(emotion) not in valid_emotions:
        return False
    if not (1 <= emotion_level <= 5):
        return False
    if not (50 <= pitch < 200):
        return False
    if not (50 <= speed < 300):
        return False
    if not (1 <= volume <= 100):
        return False

    return True


@beartype
def generate_jtalk_file(text: str,
                        emotion: str,
                        pitch: int,
                        speed: int,
                        file_path: pathlib.Path | None = None,
                        semitone_shift: float = 0.0,
                        voice_name: str = 'mei') -> pathlib.Path:
    """Generate jtalk audio file.

    Args:
        text: Phrase to be said.
        emotion: Emotion of the phrase to be played.
        pitch: Weight of GV (global variance) for log F0, i.e. intonation
            dynamics/expressiveness (open_jtalk `-jf`). Despite the name, this
            does not shift the average pitch -- use semitone_shift for that.
            Value between 50 to 200.
        speed: Speed of the phrase to be played. Value between 50 and 400.
        file_path: Path to save mp3 audio file.
        semitone_shift: Additional half-tone pitch shift (open_jtalk `-fm`),
            the actual average-pitch control. 0.0 (default) reproduces the
            pre-existing behavior exactly. See VOICE_PRESETS/resolve_voice_params
            for the per-robot presets built on top of this.
        voice_name: htsvoice model folder name under
            MMDAgent_Example-1.6/Voice/ (default 'mei', the only model
            bundled today). Exposed so a differently-voiced htsvoice set can
            be swapped in later without code changes, as long as it follows
            the same `<voice_name>/<voice_name>_<emotion>.htsvoice` layout.

    Returns:
        Path to the generated audio file.
    """
    lib_route = _lib_route()
    text = adjust_text(text)
    echo = f'echo {text} | '
    open_jtalk = f'{lib_route}open_jtalk-1.11/bin/open_jtalk '
    dic = f'-x {lib_route}open_jtalk_dic_utf_8-1.11 '
    htsvoice = (f'-m {lib_route}MMDAgent_Example-1.6/Voice/{voice_name}/'
                f'{voice_name}_{normalize_emotion(emotion)}.htsvoice ')
    speed_param = f'-r {float(speed) / 100} '
    intonation = f'-jf {float(pitch) / 100} '
    pitch_shift = f'-fm {float(semitone_shift)} '
    if file_path is None:
        file_path = OUTPUT_FILE
    # After trimming leading/trailing silence, prepend a ~1s quiet pink-noise
    # "breath" as an amp wake-up cue. The speaker amps have a signal-detect
    # standby that swallows ~1s of audio whenever a robot has been silent for a
    # while (each robot's amp re-sleeps between its own turns), and digital
    # silence cannot wake them, so plain padding does not help (2026-08-03
    # booth finding; a continuous sub-audible keep-alive tone was audible on
    # these small speakers and got rejected). Most of the noise is eaten during
    # wake-up; only a faint short "shh" leaks right before the speech.
    raw_path = f'{file_path}.raw.wav'
    wake_path = f'{file_path}.wake.wav'
    sox = (f'sox -t wav - -p silence 1 0.1 0.1% reverse | '
           f'sox -p -t wav {raw_path} silence 1 0.1 0.1% reverse pad 0 0.1 && '
           f'sox -n -r $(soxi -r {raw_path}) -c $(soxi -c {raw_path}) -b $(soxi -b {raw_path}) '
           f'{wake_path} synth 1.0 pinknoise vol 0.04 fade t 0.05 1.0 0.2 pad 0 0.1 && '
           f'sox {wake_path} {raw_path} {str(file_path)}')
    outwav = f'-ow /dev/stdout | {sox}'
    subprocess.run(echo + open_jtalk + dic + htsvoice + speed_param + intonation + pitch_shift + outwav,
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
