# cube_petit_text_to_speech

OpenJTalk-based TTS: `cube_petit_text_to_jtalk` (joystick/hotword-triggered phrases) and
`speech_action_server` (the `Speech.action` server every speaker -- scenario/interaction
nodes, `cube_petit_python_api`'s `SpeechCommander`, this package's own joystick node --
ultimately calls to actually synthesize and play audio). All speech funnels through
`speech_action_server`, so per-robot voice differentiation lives there rather than in any
individual caller.

## Per-robot voice (orange/pink/violet)

Added for the ROSConJP conversation demo (2026-08-04) so a listener can tell the three
individuals apart by voice alone. `speech_action_server` exposes four ROS parameters:

- `voice_preset` (string, default `'default'`): looked up in
  `utils/jtalk.py`'s `VOICE_PRESETS` table for a baseline `(semitone_shift, speed_scale)`.
  Unknown names (including `'orange'`) fall back to `'default'` -- the unshifted,
  unscaled baseline, byte-for-byte identical to the pre-2026-08 voice.
- `voice_semitone_shift` (float, default `0.0`): an *additional* half-tone pitch shift,
  added on top of the preset's own value and forwarded to OpenJTalk's `-fm` option (the
  actual average-pitch control -- note that `Speech.action`'s own `pitch` field is
  forwarded to `-jf`, OpenJTalk's GV weight for log F0/intonation dynamics, *not* average
  pitch, despite the name).
- `voice_speed_scale` (float, default `1.0`): an additional multiplier on top of the
  preset's own value and of whatever `speed` each individual utterance already requests.
- `voice_name` (string, default `'mei'`): the htsvoice model folder under
  `MMDAgent_Example-1.6/Voice/`. Only `mei` (5 emotion files) is bundled today; swapping
  models requires installing another htsvoice set with the same
  `<voice_name>/<voice_name>_<emotion>.htsvoice` layout (e.g. apt's
  `hts-voice-nitech-jp-atr503-m001`) and is not required to get 3 distinguishable voices,
  since the semitone shift alone already differentiates them.

`voice_preset`/`voice_semitone_shift`/`voice_speed_scale`/`voice_name` are plumbed through
as launch arguments from `launch/cube_petit_text_to_jtalk.launch.py`, and from there through
`cube_petit_bringup.launch.py` (default derived from the hostname, same as `face_color`) and
`cube_petit_bringup/launch/conversation_demo.launch.py` (default `'violet'`, since that
launch file's usual target is violet -- see its module docstring).

### Current presets (`utils/jtalk.py::VOICE_PRESETS`)

Chosen to match each individual's `personality.yaml` (2026-08-03, ありさん指示):

| preset    | semitone_shift | speed_scale | personality                          | voice intent                    |
|-----------|---------------:|------------:|---------------------------------------|----------------------------------|
| `default` | 0.0            | 1.0         | orange, baseline                      | unchanged                        |
| `pink`    | +1.5           | 0.90        | おっとりマイペース (〜だよ〜)          | a bit higher & slower, soft      |
| `violet`  | +3.0           | 1.10        | いたずら好き・好奇心旺盛・やんちゃ (〜だぜ、〜じゃん) | higher & a bit faster, playful/brisk |

These are a starting point, not final -- confirm and retune by ear on real hardware. To
retune, edit `VOICE_PRESETS` directly, or override live without a code change:

```bash
ros2 launch cube_petit_bringup cube_petit_bringup.launch.py \
    voice_preset:=pink voice_semitone_shift:=1.0 voice_speed_scale:=1.0
```

`voice_semitone_shift`/`voice_speed_scale` compose with the preset (additive/multiplicative,
see `resolve_voice_params()`), so the command above nudges `pink` one extra half-tone higher
without touching the table.
