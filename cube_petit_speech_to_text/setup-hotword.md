## Hotword(EfficientWord-Net)

### Apt Install

```bash
sudo apt install -y awscli portaudio19-dev jq gnustep-gui-runtime
```

### Install EfficientWord-Net

- pip install

```bash
pip install tflite-runtime
pip install EfficientWord-Net
```

## EfficientWord-Net Quick Test

```bash
python -m eff_word_net.engine
```

## Learn hotword model(Ubuntu22未確認)

**1. Create Audio Data by using AWS Polly**<br>

Run this Script

```bash
#!/bin/bash

WORD="<KEYWORD IN JAPANESE>"
AUDIO_OUT="<PATH_TO_AUDIO_OUT>"
aws polly describe-voices | jq -r '.Voices[] | select(.LanguageName=="Japanese") | [.Id, .SupportedEngines[0]] | @tsv' | while read voice_id voice_engine; do aws polly synthesize-speech --text "$WORD" --engine "${voice_engine}" --voice-id "${voice_id}" --output-format mp3 --sample-rate 16000 ${AUDIO_OUT}/${voice_id}.mp3;done
```

**2. Generate KEYWORD.json**

```bash
PATH_TO_REF_OUT=`rospack find speech_recognition_ros`/resources/efficient_word_net
KEYWORD="<KEYWORD_ID>"
python -m eff_word_net.generate_reference --input-dir ${AUDIO_OUT}  --output-dir ${PATH_TO_REF_OUT} --wakeword ${KEYWORD} --model-type resnet_50_arc
mv ${PATH_TO_REF_OUT}/${KEYWORD}_ref.json ${PATH_TO_REF_OUT}/${KEYWORD}.json
```

## Launch

```bash
roslaunch speech_recognition_ros hotword_detector.launch
```
