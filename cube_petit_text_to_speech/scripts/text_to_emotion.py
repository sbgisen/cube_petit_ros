#!/usr/bin/env python
# coding: UTF-8

import json
import os
import re
import struct
import subprocess
import sys
import time
from datetime import datetime
from tokenize import String

import rospy
from cube_speech.srv import CallSpeech
from mlask import MLAsk
#
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String

# module_name, package_name, ClassName, method_name,
# ExceptionName, function_name, GLOBAL_CONSTANT_NAME
# global_var_name, instance_var_name, function_parameter_name, local_var_name.

# init
rospy.loginfo("------------------------------------------")
rospy.init_node('cube_petit_text_to_emoiton')

emotion_analyzer = MLAsk()

# wait for service
rospy.wait_for_service('speech_service')
try:
    speaker = rospy.ServiceProxy('speech_service', CallSpeech)
except rospy.ServiceException:
    rospy.loginfo("Fail.")

# 1Hz
rate = rospy.Rate(10)

# global var
julius_text = ''
emotion_flag = 1


def text_to_emotion_callback(data):
    if data.data:
        emotion_flag = 1
    elif data.data:
        emotion_flag = 0


def julius_callback(phrase):
    global julius_text
    julius_text = phrase.data
    if emotion_flag:
        result_json = emotion_analyzer.analyze(julius_text)
        parsed_data = json.dumps(result_json, indent=4)
        rospy.loginfo(result_json)
        if "emotion" in parsed_data:
            text = str(result_json["emotion"])
            start_index = text.find('{')
            if start_index != -1:
                result = text[start_index:]  # '{' から末尾までの部分を取得
                # rospy.loginfo(result)  # '{' から始まる部分が出力されます
                match = re.search(r"'([^']+)'", result)
                if match:
                    extracted_string = match.group(1)
                    # rospy.loginfo(extracted_string)
                    publish_emotion.publish(extracted_string)
        if "activation" in result_json:  # 'NEUTRAL' 'mostly_ACTIVE' 'ACTIVE' 'mostly_PASSIVE' 'PASSIVE'
            # rospy.loginfo(str(result_json["activation"]))
            publish_activation.publish(str(result_json["activation"]))
        if "orientation" in result_json:  # 'NEUTRAL' 'POSITIVE' 'NEGATIVE' 'mostly_POSITIVE' 'mostly_NEGATIVE'
            # rospy.loginfo(str(result_json["orientation"]))
            publish_orientation.publish(str(result_json["orientation"]))


# start
rospy.Subscriber("/julius_result_text", String, julius_callback, queue_size=1)
rospy.Subscriber("/text_to_emotion_flag", Bool, text_to_emotion_callback, queue_size=1)

# 'yorokobi' 'ikari' 'aware' 'kowagari' 'haji' 'suki' 'iya' 'takaburi' 'yasuragi' 'odoroki'
publish_emotion = rospy.Publisher('/talk_to_emotion/emotion', String, queue_size=1)

publish_activation = rospy.Publisher('/talk_to_emotion/activation', String, queue_size=1)
publish_orientation = rospy.Publisher('/talk_to_emotion/orientation', String, queue_size=1)

rospy.spin()
