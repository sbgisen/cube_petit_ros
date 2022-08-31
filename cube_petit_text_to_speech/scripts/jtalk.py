#!/usr/bin/env python
# coding: UTF-8

import sys
from tokenize import String
import rospy
from cube_speech.srv import CallSpeech
import subprocess
from datetime import datetime
import struct, time
import os
from std_msgs.msg import String

# 
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

# module_name, package_name, ClassName, method_name, 
# ExceptionName, function_name, GLOBAL_CONSTANT_NAME
# global_var_name, instance_var_name, function_parameter_name, local_var_name.

# init [TODO] Class
rospy.loginfo("------------------------------------------1")
rospy.init_node('cube_petit_text_to_speech')

rospy.wait_for_service('speech_server')
try:
    speaker = rospy.ServiceProxy('speech_server', CallSpeech)
except rospy.ServiceException:
    rospy.loginfo("Fail.")

# 1Hz
rate = rospy.Rate(10) 

# instance_var_name [TODO] param or smtng easy
emotion = 'happiness'
pitch = 130
speed = 100
speech_method = 'jtalk'  


# Class method1: text_to_jtalk
def text_to_jtalk(phrase):
    emotion = 'happiness'
    rospy.loginfo('Speech Happily: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)


def text_to_jtalk_sadness(phrase):
    emotion = 'sadness'
    rospy.loginfo('Speech Sadly: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)

def text_to_jtalk_shout(phrase):
    emotion = 'shout'
    rospy.loginfo('Speech Shout: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)


julius_text = ''

hight_height_flag = 0
low_height_flag = 0
middle_height_flag = 0


def julius_callback(phrase):
    global julius_text
    julius_text = phrase.data

def imu_callback(data):
    global hight_height_flag
    global low_height_flag
    if(data.data > 50 and hight_height_flag == 0):
        text_to_jtalk("高いよう")
        hight_height_flag = 1
    elif(data.data < -50 and  low_height_flag == 0):
        text_to_jtalk("低いよう")
        low_height_flag = 1
    elif(data.data <= 30 and data.data >= -30):
        hight_height_flag = 0
        low_height_flag = 0


# Class method2: joyCallback [TODO] MUTEX
def callback(data):
    global julius_text
    if data.buttons[0] == 1:      # batu
        rospy.loginfo("X")
        text_to_jtalk('こんにちは')
    elif data.buttons[1] == 1:    # maru
        rospy.loginfo("○")
    elif data.buttons[2] == 1:    # sankaku
        rospy.loginfo("△")
        text_to_jtalk_shout('ねえねえ')
        # rate.sleep()
    elif data.buttons[3] == 1:    # sikaku
        rospy.loginfo("□")
        text_to_jtalk('こっち来て')
        # rate.sleep()
    elif data.buttons[4] == 1:    # L1
        rospy.loginfo("L1")
    elif data.buttons[5] == 1:    # R1
        rospy.loginfo("R1")
    elif data.buttons[6] == 1:    # L2
        rospy.loginfo("L2")
    elif data.buttons[7] == 1:    # R2
        rospy.loginfo("R2")
    elif data.buttons[8] == 1:    # SHARE
        rospy.loginfo("SHARE")
        text_to_jtalk('好きな野菜はなんですか〜？')
        text_to_jtalk('ポンッ！')
        julius_text = ''
        status = os.system('amixer -D pulse sset Capture 40000')
        rospy.loginfo('unmute mic')
        rospy.sleep(5.5)
        status = os.system('amixer -D pulse sset Capture 0')
        rospy.loginfo('mute mic')
        rospy.sleep(0.5)
        if julius_text != '':
            julius_text = julius_text + "ですね？"
            text_to_jtalk(julius_text)
            julius_text = ''
        else:
            text_to_jtalk_sadness("わかりませんでした")


    elif data.buttons[9] == 1:    # OPTION
        rospy.loginfo("OPTION")
    elif data.buttons[10] == 1:    # PS
        rospy.loginfo("PS Button")
    elif data.buttons[11] == 1:    # Left Stick
        rospy.loginfo("Left Stick")
    elif data.buttons[12] == 1:    # Right Stick
        rospy.loginfo("Right Stick")
    elif data.axes[6] == 1.0:    # Left Cross Button
        rospy.loginfo("Left Cross Button")
        text_to_jtalk('一緒に写真を撮ろう！')
    elif data.axes[6] == -1.0:    # Right Cross Button
        rospy.loginfo("Right Cross Button")
        text_to_jtalk_sadness('お腹が減ったよ、充電したいな')
    elif data.axes[7] == 1.0:    # Up Cross Button
        rospy.loginfo("Up Cross Button")
        text_to_jtalk('おかえりなさい！待ってたよ')
    elif data.axes[7] == -1.0:    # Down Cross Button
        rospy.loginfo("Down Cross Button")
        text_to_jtalk('お疲れ様！今日はどんな一日だった？')


# start 

# talk test
phrase = 'テストテスト。おはようございます。'
result = speaker(
    phrase, speech_method, emotion, None, pitch, speed).result
if not result:
    rospy.loginfo('Speech failed for some unknown reason')
else:
    rospy.loginfo('Robot speech: {}'.format(phrase))

# talk daytime
d = datetime.now()
phrase1 = '今は%s月%s日、%s時%s分%s秒です。起動しました' % (d.month, d.day, d.hour, d.minute, d.second)
result = speaker(
    phrase1, speech_method, emotion, None, pitch, speed).result
if not result:
    rospy.loginfo('Speech failed for some unknown reason')
else:
    rospy.loginfo('Robot speech: {}'.format(phrase1))

rospy.Subscriber("/joystick/joy",Joy,callback, queue_size=1)
rospy.Subscriber("/imu_height",Int32,imu_callback, queue_size=10)
rospy.Subscriber("/julius_result_text", String, julius_callback, queue_size=1) 


status = os.system('amixer -D pulse sset Capture 0')
rospy.loginfo('mute mic')


rospy.spin()