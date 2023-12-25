#!/usr/bin/env python
# coding: UTF-8

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
#
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_msgs.msg import Float32

from omron_env_sensor_msgs.msg import DataShort

import re

# module_name, package_name, ClassName, method_name,
# ExceptionName, function_name, GLOBAL_CONSTANT_NAME
# global_var_name, instance_var_name, function_parameter_name, local_var_name.

# init [TODO] Class
rospy.loginfo("------------------------------------------")
rospy.init_node('cube_petit_text_to_speech')

rospy.wait_for_service('speech_service')
try:
    speaker = rospy.ServiceProxy('speech_service', CallSpeech)
except rospy.ServiceException:
    rospy.loginfo("Fail.")

# 1Hz
rate = rospy.Rate(10)

# instance_var_name [TODO] param or smtng easy
emotion = 'happiness'
pitch = 130
speed = 100
volume = 100
speech_method = 'jtalk'


# Class method1: text_to_jtalk
def text_to_jtalk(phrase):
    emotion = 'happiness'
    rospy.loginfo('Speech Happily: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed, 100).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)


def text_to_jtalk_sadness(phrase):
    emotion = 'sadness'
    rospy.loginfo('Speech Sadly: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed, 100).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)

def text_to_jtalk_shout(phrase):
    emotion = 'shout'
    rospy.loginfo('Speech Shout: ' + phrase)
    result = speaker(
        phrase, speech_method, emotion, None, pitch, speed, 100).result
    if not result:
        rospy.loginfo('Speech failed for some unknown reason')
    else:
        rospy.loginfo('Robot speech: {}'.format(phrase))
    # rospy.sleep(2.0)


julius_text = ''
battery_voltage = 0
battery_remaining = 0
hight_height_flag = 0
low_height_flag = 0
middle_height_flag = 0

true_count = 0
false_count = 0
hotword_flag = 1
motion_sensor_flag = 0
imu_flag = 0
mute_flag = 0

# temperature 温度(-10~60度)
# relative_humidity 湿度(30~85%)
# ambient_light 照度(10~2000lx)
# barometric_pressure 気圧(700~1100hPa)
# sound_noise 騒音(40~94dB)
# etvoc (0~29206ppb)室内環境下における総揮発性有機化合物濃度
# eco2 (ppm)
# discomfort_index 不快指数
# heat_stroke 熱中症警戒度

temperature = 0.0
# if 50 over , its too hot
relative_humidity = 0.0
ambient_light = 0
barometric_pressure = 0.0
sound_noise = 0.0
etvoc =  0.0
eco2 = 0.0
discomfort_index = 0.0
heat_stroke =  0.0

get_julius_text_flag = 0
body_temperature_high_flag = 0

def env_sensor_callback(data):
    global temperature
    temperature = data.temperature
    global relative_humidity
    relative_humidity = data.relative_humidity
    global ambient_light
    ambient_light = data.ambient_light
    global barometric_pressure
    barometric_pressure = data.barometric_pressure
    global sound_noise
    sound_noise = data.sound_noise
    # etvoc =  data.etvoc
    #eco2 = data.eco2
    # discomfort_index = data.discomfort_index
    # heat_stroke =  data.heat_stroke
    global body_temperature_high_flag

    if temperature > 50:
        if body_temperature_high_flag == 0:
            text_to_jtalk("発熱しています、冷やしてください")
        body_temperature_high_flag = 1
    if temperature < 35:
        body_temperature_high_flag = 0


def julius_callback(phrase):
    global julius_text
    julius_text = phrase.data
    rospy.loginfo("[jtalk.py(julius_callback)]:"+ julius_text)

def imu_callback(data):
    global hight_height_flag
    global low_height_flag
    global imu_flag
    if(imu_flag > 0):
        if(data.data > 50 and hight_height_flag == 0):
            text_to_jtalk("高いよう")
            hight_height_flag = 1
        elif(data.data < -50 and  low_height_flag == 0):
            text_to_jtalk("低いよう")
            low_height_flag = 1
        elif(data.data <= 30 and data.data >= -30):
            hight_height_flag = 0
            low_height_flag = 0


def motion_sensor_callback(data):
    global true_count
    global false_count
    global motion_sensor_flag
    if(motion_sensor_flag > 0):
        if(data.data == True):
            true_count = true_count + 1
            if( true_count > 20 and false_count > 0):
                true_count = 0
                false_count = 0
                text_to_jtalk("やっほー！来てくれてありがとう、仲良くしてね")
        else:
            false_count = false_count + 1

def battery_callback(data):
    global battery_remaining
    battery_remaining = data.data

def battery_voltage_callback(data):
    global battery_voltage
    battery_voltage = data.data

def recog_and_talk(arg_text):
    global hotword_flag
    global julius_text
    global temperature
    rospy.loginfo("[jtalk.py] recog_and_talk")
    talk_text = ''

    if "コントローラ" in arg_text:
        battery_status_result = subprocess.check_output('upower --show-info /org/freedesktop/UPower/devices/gaming_input_sony_controller_battery_88o03o4co2bo6fo06 | grep percentage| sed -e \'s/[^0-9]//g\'', shell=True)
        num_str = str(battery_status_result)
        num_str = re.sub(r'\D', '', num_str)
        if num_str != '':
            talk_text = "コントローラのバッテリ残量は" + num_str + "パーセントです"
            text_to_jtalk(talk_text)
        else:
            talk_text = "コントローラが繋がっていません"
            text_to_jtalk(talk_text)
    elif ("何時" in arg_text) or ("時刻" in arg_text) or ("時間" in arg_text):
        d = datetime.now()
        talk_text = '今は%s月%s日、%s時%s分%s秒です。' % (d.month, d.day, d.hour, d.minute, d.second)
        text_to_jtalk(talk_text)
    elif "名前" in arg_text:
        text_to_jtalk("僕の名前はキューブプチです。あなたの名前はなんですか")
        julius_text = ''
        rospy.sleep(5.5)
        if julius_text != '':
            talk_text = julius_text + "さんですね。素敵な名前ですね。"
            text_to_jtalk(talk_text)
        else:
            text_to_jtalk("わかりませんでした。もう一度話しかけてね")
            rospy.loginfo('[WARN] julius_text is none ')
    elif ("バッテリ残量" in arg_text) or ("バッテリ" in arg_text):
        talk_text = "バッテリの残量は" + str(battery_remaining) + "パーセントです"
        text_to_jtalk(talk_text)
    elif ("バッテリ電圧" in arg_text):
        talk_text = "バッテリの電圧は" + str(battery_voltage) + "です"
        text_to_jtalk(talk_text)
    elif ("体内温度" in arg_text):
        talk_text = "体内温度は" + str(temperature) + "度です"
        text_to_jtalk(talk_text)
    elif ("扇風機" in arg_text) or ("ファン" in arg_text)or ("サーキュレーター" in arg_text):
        talk_text = "おっけー"
        text_to_jtalk(talk_text)
        publish.publish(True)
    elif ("自己紹介して" in arg_text):
        talk_text = "僕は、ロス対応の自律走行ロボットキット、キューブプチです。大学での研究用や教育用に安価で小型に開発しています。よろしくね"
        text_to_jtalk(talk_text)
        publish.publish(True)
    elif ("充電ドック" in arg_text) or ("充電ドッグ" in arg_text) or ("充電して" in arg_text):
        talk_text = "充電ドッグに戻ります"
        text_to_jtalk(talk_text)
        publish_charge.publish(1)
    else:
        talk_text = julius_text + "ですね？"
        text_to_jtalk(talk_text)
    julius_text = ''

def hotword_callback(data):
    global hotword_flag
    rospy.loginfo("hotword_callback")
    global julius_text

    if hotword_flag == 1:
        hotword_flag = 0
        if str(data.data) == "Cuboid":
            text_to_jtalk('僕の名前はキューブプチです')
        elif str(data.data) == "Cube-petit":
            text_to_jtalk('はーい！なんですか？')
            julius_text = ''
            rospy.sleep(3.0)
            count = 0
            listen_flag = 0
            while count < 5:
                count += 1
                rospy.sleep(1.0)
                if julius_text != '':
                    listen_flag = 1
                    rospy.loginfo('julius_text:'+julius_text)
                    recog_and_talk(julius_text)
                    break
            if listen_flag == 0:
                text_to_jtalk("わかりませんでした。もう一度話しかけてね")
            julius_text != ''
            rospy.loginfo('[WARN] julius_text is none ')
        hotword_flag = 1





# def env_sensor_callback(data):
#     print("atui")

# Class method2: joyCallback [TODO] MUTEX
def callback(data):
    global julius_text
    global motion_sensor_flag
    global imu_flag
    global battery_voltage
    global hotword_flag
    global mute_flag
    if data.buttons[0] == 1:      # batu
        rospy.loginfo("X")
        text_to_jtalk('こんにちは')
    # elif data.buttons[1] == 1:    # maru
        # rospy.loginfo("○")
    elif data.buttons[2] == 1:    # sankaku
        rospy.loginfo("△")
        text_to_jtalk('はいどうも')
        # rate.sleep()
    elif data.buttons[3] == 1:    # sikaku
        rospy.loginfo("□")
        text_to_jtalk('ハローワールド！僕の名前はキューブプチです！')
        # rate.sleep()
    elif data.buttons[4] == 1:    # L1
        rospy.loginfo("L1")
        text_to_jtalk('こっち来てえ')
    elif data.buttons[5] == 1:    # R1
        rospy.loginfo("R1")
        text_to_jtalk('ソラジロー先輩、どこですか〜？')
    elif data.buttons[6] == 1:    # L2
        rospy.loginfo("L2")
    elif data.buttons[7] == 1:    # R2
        rospy.loginfo("R2")
    elif data.buttons[8] == 1:    # SHARE
        rospy.loginfo("SHARE")
        text_to_jtalk('好きな野菜はなんですか〜？')
        text_to_jtalk('ポンッ！')

        hotword_flag = 0
        julius_text = ''
        yasai_text = ''
        rospy.sleep(5.5)
        julius_text = julius_text.rstrip('です')
        julius_text = julius_text.rstrip('だよ')
        if julius_text != '':
            if julius_text == "キューブプチ":
                text_to_jtalk("僕は食べられないよう！")
            elif julius_text == "キューボイド":
                text_to_jtalk("キューボイドは僕のお兄ちゃんの名前ですよ")
            elif julius_text == "大根" or julius_text == "じゃがいも" or julius_text == "アスパラガス" or julius_text == "たけのこ":
                # レンコン かぼちゃ アスパラ ブロッコリー
                # アボカド
                # たまねぎ
                # じゃがいも
                # そらまめ
                # とうもろこし
                # 白菜
                # キャベツ
                # ピーマン
                # ナス
                # 筍
                # そら豆
                # えんどう豆
                # オクラ
                yasai_text = julius_text + "は僕も大好きです！"
                text_to_jtalk(yasai_text)
            else:
                yasai_text = julius_text + "ですね？"
                text_to_jtalk(yasai_text)
            julius_text = ''
        else:
            text_to_jtalk_sadness("わかりませんでした")
        hotword_flag = 1


    elif data.buttons[9] == 1:    # OPTION
        hotword_flag = 0
        text_to_jtalk('音声認識モード。ポンッ！')
        julius_text = ''
        rospy.sleep(3.0)
        count = 0
        listen_flag = 0
        while count < 5:
            count += 1
            rospy.sleep(1.0)
            if julius_text != '':
                listen_flag = 1
                rospy.loginfo('julius_text:'+julius_text)
                recog_and_talk(julius_text)
                break
        if listen_flag == 0:
            text_to_jtalk("わかりませんでした。もう一度話しかけてね")
        julius_text != ''
        rospy.loginfo('[WARN] julius_text is none ')
        hotword_flag = 1
    elif data.buttons[10] == 1:    # PS
        rospy.loginfo("PS Button")
        if mute_flag == 0:
        # マイクミュート [TODO]ミュート　ミュート解除
            text_to_jtalk("マイクをミュートします")
            status = os.system('amixer -D pulse sget Capture')
            rospy.loginfo('mute mic')
            rospy.loginfo(str(status))
            rospy.loginfo(status)
            mute_flag = 1
        elif mute_flag == 1:
            text_to_jtalk("マイクのミュートを解除します")
            status = os.system('amixer -D pulse sset Capture 80%')
            mute_flag = 0
    elif data.buttons[11] == 1:    # Left Stick
        rospy.loginfo("Left Stick")
        if(imu_flag==0):
            text_to_jtalk("高さ検知機能オン")
            imu_flag = 1
        else:
            text_to_jtalk("高さ検知機能オフ")
            imu_flag = 0
    elif data.buttons[12] == 1:    # Right Stick
        rospy.loginfo("Right Stick")
        if(motion_sensor_flag==0):
            text_to_jtalk("モーションセンサーオン")
            motion_sensor_flag = 1
        else:
            text_to_jtalk("モーションセンサーオフ")
            motion_sensor_flag = 0
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
# phrase = 'テストテスト。おはようございます。'
# result = speaker(
#     phrase, speech_method, emotion, None, pitch, speed, volume).result
# if not result:
#     rospy.loginfo('Speech failed for some unknown reason')
# else:
#     rospy.loginfo('Robot speech: {}'.format(phrase))

# # talk daytime
# d = datetime.now()
# phrase1 = '今は%s月%s日、%s時%s分%s秒です。起動しました' % (d.month, d.day, d.hour, d.minute, d.second)
# result = speaker(
#     phrase1, speech_method, emotion, None, pitch, speed, volume).result
# if not result:
#     rospy.loginfo('Speech failed for some unknown reason')
# else:
#     rospy.loginfo('Robot speech: {}'.format(phrase1))

rospy.Subscriber("/joystick/joy",Joy,callback, queue_size=1)
rospy.Subscriber("/imu_height",Int32,imu_callback, queue_size=10)
rospy.Subscriber("/julius_result_text", String, julius_callback, queue_size=1)
rospy.Subscriber("/motion_sensor", Bool, motion_sensor_callback, queue_size=1)
rospy.Subscriber("/gazebo_battery_monitor/battery_remaining", Float64, battery_callback, queue_size=1)
rospy.Subscriber("/gazebo_battery_monitor/battery_voltage", Float64, battery_voltage_callback, queue_size=1)
rospy.Subscriber("/omron_env_sensor", DataShort, env_sensor_callback, queue_size=1)
rospy.Subscriber("/hotword_detector/detect_word", String, hotword_callback, queue_size=1)

publish = rospy.Publisher('/fan_on', Bool, queue_size=1)
publish_charge = rospy.Publisher('/do_charge', Int8, queue_size=1)
# status = os.system('amixer -D pulse sset Capture 0')
# rospy.loginfo('mute mic')


rospy.spin()
