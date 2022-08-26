#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from cube_speech.srv import CallSpeech
import subprocess
from datetime import datetime
import struct, time
from std_msgs.msg import Int16, String
import os
import io
import signal

rospy.init_node('cube_petit_speech_to_text')
rate = rospy.Rate(1) # 1hz

# /julius_result_textのトピックにテキストをPublishする
publish = rospy.Publisher('/julius_result_text', String, queue_size=1)

class juliusSpeechToText:
    def __init__(self):
        self.pkill_julius()
        self.startListening()
    def __del__(self):
        self.stopListening()
        self.pkill_julius()
    def startListening(self):
        cmd = 'cd ~/lib/ssr-kit-v4.5;bash run-linux.sh'
        self.julius_process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,shell=True)
        rospy.loginfo('julius:started\n'), 

    def stopListening(self):
        rospy.loginfo('julius:successfully finished\n')
        self.julius_process.kill()
    def pkill_julius(self):
        p = subprocess.Popen(['pgrep', '-l' , 'julius'], stdout=subprocess.PIPE)
        out, err = p.communicate()
        for line in out.splitlines():        
            line = bytes.decode(line)
            pid = int(line.split(None, 1)[0])
            os.kill(pid, signal.SIGKILL)


    def textStreaming(self):
        for line in iter(self.julius_process.stdout.readline, ""):
            out_string = line.rstrip('\n')
            # print(out_string)
            find_text_flag = out_string.find('sentence1:  ')
            find_short_text_flag = out_string.find('<input rejected by short input>')
            if find_text_flag != -1:
                text_string = out_string.lstrip('sentence1:  ')
                if text_string != '。' and text_string != '' and text_string != '\n':
                    # None
                    rospy.loginfo('julius_text:'+ text_string)
                    publish.publish(text_string)
            if find_short_text_flag != -1:
                text_string = out_string.rstrip('<input rejected by short input>')
                text_string = text_string.lstrip('pass1_best:  ')
                if text_string != '。' and text_string != '' and text_string != '\n':
                    # None
                    rospy.loginfo('julius_text:'+ text_string)
                    publish.publish(text_string)

def callback(data):
    rospy.loginfo('Julius Start: 音声認識結果だよ')
    publish.publish(u'音声認識結果だよ')
    # status = os.system('amixer -D pulse sset Capture 65536')
    # status = os.system('amixer -D pulse sset Capture 0')

def main():
    julius = juliusSpeechToText()
    print("Juliusによる音声認識を開始")
    status = os.system('amixer -D pulse sset Capture 0')
    print('マイクオフ:'+ status)
    julius.textStreaming()
    rospy.spin()

if __name__ == '__main__':
    main()