#!/usr/bin/env python
# -*- coding: utf-8 -*-

import io
import os
import signal
import struct
import subprocess
import sys
import time
from datetime import datetime

import rospy
from cube_speech.srv import CallSpeech
from std_msgs.msg import Int16
from std_msgs.msg import String

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
        cmd = 'cd ~/lib/julius_libs/dictation-kit/;bash run-linux-dnn.sh'
        self.julius_process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,shell=True)
        rospy.loginfo('julius:started\n')
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
            out_string = line.decode().replace("\n", " ")
            print(out_string)
            # rospy.loginfo('\njulius_text RAW ->:\n' + out_string + '\n<-:julius_text RAW\n' )
            find_text_flag = out_string.find('sentence1:')
            find_pass_flag = out_string.find('pass1_best:')
            find_short_text_flag = out_string.find('<input rejected by short input>')
            if find_text_flag != -1 and find_short_text_flag == -1:
                text_string = out_string.replace("sentence1:", "")
                text_string = text_string.replace(" ", "")
                text_string = text_string.replace("。", "")
                if text_string != '。' and len(text_string) > 3:
                    # rospy.loginfo('\njulius_text: OUTPUT\n'+ text_string + '\n')
                    publish.publish(text_string)
                    print(text_string)
            elif find_pass_flag != -1 and find_short_text_flag != -1:
                # rospy.loginfo('\njulius_text SHORT RAW ->:\n'+ out_string + '\n<-:julius_text SHORT RAW\n')
                text_string = out_string.replace("pass1_best:", "")
                text_string = text_string.replace("<input rejected by short input>", "")
                text_string = text_string.replace(" ", "")
                text_string = text_string.replace("。", "")
                if text_string != '。' and len(text_string) > 3:
                    # rospy.loginfo('\njulius_text SHORT OUTPUT:\n'+ text_string + '\n' + str(len(text_string)) + '\n')
                    publish.publish(text_string)
                    print(text_string)
            text_string=''
            out_string=''
        line=''

def callback(data):
    rospy.loginfo('Julius Start: 音声認識結果だよ')
    publish.publish(u'音声認識結果だよ')

def main():
    julius = juliusSpeechToText()
    rospy.loginfo("Juliusによる音声認識を開始")
#     status = os.system('amixer -D pulse sset Capture 0')
#     rospy.loginfo('マイクオフ:'+ str(status))
    julius.textStreaming()
    rospy.spin()

if __name__ == '__main__':
    main()
