#!/usr/bin/env python
#-*- coding:utf-8 -*-

import pygame
from pygame.locals import *
import sys
import math
import roslib
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int16

WIDTH=1080
HEIGHT=150

FONT = None

rospy.init_node('viewStatus')

class statusView:
    def __init__(self):
        self.viewstatus = rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)
        self.sleeptime= rospy.Subscriber("/round_goal/sleeptime", Int16, self.sleeptime_callback)
        self.status = []
        self.wait_time = 0

    def move_base_status_callback(self, move_base_status):
        if len(move_base_status.status_list) >0:
            endidx = len(move_base_status.status_list) - 1
            self.status =move_base_status.status_list[endidx].status
            # print self.status 

    def sleeptime_callback(self, sleeptime_topic):
        self.wait_time = sleeptime_topic.data
        # print self.wait_time

def blinkMesgWhite(phrase,count,font,screen):
    count =count*5
    c = int(202+53*math.sin(float(count)/1000.0))
    d = int(128+127*math.cos(float(count)/1000.0))
    text = font.render(phrase,True,(d,d,c))
    screen.blit(text,[100,25])

def runningMesg(phrase,count,font,screen):
    count =count*5
    # phrase = STRING3
    text = font.render(phrase,True,(0,0,200))
    x1 = 470+int(300*math.sin(float(count)/1000.0))
    screen.blit(text,[x1,25])

def warningMesg(phrase,count,font,screen):
    count =count*5
    # phrase = STRING3
    text = font.render(phrase,True,(200,0,0))
    x1 = 470+int(300*math.sin(float(count)/1000.0))
    screen.blit(text,[x1,25])

def goalMesg(phrase,count,font,screen):
    count =count*5
    # phrase = STRING3
    c = int(127*(1.3 + 0.7 * math.sin(float(count)/300.0)))
    text = font.render(phrase,True,(0, c, 0))
    # x1 = 670+int(600*math.sin(float(count)/1000.0))
    screen.blit(text,[5,25])

def start():
    pygame.init()
    sv = statusView()
    r = rospy.Rate(60.0)
    screen=pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("Running Stutus")
    font = pygame.font.Font(FONT,120)
    mesg = 0

    counter = 0
    while True:
        key = sv.status
        screen.fill((255,255,255))
        if key == 1:
            phrase = "Running"
            runningMesg(phrase,counter,font,screen)
        elif key== 3:
            phrase = "Goal reach!"
            if sv.wait_time > 0:
                phrase = phrase + " Restart in " + str(sv.wait_time) + "s"
            goalMesg(phrase,counter,font,screen)
        elif key== 4:
            phrase = "Abort"
            warningMesg(phrase,counter,font,screen)
        else: 
            phrase = "Thinking..."
            runningMesg(phrase,counter,font,screen)
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        counter=counter+1
        counter = counter % (65535*256)
        r.sleep()

if __name__=='__main__':
    start()
