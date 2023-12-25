#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import usb.core
import usb.util
import pyaudio
import wave
import math
import numpy as np
import tf.transformations as T
import os
import rospy
import struct
import sys
from std_msgs.msg import String, Bool, Int32, ColorRGBA
try:
    from pixel_ring import usb_pixel_ring_v2
except IOError as e:
    print(e)
    raise RuntimeError("Check the device is connected and recognized")

class RespeakerInterface(object):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 100000

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VENDOR_ID,
                                 idProduct=self.PRODUCT_ID)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        rospy.loginfo("Initializing Respeaker device")
        self.dev.reset()
        self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
        self.set_led_think()
        time.sleep(10)  # it will take 10 seconds to re-recognize as audio device
        self.set_led_trace()
        rospy.loginfo("Respeaker device initialized (Version: %s)" % self.version)

    def __del__(self):
        try:
            self.close()
        except:
            pass
        finally:
            self.dev = None

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        #response = struct.unpack(b'ii', struct.pack(b'ii',*response))
        response = struct.unpack(b'ii', bytes([x for x in response]))

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])
        #result = response[0]

        return result

    def set_led_think(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.think()

    def set_led_trace(self):
        self.pixel_ring.set_brightness(20)
        self.pixel_ring.trace()

    def set_led_color(self, r, g, b, a):
        self.pixel_ring.set_brightness(int(20 * a))
        self.pixel_ring.set_color(r=int(r*255), g=int(g*255), b=int(b*255))

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


class RespeakerAudio(object):
    def __init__(self, on_audio, channels=None, suppress_error=True):

        # find device
        count = self.pyaudio.get_device_count()
        rospy.logdebug("%d audio devices found" % count)
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = str(info["name"].encode("utf-8"))
            chan = info["maxInputChannels"]
            rospy.logdebug(" - %d: %s" % (i, name))
            if name.lower().find("respeaker") >= 0:
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo("Found %d: %s (channels: %d)" % (i, name, chan))
                break


class RespeakerNode(object):
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)

        self.respeaker = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)
        self.speech_audio_buffer = bytearray()
        self.is_speeching = False
        self.speech_stopped = rospy.Time(0)
        self.prev_is_voice = None
        self.prev_doa = None
        # advertise
        self.pub_vad = rospy.Publisher("is_speeching", Bool, queue_size=1, latch=True)
        self.pub_doa_raw = rospy.Publisher("sound_direction", Int32, queue_size=1, latch=True)
        self.pub_doa = rospy.Publisher("sound_localization", PoseStamped, queue_size=1, latch=True)
        self.pub_audio = rospy.Publisher("audio", AudioData, queue_size=10)
        self.pub_speech_audio = rospy.Publisher("speech_audio", AudioData, queue_size=10)
        self.pub_audios = {c:rospy.Publisher('audio/channel%d' % c, AudioData, queue_size=10) for c in self.respeaker_audio.channels}
        # init config
        self.config = None
        self.dyn_srv = Server(RespeakerConfig, self.on_config)
        # start
        self.speech_prefetch_bytes = int(
            self.speech_prefetch * self.respeaker_audio.rate * self.respeaker_audio.bitdepth / 8.0)
        self.speech_prefetch_buffer = bytearray()
        self.respeaker_audio.start()
        self.info_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate),
                                      self.on_timer)
        self.timer_led = None
        self.sub_led = rospy.Subscriber("status_led", ColorRGBA, self.on_status_led)
        self.big_data0 = []


    def on_status_led(self, msg):
        self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
        if self.timer_led and self.timer_led.is_alive():
            self.timer_led.shutdown()
        self.timer_led = rospy.Timer(rospy.Duration(3.0),
                                       lambda e: self.respeaker.set_led_trace(),
                                       oneshot=True)




if __name__ == '__main__':
    rospy.init_node("respeaker_led_node")
    n = RespeakerNode()
    rospy.spin()
