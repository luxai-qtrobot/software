#!/usr/bin/env python3
import os
import queue
import sounddevice as sd
import vosk
import sys
import time
import rospy
import json
import usb.core
import usb.util
import struct
import pvporcupine

from tuning import Tuning
from std_msgs.msg import String


HOTWORDS = ['/home/qtrobot/Downloads/Hey-QT_en_raspberry-pi_v2_1_0.ppn']
ACEES_KEY = "K8eJOJwOEKK0AxUzCVy8Rd2lV6uVgxcvsFkEMt561J+kwP8Ng1V9eg=="




class PorcupineHotWord(object):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self, keyword_paths, access_key):

        # find respeaker mic
        self.device_index  = self.get_respeaker_device_index()
        if not self.device_index  :
            rospy.logfatal("could not find Respeack microphone device")
            raise Exception('device')

        
        # open mic audio device
        device_info = sd.query_devices(self.device_index, 'input')

        # Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher 
        # sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5
        # will be used.
        self.ppn = pvporcupine.create(keyword_paths=keyword_paths, access_key=access_key, sensitivities=[0.7])        

        # self.audio_pub = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)

    """
        Get ReSpeaker microphone device
    """
    def get_respeaker_device_index(self):
        devices = sd.query_devices()
        index = 0
        for dev in devices:
            if "ReSpeaker" in dev['name']:            
                return index
            index = index + 1
        return None
    

    def stream_callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            return 
        audio_frame = struct.unpack_from("h" * self.ppn.frame_length, indata)
        result = self.ppn.process(audio_frame)
        if result >= 0:
            print('Detected...')
            os.system('play /home/qtrobot/robot/data/audios/QT/bell.wav >/dev/null 2>&1')


    def run(self):        
        with sd.RawInputStream(samplerate=self.ppn.sample_rate, 
                                blocksize=self.ppn.frame_length,
                                device=self.device_index, 
                                dtype='int16', 
                                channels=1, 
                                callback=self.stream_callback):
            while not rospy.is_shutdown():
                time.sleep(1)


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return
    return Tuning(dev)


if __name__ == "__main__":
    rospy.init_node('qt_hotwrod_app')
        
    # adjusting the Respeaker params for better ASR 
    rospy.loginfo("adjusting the Respeaker params for better ASR (noise suppression)")

    dev = find()
    if not dev:
        rospy.logfatal("could not open Respeack microphone for tunning")
        sys.exit(1)
        
    dev.write("AGCONOFF", 0)
    dev.write("AGCGAIN", 15.0)
    dev.write("STATNOISEONOFF_SR", 1)
    dev.write("MIN_NS_SR", 0.01)
    dev.write("GAMMA_NS_SR", 1.80)
    dev.write("CNIONOFF", 0)
    dev.close()


    hotword = PorcupineHotWord(HOTWORDS, ACEES_KEY)
    hotword.run()

    rospy.loginfo("qt_hotwrod_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_hotwrod_app shutdown")
        
