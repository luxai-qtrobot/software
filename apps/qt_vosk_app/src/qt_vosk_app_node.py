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


from tuning import Tuning

from qt_vosk_app.srv import *


MODELS_PATH = '/home/qtrobot/robot/vosk/models/'
DEFAULT_LANG = 'en_US'


q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


class QTrobotVoskSpeech(object):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self, prefix, language):
        self.prefix = prefix
        self.language = language
        # find respeaker mic
        self.device_index  = self.get_respeaker_device_index()
        if not self.device_index  :
            rospy.logfatal("could not find Respeack microphone device")
            raise Exception('device')

        # open mic audio device
        device_info = sd.query_devices(self.device_index, 'input')
        # soundfile expects an int, sounddevice provides a float:
        self.device_samplerate = int(device_info['default_samplerate'])

        self.model = vosk.Model(MODELS_PATH + self.language)

        # start recognize service
        self.speech_recognize = rospy.Service(prefix+'/recognize', speech_recognize, self.callback_recognize)


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


    """
        ros speech recognize callback
    """
    def callback_recognize(self, req):
        # clear queue
        q.queue.clear()
        print("options:", len(req.options), req.options)
        print("language:", req.language)
        print("timeout:", str(req.timeout))
        timeout = (req.timeout if (req.timeout != 0) else 20)
        language = (req.language if (req.language != '') else self.language)
        
        # check if we need to change the language model
        print('current language: ' + self.language)
        if language != self.language:
            print('switching language to ' + language)
            # VOSK python API does not implement exception!
            # so we need to check the path by ourselves 
            
            if os.path.exists(MODELS_PATH + language):
                self.model = vosk.Model(MODELS_PATH + language)
                self.language = language
            else:
                rospy.loginfo('could not load language model for ' + language)
                return speech_recognizeResponse('')
        
        with sd.RawInputStream(samplerate=self.device_samplerate, blocksize = 8000, device=self.device_index, dtype='int16', channels=1, callback=callback):
            
            rec = vosk.KaldiRecognizer(self.model, self.device_samplerate)
            
            t_start = time.time()
            should_stop = False
            transcript = ''
            while not should_stop:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = rec.Result()
                    # print(result)
                    jres = json.loads(result)
                    transcript = jres['text']
                    for option in req.options:
                        if option.strip() and option in transcript:
                            transcript = option
                    should_stop = True
                else:
                    result = rec.PartialResult()
                    # print(result)
                    jres = json.loads(result)
                    for option in req.options:
                        if option.strip() and option in jres['partial']:
                            transcript = option
                    should_stop = True if transcript else False
                should_stop = should_stop or ((time.time() - t_start) > timeout)

        return speech_recognizeResponse(transcript)



def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return
    return Tuning(dev)


if __name__ == "__main__":
    rospy.init_node('qt_vosk_app')
        
    language = DEFAULT_LANG
    if rospy.has_param('~langauge'):
        language = rospy.get_param('~langauge')
        
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
    
    gspeech = QTrobotVoskSpeech('/qt_robot/speech', language)
    rospy.loginfo("qt_vosk_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_vosk_app shutdown")
    
    dev.close()
