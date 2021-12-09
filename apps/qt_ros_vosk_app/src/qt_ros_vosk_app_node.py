#!/usr/bin/env python3
import os
import queue
import vosk
import time
import rospy
import json

from audio_common_msgs.msg import AudioData
from qt_vosk_app.srv import *


AUDIO_RATE = 16000
MODELS_PATH = '/home/qtrobot/robot/vosk/models/'
DEFAULT_LANG = 'en_US'


q = queue.Queue()

def callback(msg):
    q.put(bytes(msg.data))


class QTrobotVoskSpeech(object):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self, prefix, language):
        self.prefix = prefix
        self.language = language

        # # open mic audio device
        # device_info = sd.query_devices(self.device_index, 'input')
        # # soundfile expects an int, sounddevice provides a float:
        # self.device_samplerate = int(device_info['default_samplerate'])

        self.model = vosk.Model(MODELS_PATH + self.language)

        rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, callback)

        # start recognize service
        self.speech_recognize = rospy.Service(prefix+'/recognize', speech_recognize, self.callback_recognize)

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
        
        # with sd.RawInputStream(samplerate=self.device_samplerate, blocksize = 8000, device=self.device_index, dtype='int16', channels=1, callback=callback):
        
        rec = vosk.KaldiRecognizer(self.model, AUDIO_RATE)        
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



if __name__ == "__main__":
    rospy.init_node('qt_vosk_app')
        
    language = DEFAULT_LANG
    if rospy.has_param('~langauge'):
        language = rospy.get_param('~langauge')
            
    gspeech = QTrobotVoskSpeech('/qt_robot/speech', language)
    rospy.loginfo("qt_vosk_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_vosk_app shutdown")
