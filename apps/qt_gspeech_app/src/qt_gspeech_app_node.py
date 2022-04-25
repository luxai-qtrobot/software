#!/usr/bin/env python3
import os
import queue
import time
import rospy

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from qt_gspeech_app.srv import *


class QTrobotGoogleSpeech():
    """QTrobot speech recognition using google cloud service"""

    def __init__(self):
        
        self.aqueue = queue.Queue(maxsize=2000) # more than one minute         
        self.audio_rate = rospy.get_param("/qt_gspeech_app/audio_rate", 16000)
        self.language = rospy.get_param("/qt_gspeech_app/default_language", 'en_US')


        # start recognize service
        self.speech_recognize = rospy.Service('/qt_robot/speech/recognize', speech_recognize, self.callback_recognize)        
        rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.callback_audio_stream)

     


    def callback_audio_stream(self, msg):                
        indata = bytes(msg.data)           
        try:
            self.aqueue.put_nowait(indata)            
        except:
            pass


    """
        ros speech recognize callback
    """
    def callback_recognize(self, req):
        print("options:", len(req.options), req.options)
        print("language:", req.language)
        print("timeout:", str(req.timeout))        
        timeout = (req.timeout if (req.timeout != 0) else 15)
        language = (req.language if (req.language != '') else self.language)
        options = list(filter(None, req.options)) # remove the empty options 
        
        transcript = self.recognize_gspeech(timeout, options, True)
        return speech_recognizeResponse(transcript)



    def contains_options(self, options, transcript):
        if not transcript:
            return None        
        for opt in options:
            opt = opt.strip()
            # do not split the transcript of an option contains more than a word such as 'blue color'
            phrase = transcript if (len(opt.split()) > 1) else transcript.split()
            if opt and opt in phrase:
                return opt
        return None


    def recognize_gspeech(self, timeout, options, clear_queue=False):        
        
        # clear queue (keep the last second)
        if clear_queue:                        
            # self.aqueue.queue.clear()
            # example : if audio rate is 16000 and respeaker buffersize is 512, then the last one second will be around 31 item in queue
            while self.aqueue.qsize() > int(self.audio_rate / 512 / 2):
                self.aqueue.get()

        return ""


if __name__ == "__main__":
    rospy.init_node('qt_gspeech_app')
                    
    speech = QTrobotGoogleSpeech()
    rospy.loginfo("qt_gspeech_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_gspeech_app shutdown")
    
