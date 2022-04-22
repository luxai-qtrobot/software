#!/usr/bin/env python3
import os
import queue
import time
import rospy
import json
import vosk
import pvporcupine
from threading import Thread, Condition

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from qt_vosk_app.srv import *



class QTrobotVoskSpeech(Thread):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self):
        super(QTrobotVoskSpeech, self).__init__()

        self.is_kaldi_recognizing = False
        self.aqueue = queue.Queue(maxsize=2000) # more than one minute 
        self.condition = Condition()
        self.audio_rate = rospy.get_param("/qt_vosk_app/audio_rate", 16000)
        self.language = rospy.get_param("/qt_vosk_app/vosk/default_language", 'en_US')
        self.model_path = rospy.get_param("/qt_vosk_app/vosk/vosk_model_path")
        
        self.enable_hotword = rospy.get_param("/qt_vosk_app/hotword/enable_hotword", False)
        self.hotword_model = rospy.get_param("/qt_vosk_app/hotword/hotword_model")
        self.hotword = rospy.get_param("/qt_vosk_app/hotword/hotword")
        self.access_key = rospy.get_param("/qt_vosk_app/hotword/access_key")
        self.sensitivity = rospy.get_param("/qt_vosk_app/hotword/sensitivity", 0.7)
        self.publish_hotword = rospy.get_param("/qt_vosk_app/hotword/publish_hotword", False)
        self.feedback_audio = rospy.get_param("/qt_vosk_app/hotword/feedback_audio", None)

        # initialize vosk 
        self.model = vosk.Model(self.model_path + self.language)

        # initialize porcupine
        # Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher 
        # sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5
        # will be used.
        if self.enable_hotword:
            self.ppn = pvporcupine.create(keyword_paths=[self.hotword_model], access_key=self.access_key, sensitivities=[self.sensitivity]) 
            self.recognize_pub = rospy.Publisher('/qt_robot/speech/recognize', String, queue_size=10)
            if self.publish_hotword:
                self.hotword_pub = rospy.Publisher('/qt_robot/speech/hotword', String, queue_size=1)
                

        # start recognize service
        self.speech_recognize = rospy.Service('/qt_robot/speech/recognize', speech_recognize, self.callback_recognize)        
        rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.callback_audio_stream)

        # start the background thread 
        if self.enable_hotword:
            self.start()

     
    def stop(self):        
        with self.condition:
            self.condition.notifyAll()


    def run(self):
        """
        background thread which wait for wakeword and recognize 
        whatever being said after wakeword
        """
        while not rospy.is_shutdown():
            with self.condition:
                self.condition.wait()
            if rospy.is_shutdown():
                break
            print('Detected...')
            if self.publish_hotword:
                self.hotword_pub.publish(self.hotword)
            if self.feedback_audio:
                os.system(f"play {self.feedback_audio} >/dev/null 2>&1")
            transcript = self.recognize_kaldi(10, [], clear_queue=True)
            print(transcript)
            if transcript:
                self.recognize_pub.publish(transcript)



    def callback_audio_stream(self, msg):                
        indata = bytes(msg.data)           
        try:
            self.aqueue.put_nowait(indata)            
        except:
            pass

        # check for hotword if kaldi is not busy recognizing 
        if self.enable_hotword and not self.is_kaldi_recognizing:
            audio_frame = struct.unpack_from("h" * self.ppn.frame_length, indata)                    
            result = self.ppn.process(audio_frame)
            if result >= 0:                
                # notify the background thread to start recognizig 
                with self.condition:
                    self.condition.notifyAll()
        

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

        # check if we need to change the language model
        # print('current language: ' + self.language)
        if language != self.language:
            print('switching language to ' + language)
            # VOSK python API does not implement exception!
            # so we need to check the path by ourselves 
            
            if os.path.exists(self.model_path + language):
                self.model = vosk.Model(self.model_path + language)
                self.language = language
            else:
                rospy.loginfo('could not load language model for ' + language)
                return speech_recognizeResponse('')
        
        # with sd.RawInputStream(samplerate=self.device_samplerate, blocksize = 8000, device=self.device_index, dtype='int16', channels=1, callback=callback):
        transcript = self.recognize_kaldi(timeout, options, True)
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


    def recognize_kaldi(self, timeout, options, clear_queue=False):        
        self.is_kaldi_recognizing = True
        # clear queue (keep the last second)
        if clear_queue:                        
            # self.aqueue.queue.clear()
            # example : if audio rate is 16000 and respeaker buffersize is 512, then the last one second will be around 31 item in queue
            while self.aqueue.qsize() > int(self.audio_rate / 512 / 2):
                self.aqueue.get()

        if options:            
            rec = vosk.KaldiRecognizer(self.model, self.audio_rate, json.dumps(options, ensure_ascii=False))
        else:
            rec = vosk.KaldiRecognizer(self.model, self.audio_rate) 

        t_start = time.time()
        # should_stop = False
        transcript = ''
        while True:
            data = self.aqueue.get()
            if rec.AcceptWaveform(data):
                result = rec.Result()                 
                jres = json.loads(result)                
                transcript = jres['text'].strip()
                # print('result:', transcript)
                if transcript:
                    break

                # if not options:
                #     if transcript: 
                #         break
                # else:
                #     word = self.contains_options(options, transcript)
                #     if word:
                #         transcript = word
                #         break                            
            # else:
            #     result = rec.PartialResult()                     
            #     jres = json.loads(result)
            #     print('partial result:', jres['partial'])
            #     word = self.contains_options(options, jres['partial'])
            #     if word:
            #         transcript = word
            #         break
            # check the timeout
            if (time.time() - t_start) > timeout:
                transcript = ''
                break

        self.is_kaldi_recognizing = False        
        return transcript


if __name__ == "__main__":
    rospy.init_node('qt_vosk_app')
                    
    speech = QTrobotVoskSpeech()
    rospy.loginfo("qt_vosk_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_vosk_app shutdown")
    speech.stop()    
