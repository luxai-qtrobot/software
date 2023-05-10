#!/usr/bin/env python3
import os
import queue
import time
import rospy

from google.cloud import speech
# from google.cloud.speech import enums
# from google.cloud.speech import types
from google.api_core import exceptions as gexcp

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from qt_gspeech_app.srv import *

class MicrophoneStream(object):

    def __init__(self, buffer):
        self.stream_buff = buffer
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self.closed = True
        self.stream_buff.put(None)

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self.stream_buff.get()
            if chunk is None:
                return
            data = [chunk]
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self.stream_buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)



class QTrobotGoogleSpeech():
    """QTrobot speech recognition using google cloud service"""

    def __init__(self):
        
        self.aqueue = queue.Queue(maxsize=2000) # more than one minute         
        self.audio_rate = rospy.get_param("/qt_gspeech_app/audio_rate", 16000)
        self.language = rospy.get_param("/qt_gspeech_app/default_language", 'en-US')
        self.model = rospy.get_param("/qt_gspeech_app/model", 'default')
        self.use_enhanced_model = rospy.get_param("/qt_gspeech_app/use_enhanced_model", False)
        
        print(f"audio rate:{self.audio_rate}, default language:{self.language}, model:{self.model}, use_enhanced_model:{self.use_enhanced_model}")

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
        # timeout = (req.timeout if (req.timeout != 0) else 15)
        timeout = req.timeout
        language = (req.language if (req.language != '') else self.language)
        language = language.replace("_", "-")
        options = list(filter(None, req.options)) # remove the empty options 
        
        transcript = self.recognize_gspeech(timeout, options, language, True)
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


    def recognize_gspeech(self, timeout, options, language, clear_queue=False):        
        
        # clear queue (keep the last second)
        if clear_queue:                        
            # self.aqueue.queue.clear()
            # example : if audio rate is 16000 and respeaker buffersize is 512,then the last one second will be around 31 item in queue
            while self.aqueue.qsize() > int(self.audio_rate / 512 / 2):
                self.aqueue.get()


        # init google speech client
        self.client = speech.SpeechClient()

        answer_context = []
        speech_context = None
        if len(options) > 0:
            for option in options:
                if option.strip():
                    answer_context.append(option.lower().strip())
            speech_context = speech.SpeechContext(phrases = answer_context) if len(answer_context) else None
            config = speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=self.audio_rate,
                language_code=str(language.strip()),
                model=self.model,
                use_enhanced=self.use_enhanced_model,
                speech_contexts=[speech_context],
            )
        else:
            config = speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=self.audio_rate,
                model=self.model,
                use_enhanced=self.use_enhanced_model,
                language_code= str(language.strip()),
                enable_automatic_punctuation=True,
            )
        streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True,
            enable_voice_activity_events=True, 
            )
        with MicrophoneStream(self.aqueue) as mic:
            start_time = time.time()
            audio_generator = mic.generator()
            requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )
            try:
                if timeout > 0 :
                    responses = self.client.streaming_recognize(streaming_config, requests, timeout=timeout)
                else:
                    responses = self.client.streaming_recognize(streaming_config, requests)
                output = self.validate_response(responses, answer_context, start_time, timeout)
            except:
                output = ""
                print("exception")     

        print("Detected [%s]" % (output))
        return output


    """
        looping over google responses
    """
    def validate_response(self, responses, context, start_time, timeout):
        transcript = ""
        for response in responses:
            print(response)
            if not response.results:
                continue
            result = response.results[0]
            if not result.alternatives:
                continue
            transcript = result.alternatives[0].transcript

            if not result.is_final:
                if context:
                    for option in context:
                        if option == transcript.lower().strip():
                            return transcript
            else:
                 return transcript
        return transcript


if __name__ == "__main__":
    rospy.init_node('qt_gspeech_app')            
    gspeech = QTrobotGoogleSpeech()
    rospy.loginfo("qt_gspeech_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_gspeech_app shutdown")
    
