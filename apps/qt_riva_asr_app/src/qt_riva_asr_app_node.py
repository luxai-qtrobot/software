#!/usr/bin/env python3
import os
import queue
import time
import rospy

import riva.client

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from qt_riva_asr_app.srv import *


class MicrophoneStream:
    """Opens a recording stream as responses yielding the audio chunks."""

    def __init__(self, buffer) -> None:
        self.stream_buff = buffer
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self


    def __exit__(self, type, value, traceback):
        self.closed = True
        self.stream_buff.put(None)


    def __next__(self) -> bytes:
        if self.closed:
            raise StopIteration
        chunk = self.stream_buff.get()
        if chunk is None:
            raise StopIteration
        
        data = [chunk]

        while True:
            try:
                chunk = self.stream_buff.get(block=False)
                if chunk is None:
                    assert not self.closed
                data.append(chunk)
            except queue.Empty:
                break

        return b''.join(data)

    def __iter__(self):
        return self


class QTrobotRivaASR():
    """QTrobot speech recognition using riva asr"""

    def __init__(self):
        
        self.aqueue = queue.Queue(maxsize=2000) # more than one minute         
        self.audio_rate = rospy.get_param("/qt_riva_asr_app/audio_rate", 16000)
        self.language_code = rospy.get_param("/qt_riva_asr_app/language_code", 'en-US')
        self.file_streaming_chunk = rospy.get_param("/qt_riva_asr_app/file_streaming_chunk", 1600)
        self.server = rospy.get_param("/qt_riva_asr_app/server", 'localhost:50051')
        self.use_ssl = rospy.get_param("/qt_riva_asr_app/use_ssl", False)
        self.ssl_cert = rospy.get_param("/qt_riva_asr_app/ssl_cert", None)
        self.profanity_filter = rospy.get_param("/qt_riva_asr_app/profanity_filter", False)

        self.automatic_punctuation = rospy.get_param("/qt_riva_asr_app/automatic_punctuation", True)
        self.no_verbatim_transcripts = rospy.get_param("/qt_riva_asr_app/no_verbatim_transcripts", False)
        self.boosted_lm_words = rospy.get_param("/qt_riva_asr_app/boosted_lm_words", None)
        self.boosted_lm_score = rospy.get_param("/qt_riva_asr_app/boosted_lm_score", 4.0)
        self.speaker_diarization = rospy.get_param("/qt_riva_asr_app/speaker_diarization", True)

        print(f"audio rate:{self.audio_rate}, language code:{self.language_code}", self.use_ssl, self.ssl_cert, self.boosted_lm_words)

        self.auth = riva.client.Auth(self.ssl_cert, self.use_ssl, self.server)
        self.asr_service = riva.client.ASRService(self.auth)
        self.config = riva.client.StreamingRecognitionConfig(
            config=riva.client.RecognitionConfig(
                encoding=riva.client.AudioEncoding.LINEAR_PCM,
                language_code=self.language_code,
                max_alternatives=1,
                profanity_filter=self.profanity_filter,
                enable_automatic_punctuation=self.automatic_punctuation,
                verbatim_transcripts=not self.no_verbatim_transcripts,
                sample_rate_hertz=self.audio_rate,
                audio_channel_count=1,
            ),
            interim_results=True,
        )
        riva.client.add_word_boosting_to_config(self.config, self.boosted_lm_words, self.boosted_lm_score)
        riva.client.add_speaker_diarization_to_config(self.config, self.speaker_diarization)

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
        timeout = req.timeout if req.timeout != 0 else 15
        language = (req.language if (req.language != '') else self.language_code)
        language = language.replace("_", "-")
        options = list(filter(None, req.options)) # remove the empty options 
        
        transcript = self.recognize_riva(timeout, options, language, True)
        print(transcript)
        return speech_recognizeResponse(transcript)


    def recognize_riva(self, timeout, options, language, clear_queue=False):        
        # clear queue (keep the last second)
        if clear_queue:                        
            self.aqueue.queue.clear()
            # example : if audio rate is 16000 and respeaker buffersize is 512,then the last one second will be around 31 item in queue
            # while self.aqueue.qsize() > int(self.audio_rate / 512 / 2):
            #     self.aqueue.get()


        with MicrophoneStream(self.aqueue) as audio_chunk_iterator:
            start_time = time.time()
            responses = self.asr_service.streaming_response_generator( audio_chunks=audio_chunk_iterator, streaming_config=self.config)
            transcript = None
            for response in responses:                
                if response.results:
                    for result in response.results:
                        if not result.alternatives:
                            continue
                        transcript = result.alternatives[0].transcript
                        if result.is_final and not options:
                            return transcript.strip()
                        else:
                            # print(result)
                            opt =  self.contains_options(options, transcript.strip())
                            if opt:
                                return opt

                # check timeout 
                if timeout > 0 and not transcript:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > timeout:
                        return ''


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



if __name__ == "__main__":
    rospy.init_node('qt_riva_asr_app')            
    asr = QTrobotRivaASR()
    rospy.loginfo("qt_riva_asr_app is ready!")
    rospy.spin()
    rospy.loginfo("qt_riva_asr_app shutdown")
    
