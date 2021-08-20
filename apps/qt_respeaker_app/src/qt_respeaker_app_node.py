#!/usr/bin/env python3

import os
import rospy
import struct
import sys
import time
import usb.core
import usb.util
import pyaudio
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool, Int32, ColorRGBA
from contextlib import contextmanager
from tuning import Tuning

try:
    from pixel_ring import usb_pixel_ring_v2
except IOError as e:
    print(e)
    raise RuntimeError("Check the device is connected and recognized")

# suppress error messages from ALSA
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield

class RespeakerInterface(object):

    def __init__(self):
        self.dev = self.find()
        rospy.loginfo("Initializing Respeaker device")
        #self.dev.reset()
        self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
        self.device_tuning = Tuning(self.dev)
        #set tuning parameters
        params = rospy.get_param_names()
        for p in params:
            if "qt_respeaker_app/tuning" in p:
                parameter = p.replace('/qt_respeaker_app/tuning/',"")
                self.device_tuning.write(str(parameter), rospy.get_param(p))
                rospy.loginfo("Set parameter %s : %s", parameter, str(rospy.get_param(p)))
        rospy.loginfo("Respeaker device initialized (Version: %s)" % self.device_tuning.version)
        

    def __del__(self):
        try:
            self.tuning().close()
        except:
            pass
        finally:
            self.dev = None
    
    def tuning(self):
        return self.device_tuning
    
    #def set_led_think(self):
    #    self.pixel_ring.set_brightness(10)
    #    self.pixel_ring.think()
    #
    #def set_led_trace(self):
    #    self.pixel_ring.set_brightness(10)
    #    self.pixel_ring.trace()
    #
    #def set_led_listen(self):
    #    self.pixel_ring.set_brightness(10)
    #    self.pixel_ring.listen()
    #
    #def set_led_spin(self):
    #    self.pixel_ring.set_brightness(10)
    #    self.pixel_ring.spin()
    #def set_led_color(self, r, g, b, a):
    #    self.pixel_ring.set_brightness(int(20 * a))
    #    self.pixel_ring.set_color(r=int(r*255), g=int(g*255), b=int(b*255))

    def find(self, vid=0x2886, pid=0x0018):
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if not dev:
            raise RuntimeError("Failed to find Respeaker device")
        return dev

class RespeakerAudio(object):
    def __init__(self, on_audio, channels=None, suppress_error=True):
        self.on_audio = on_audio
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = 6
        self.channels = channels
        self.device_index = None
        self.rate = 16000
        self.bitwidth = 2
        self.bitdepth = 16

        # # find respeaker mic
        self.device_index  = self.get_respeaker_device()        
        if self.channels is None:
            self.channels = range(self.available_channels)
        else:
            self.channels = filter(lambda c: 0 <= c < self.available_channels, self.channels)
        
        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=1024,
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def get_respeaker_device(self):
        info = self.pyaudio.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        for i in range(0, numdevices):
            if (self.pyaudio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                device = self.pyaudio.get_device_info_by_host_api_device_index(0, i)
                if "ReSpeaker" in device.get('name'):
                    return device.get('index')

    def __del__(self):
        self.stop()
        try:
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

    def stream_callback(self, in_data, frame_count, time_info, status):
        for chan in self.channels:
            chan_data = np.frombuffer(in_data, dtype=np.int16)[chan::self.available_channels]
            self.on_audio(chan_data.tobytes(), chan)
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()


class RespeakerNode(object):
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)

        self.update_rate = rospy.get_param("/qt_respeaker_app/update_rate", 10.0)
        suppress_pyaudio_error = rospy.get_param("/qt_respeaker_app/suppress_pyaudio_error", True)
        
        self.respeaker = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)
        self.is_speaking = False
        # advertise
        self.pub_vad = rospy.Publisher("qt_respeaker_app/is_speaking", Bool, queue_size=1, latch=True)
        self.pub_doa = rospy.Publisher("qt_respeaker_app/sound_direction", Int32, queue_size=1, latch=True)
        self.pub_audios = {c: rospy.Publisher('qt_respeaker_app/channel%d' % c, AudioData, queue_size=10) for c in self.respeaker_audio.channels}
                
        # start
        self.respeaker_audio.start()
        self.info_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate),self.on_timer)
        self.timer_led = None
        #self.sub_led = rospy.Subscriber("qt_respeaker_app/status_led", ColorRGBA, self.on_status_led)

    def on_shutdown(self):
        try:
            self.respeaker.tuning().close()
        except:
            pass
        finally:
            self.respeaker = None
        try:
            self.respeaker_audio.stop()
        except:
            pass
        finally:
            self.respeaker_audio = None
    

    #def on_status_led(self, msg):
    #    self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
    #    if self.timer_led and self.timer_led.is_alive():
    #        self.timer_led.shutdown()
    #    self.timer_led = rospy.Timer(rospy.Duration(3.0),
    #                                   lambda e: self.respeaker.set_led_trace(),
    #                                   oneshot=True)

    def on_audio(self, data, channel):
        self.pub_audios[channel].publish(AudioData(data=data))

    def on_timer(self, event):
        is_voice = self.respeaker.tuning().is_voice()
        if is_voice:
            self.pub_doa.publish(self.respeaker.tuning().direction)
            self.pub_vad.publish(Bool(data=is_voice))


if __name__ == '__main__':
    rospy.init_node("qt_respeaker_app_node")
    n = RespeakerNode()
    rospy.spin()
