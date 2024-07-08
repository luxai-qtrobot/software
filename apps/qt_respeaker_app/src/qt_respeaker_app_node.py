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

from qt_respeaker_app.srv import *

try:
    from pixel_ring import usb_pixel_ring_v2
except IOError as e:
    print(e)
    raise RuntimeError("Check the device is connected and recognized")

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




class Tuning:
    TIMEOUT = 100000
    # parameter list
    # name: (id, offset, type, max, min , r/w, info)
    PARAMETERS = {
        'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
        'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
        'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
        'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
        'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
        'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
        'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
        'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
        'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
        'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
        'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
        'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
        'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
        'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
        'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
        'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
        'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
        'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
        'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
        'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
        'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
        'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
        'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
        'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
        'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
        'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
        'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
        'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
        'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
        'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
        'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
        'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
        'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
        'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
        'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
        'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
        # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
        'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
    }

    def __init__(self, vid=0x2886, pid=0x0018):
        self.dev = usb.core.find(idVendor=vid, idProduct=pid)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")

    def write(self, name, value):
        try:
            data = self.PARAMETERS[name]
        except KeyError:
            return False

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
        return True

    def read(self, name):        
        try:
            data = self.PARAMETERS[name]
        except KeyError:
            return None

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        response = struct.unpack(b'ii', response.tobytes())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)




class RespeakerInterface(object):

    def __init__(self):        
        rospy.loginfo("Initializing Respeaker device")
        
        self.device_tuning = Tuning()
        self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.device_tuning.dev)
        
        #set tuning parameters
        params = rospy.get_param_names()
        for p in params:
            if "qt_respeaker_app/tuning" in p:
                parameter = p.replace('/qt_respeaker_app/tuning/',"")
                self.device_tuning.write(str(parameter), rospy.get_param(p))
                rospy.loginfo("Set parameter %s : %s", parameter, str(rospy.get_param(p)))
        rospy.loginfo("Respeaker device initialized (Version: %s)" % self.device_tuning.version)

        self.set_led_trace()


    def __del__(self):
        try:
            self.tuning().close()
        except:
            pass
    
    def tuning(self):
        return self.device_tuning
    
    def set_led_think(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.think()
    
    def set_led_trace(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.trace()
    
    def set_led_listen(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.listen()
    
    def set_led_spin(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.spin()

    def set_led_color(self, r, g, b, a):
        # print('set_led_color', r, g, b, a)
        self.pixel_ring.set_brightness(int(20 * a))
        self.pixel_ring.set_color(r=int(r*255), g=int(g*255), b=int(b*255))


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
        rospy.loginfo("Using microphone device %d", self.device_index)
        if self.channels is None:
            self.channels = range(self.available_channels)
        else:
            self.channels = filter(lambda c: 0 <= c < self.available_channels, self.channels)
        
        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=512,
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def get_respeaker_device(self):
        dev_id = rospy.get_param("/qt_respeaker_app/mic_device_id", None)
        if dev_id:
            return dev_id            
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

        self.update_rate = rospy.get_param("/qt_respeaker_app/update_rate", 100.0)
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
        self.sub_led = rospy.Subscriber("qt_respeaker_app/status_led", ColorRGBA, self.on_status_led)

        # start tuning services
        self.tuning_set = rospy.Service('/qt_respeaker_app/tuning/set', tuning_set, self.tuning_set)        
        self.tuning_get = rospy.Service('/qt_respeaker_app/tuning/get', tuning_get, self.tuning_get)


    """
        ros tuning set callback
    """
    def tuning_set(self, req):
        resp = tuning_setResponse()
        try:
            ret = self.respeaker.tuning().write(req.param, req.value)
            resp.status = ret            
        except Exception as e:
            rospy.logwarn(str(e))
            resp.status = False
        return resp            


    """
        ros tuning get callback
    """
    def tuning_get(self, req):
        resp = tuning_getResponse()
        try:
            resp.value = self.respeaker.tuning().read(req.param)            
            resp.status = False if resp.value is None else True
            resp.value = 0 if not resp.value else resp.value
        except Exception as e:
            rospy.logwarn(str(e))
            resp.value = 0
            resp.status = False
        return resp


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
    

    def on_status_led(self, msg):        
        if msg.r==0 and msg.g==0 and msg.b==0 and msg.a==0: 
            self.respeaker.set_led_trace()
        else:
            self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)

    #    if self.timer_led and self.timer_led.is_alive():
    #        self.timer_led.shutdown()
    #    self.timer_led = rospy.Timer(rospy.Duration(3.0),
    #                                   lambda e: self.respeaker.set_led_trace(),
    #                                   oneshot=True)


    def on_audio(self, data, channel):
        self.pub_audios[channel].publish(AudioData(data=data))

    def on_timer(self, event):
        try:            
            is_voice = self.respeaker.tuning().is_voice()
            if is_voice:
                self.pub_doa.publish(self.respeaker.tuning().direction)
                self.pub_vad.publish(Bool(data=is_voice))
        except: 
            rospy.logwarn("error reading respeaker!")


if __name__ == '__main__':
    rospy.init_node("qt_respeaker_app", anonymous=True)
    n = RespeakerNode()
    rospy.spin()
