#!/usr/bin/env python3

import sys
import struct
import usb.core
import usb.util

import PySimpleGUI as sg

# parameter list
# name: (id, offset, type, max, min , r/w, info)
PARAMETERS = {

    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON', 'Automatic Gain Control'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))', 'Automatic Gain Control'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))', 'Automatic Gain Control'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))', 'Automatic Gain Control'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.', 'Automatic Gain Control'),
        
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON', 'Stationary noise suppression (ASR)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))', 'Stationary noise suppression (ASR)'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)', 'Stationary noise suppression (ASR)'),

    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON', 'Stationary noise suppression'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))', 'Stationary noise suppression'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation', 'Stationary noise suppression'),

    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON', 'None-stationary noise suppression (ASR)'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))', 'None-stationary noise suppression (ASR)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)', 'None-stationary noise suppression (ASR)'),

    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON', 'None-stationary noise suppression'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))', 'None-stationary noise suppression'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation', 'None-stationary noise suppression'),
    
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only', 'Echo cancelation'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON', 'Echo cancelation'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation', 'Echo cancelation'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation', 'Echo cancelation'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation', 'Echo cancelation'),    
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON', 'Echo cancelation'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON', 'Echo cancelation'),    
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2', 'Echo cancelation'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients', 'Echo cancelation'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)', 'Echo cancelation'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))', 'Echo cancelation'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)', 'Echo cancelation'),


    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON', 'Others'),    
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off', 'Others'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds', 'Others'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON', 'Others'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only', 'Others'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)', 'Others'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)', 'Others'),            
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)', 'Others'),
    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)', 'Others'),    
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))', 'Others'),
    # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.', 'Others'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.', 'Others')
}


class Tuning:
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

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

        response = struct.unpack(b'ii', response.tostring())

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


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return

    # configuration = dev.get_active_configuration()

    # interface_number = None
    # for interface in configuration:
    #     interface_number = interface.bInterfaceNumber

    #     if dev.is_kernel_driver_active(interface_number):
    #         dev.detach_kernel_driver(interface_number)

    return Tuning(dev)


def set_param(dev, name, value):
    _value = value
    if type(_value) is bool:
        _value = 1 if True else 0
    dev.write(name, _value)



if __name__ == '__main__':

    dev = find()
    if not dev:
        print('No device found')
        sys.exit(1)

    
    sg.theme('DarkAmber')   # Add a touch of color
    # All the stuff inside your window.
    default_params = {}
    frames_layout = {}    
    for name in PARAMETERS:
        data = PARAMETERS[name]
        # ignore read only params
        if data[5] == 'ro':
            continue              
        value = dev.read(name)        
        default_params[name] = value
        label = '%s (%s-%s)' %(name, data[4], data[3])
        if data[2]=='int' and (data[3]+data[4])==1:            
            inp = sg.Checkbox('', default=(value==1), key=name, enable_events=True)
        else:
            resolution = 0.01 if data[2]=='float' else 1
            inp = sg.Slider((data[4], data[3]), default_value=value, 
                            orientation='horizontal', 
                            size=(20,10), 
                            resolution=resolution,
                            key=name, enable_events=True)
                    
        group = data[-1]
        if group not in frames_layout:
            frames_layout[group] = []
        frames_layout[group].append([sg.Text(label, tooltip=data[6], size=(30,1), text_color='white', key=name+'_label'), inp])
            
    count = 0
    layout_l = []
    layout_r = []
    for name in frames_layout:
        frame = sg.Frame(name, frames_layout[name])
        if count <= 3:
            layout_l.append([frame])
        else:
            layout_r.append([frame])
        count = count + 1
        
    layout = [[sg.Column(layout_l), sg.Column(layout_r)], [sg.Button('Reset Parameters', key='reset'), sg.Button('Close', key='close')]]
                  
    # Create the Window
    window = sg.Window('ReSpeaker Mic Array v2.0 Tunning (firmware version %s)' % dev.version, layout, finalize=True)
    
    # Event Loop to process "events" and get the "values" of the inputs
    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED or event == 'close': # if user closes window or clicks cancel
            break
        if event == 'reset': 
            for name in default_params:
                dev.write(name, default_params[name])
                window[name].update(default_params[name])
                window[name+'_label'].update(text_color='white')
        else:
            # set the single param received by event
            set_param(dev, event, values[event])
            window[event+'_label'].update(text_color= 'red' if default_params[event] != values[event] else 'white')
    
    window.close()
    dev.close()
