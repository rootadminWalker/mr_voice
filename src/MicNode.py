#!/usr/bin/python3
# -*- encoding: utf-8 -*-
import os
import struct
import time
import wave

import numpy as np
import pyaudio
import rospy
from std_msgs.msg import String


class MicAudio(object):
    def __init__(self, on_audio, channels=None, suppress_error=True):
        self.on_audio = on_audio
        self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = 16000
        self.bitwidth = 2
        self.bitdepth = 16

        # find device
        info = self.pyaudio.get_default_input_device_info()
        self.available_channels = info["maxInputChannels"]
        self.device_index = info["index"]
        self.channels = range(self.available_channels)

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=1024,
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

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
        # split channel
        data = np.fromstring(in_data, dtype=np.int16)
        chunk_per_channel = len(data) // self.available_channels
        data = np.reshape(data, (chunk_per_channel, self.available_channels))
        for chan in self.channels:
            chan_data = data[:, chan]
            # invoke callback
            self.on_audio(chan_data.tostring(), chan)
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()


def on_audio(data, channel):
    global pub, audio_dir, mic_audio, max_buf, is_voice_buf, audio_buf
    if channel == 0:
        count = len(data) / 2
        df = "%dh" % count
        digits = struct.unpack(df, data)
        value = np.sum(np.abs(digits)) / count
        if (len(is_voice_buf) > 20): is_voice_buf.pop(0)
        curr = np.sum(np.array(is_voice_buf, dtype=np.float32) * 0.5) + value
        is_voice_buf.append(value)
        curr = curr / len(is_voice_buf)

        if curr < 400:
            if len(audio_buf) > 10:
                filename = time.strftime("%H%M%S", time.gmtime())
                filename = "%s-%d.wav" % (filename, 0)
                path = os.path.join(audio_dir, filename)

                rospy.loginfo("Save to %s" % (path))

                wf = wave.open(path, "wb")
                wf.setnchannels(1)
                wf.setsampwidth(mic_audio.pyaudio.get_sample_size(pyaudio.paInt16))
                wf.setframerate(mic_audio.rate)
                wf.writeframes(b"".join(audio_buf))
                wf.close()

                audio_buf = []
                pub.publish(path)
        else:
            audio_buf.append(data)


if __name__ == "__main__":
    rospy.init_node("respeaker")

    max_buf = rospy.get_param("buffer_size", 10)
    audio_dir = rospy.get_param("audio_directory", "/tmp/speech")

    pub = rospy.Publisher("~audio_path", String, queue_size=1)

    if not os.path.exists(audio_dir):
        os.mkdir(audio_dir)

    is_voice_buf = []
    audio_buf = []

    # respeaker_interface = RespeakerInterface()
    mic_audio = MicAudio(on_audio)
    mic_audio.start()

    rospy.spin()
