#!/usr/bin/env python3
from os import path

import numpy as np
import pvporcupine
import rospy
import speech_recognition as sr
from rospkg import RosPack
from std_msgs.msg import String, Int16
from std_srvs.srv import Trigger, TriggerResponse

from core.Nodes import Node


class HotWardManager(Node):
    def __init__(self):
        super(HotWardManager, self).__init__('hotword_manager')

        self.sr = sr.Recognizer()

        base = RosPack().get_path('mr_voice')
        self.keywords_list = ['assistant', 'hey anchor']
        self.__assistant_model = path.join(base,
                                           'models/keyword_models/assistant_en_linux_v2_1_0.ppn')
        self.__hey_anchor_model = path.join(base,
                                            'models/keyword_models/hey-anchor_en_linux_v2_1_0.ppn')

        self.porcupine = pvporcupine.create(
            library_path=pvporcupine.LIBRARY_PATH,
            model_path=pvporcupine.MODEL_PATH,
            keyword_paths=[
                self.__assistant_model,
                self.__hey_anchor_model
            ],
            access_key='RsiUIPj5Om0f8W2cwl3BIRBanqRFkDfqVHHnyNJsFgx3sh5dlWWhmg=='
        )

        self.pub = rospy.Publisher(
            '~hotword_idx',
            Int16,
            queue_size=1
        )

        rospy.Subscriber(
            '/respeaker/audio_path',
            String,
            self.callback_audio_path,
            queue_size=1
        )

        rospy.Service('~lock', Trigger, self.lock_handler)
        rospy.set_param('~lock', False)

        self.main()

    @staticmethod
    def lock_handler(req):
        rospy.set_param('~lock', not rospy.get_param('~lock'))
        return TriggerResponse()

    def callback_audio_path(self, msg: String):
        if rospy.get_param('~lock'):
            return

        with sr.AudioFile(msg.data) as source:
            audio = self.sr.record(source)
            hotword_index = self.__detect_hotword(audio)
            self.pub.publish(hotword_index)

    def __detect_hotword(self, audio):
        wav_data = np.frombuffer(audio.get_wav_data(), dtype='int16')
        samples_count = len(wav_data) // self.porcupine.frame_length

        for i in np.arange(samples_count):
            frame = wav_data[i * self.porcupine.frame_length:(i + 1) * self.porcupine.frame_length]
            result = self.porcupine.process(frame)
            if result >= 0:
                return result
        return -1

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def reset(self):
        pass


if __name__ == '__main__':
    HotWardManager()
