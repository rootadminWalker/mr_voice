#!/usr/bin/env python3
from os import path

import actionlib
import numpy as np
import pvporcupine
import rospy
import speech_recognition as sr
from audioplayer import AudioPlayer
from core.Nodes import Node
from home_robot_msgs.msg import IntentManagerAction, IntentManagerGoal
from home_robot_msgs.srv import RecognizeFile
from rospkg import RosPack
from std_msgs.msg import String
from std_srvs.srv import Trigger


class HotWardManager(Node):
    def __init__(self):
        super(HotWardManager, self).__init__('hotword_manager')

        self.sr = sr.Recognizer()

        self.is_running = False

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

        self.snd_wakeup = AudioPlayer(
            path.join(base, 'snd/snd_wakeup.mp3')
        )
        self.snd_recognized = AudioPlayer(
            path.join(base, 'snd/snd_recognized.mp3')
        )

        self.recognize_file = rospy.ServiceProxy('/voice/recognize_file', RecognizeFile)

        self.intent_manager_proxy = actionlib.SimpleActionClient('/intent_manager', IntentManagerAction)

        self.topic_audio_path = rospy.get_param("topic_audio_path", "/respeaker/audio_path")
        rospy.Subscriber(self.topic_audio_path, String, self.callback_audio_path)

        rospy.set_param("~hotword_detected", False)
        rospy.set_param('~notify_sound_playing', False)

        self.main()

    @staticmethod
    def play_notify_sound(snd):
        rospy.set_param('~notify_sound_playing', True)
        snd.play()
        rospy.set_param('~notify_sound_playing', False)

    @staticmethod
    def on_session():
        return rospy.get_param('/intent_manager/on_session')

    @staticmethod
    def hotward_detected():
        return rospy.get_param('~hotword_detected')

    def callback_audio_path(self, msg: String):
        if self.is_running:
            return

        self.is_running = True
        audio_path = msg.data
        if self.hotward_detected() or self.on_session():
            self.play_notify_sound(self.snd_recognized)
            text = self.recognize_file(audio_path).text

            goal = IntentManagerGoal()
            goal.text = text
            self.intent_manager_proxy.send_goal(goal)
            self.intent_manager_proxy.wait_for_result()

            rospy.set_param('~hotword_detected', False)
            if self.on_session():
                self.play_notify_sound(self.snd_wakeup)

        else:
            with sr.AudioFile(audio_path) as source:
                audio = self.sr.record(source)
                hotword_index = self.__detect_hotword(audio)
                if hotword_index is not None:
                    self.play_notify_sound(self.snd_wakeup)
                    rospy.loginfo(f'Detected hotword {self.keywords_list[hotword_index]}')
                    rospy.set_param("~hotword_detected", True)

        self.is_running = False

    def __detect_hotword(self, audio):
        wav_data = np.frombuffer(audio.get_wav_data(), dtype='int16')
        samples_count = len(wav_data) // self.porcupine.frame_length

        for i in np.arange(samples_count):
            frame = wav_data[i * self.porcupine.frame_length:(i + 1) * self.porcupine.frame_length]
            result = self.porcupine.process(frame)
            rospy.loginfo(result)
            if result >= 0:
                return result

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def reset(self):
        pass


if __name__ == '__main__':
    HotWardManager()
