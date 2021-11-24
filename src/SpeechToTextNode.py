#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
from os import path
from threading import Thread

import actionlib
import numpy as np
import pvporcupine
import rospy
import soundfile
import speech_recognition as sr
from audioplayer import AudioPlayer
from home_robot_msgs.msg import IntentManagerAction, IntentManagerGoal
from mr_voice.msg import Voice
from rospkg import RosPack
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class SpeechToTextNode(object):
    # TODO: Try not to modify the hotword_detected variable at flowing and processing part
    def __init__(self):
        self.topic_audio_path = rospy.get_param("topic_audio_path", "/respeaker/audio_path")
        self.topic_text = rospy.get_param("topic_text", "~text")
        self.lang = rospy.get_param("lang", "en-us")

        rospy.Subscriber(self.topic_audio_path, String, self.callback_audio_path)
        self.pub_voice = rospy.Publisher(self.topic_text, Voice)
        self.facial = rospy.Publisher('/home_edu/facial', String, queue_size=1)

        rospy.set_param("~hotword_detected", False)

        rospy.Service('~start_flow', Trigger, self.start_flow_cb)
        rospy.Service('~stop_flow', Trigger, self.stop_flow_cb)

        self.intent_manager_proxy = actionlib.SimpleActionClient('/snips_intent_manager', IntentManagerAction)

        self.sr = sr.Recognizer()
        self.threads = []
        self.facial.publish('happy-1:Listening')

        base = RosPack().get_path('mr_voice')
        self.keywords_list = ['hey snippy', 'assistant', 'hey robie']
        # self.__hey_snippy_model = path.join(base,
        #                                     'keyword_models/hey-snippy__en_linux_2021-10-30-utc_v1_9_0.ppn')
        # self.__assistant_model = path.join(base,
        #                                    'keyword_models/assistant__en_linux_2021-10-30-utc_v1_9_0.ppn')
        self.__hey_robie_model = path.join(base,
                                           'keyword_models/hey-robie_en_linux_v1_9_0.ppn')

        self.porcupine = pvporcupine.create(
            library_path=pvporcupine.LIBRARY_PATH,
            model_path=pvporcupine.MODEL_PATH,
            keyword_paths=[
                self.__hey_robie_model
            ]
        )

        self.hotword_detected = False
        self.flowing = False
        self.is_running = False

        self.snd_wakeup = AudioPlayer(
            path.join(base, 'snd/snd_wakeup.mp3')
        )
        self.snd_recognized = AudioPlayer(
            path.join(base, 'snd/snd_recognized.mp3')
        )

    def callback_audio_path(self, msg: String):
        if not self.is_running:
            t = Thread(target=self._recognize_thread, args=(msg.data,))
            t.start()

    def start_flow_cb(self, req):
        self.flowing = True
        return TriggerResponse(success=True, message="Flow started")

    def stop_flow_cb(self, req):
        self.flowing = False
        return TriggerResponse(success=True, message="Flow stopped")

    def __detect_hotword(self, audio_path):
        wav_data = soundfile.read(audio_path, dtype='int16')[0]
        samples_count = len(wav_data) // self.porcupine.frame_length
        for i in np.arange(samples_count):
            frame = wav_data[i * self.porcupine.frame_length:(i + 1) * self.porcupine.frame_length]
            result = self.porcupine.process(frame)
            if result >= 0:
                return result
        return False

    def _recognize_thread(self, audio_path):
        text = ""
        direction = 0
        try:
            direction = int(audio_path.split(".")[0].split("-")[1])
        except Exception as e:
            rospy.logerr(e)

        with sr.AudioFile(audio_path) as source:
            if not self.hotword_detected:
                self.hotword_detected = self.__detect_hotword(audio_path)
                if self.hotword_detected:
                    self.snd_wakeup.play()
                    rospy.loginfo(f'Detected hotword {self.keywords_list[self.hotword_detected]}')
                    return

            rospy.set_param("~hotword_detected", bool(self.hotword_detected))
            audio = self.sr.record(source)

        if self.hotword_detected:
            self.is_running = True
            self.snd_recognized.play()
            try:
                self.facial.publish('smiling:Recognizing')
                rospy.loginfo(f"recognizing file: {audio_path}")
                text = self.sr.recognize_google(audio, language=self.lang)
            except sr.UnknownValueError:
                self.facial.publish('crying:Voice Recognition could not understand audio')
                rospy.logerr("Voice Recognition could not understand audio")
            except sr.RequestError as e:
                self.facial.publish('suspicious:Could not request results from Voice recognition service')
                rospy.logerr("Could not request results from Voice Recognition service; {0}".format(e))
            rospy.loginfo(f"{audio_path}: {text}")
            if len(text) > 0:
                voice = Voice()
                voice.time = rospy.Time.now()
                voice.text = text
                voice.direction = direction
                self.pub_voice.publish(voice)
            else:
                rospy.logerr("nothing")

            self.facial.publish('happy-1:Listening')

            goal = IntentManagerGoal()
            goal.text = text
            self.intent_manager_proxy.send_goal(goal)
            self.intent_manager_proxy.wait_for_result()
            if self.flowing:
                self.snd_wakeup.play()
            else:
                self.hotword_detected = False

        self.is_running = False


if __name__ == "__main__":
    rospy.init_node("voice")
    SpeechToTextNode()
    rospy.loginfo(rospy.get_name() + " OK.")
    rospy.spin()
