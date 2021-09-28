#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import speech_recognition as sr
from threading import Thread
import rospy
from std_msgs.msg import String
from mr_voice.msg import Voice


class SpeechToTextNode(object):
    def __init__(self):
        self.topic_audio_path = rospy.get_param("topic_audio_path", "/respeaker/audio_path")
        self.topic_text = rospy.get_param("topic_text", "~text")
        self.lang = rospy.get_param("lang", "en-us")

        rospy.Subscriber(self.topic_audio_path, String, self.callback_audio_path)
        self.pub_voice = rospy.Publisher(self.topic_text, Voice)
        self.facial = rospy.Publisher('/home_edu/facial', String, queue_size=1)

        self.sr = sr.Recognizer()
        self.threads = []
        self.facial.publish('happy-1:Listening')

    def callback_audio_path(self, msg: String):
        t = Thread(target=self._recognize_thread, args=(msg.data,))
        t.start()

    def _recognize_thread(self, path):
        text = ""
        direction = 0
        try:
            direction = int(path.split(".")[0].split("-")[1])
        except Exception as e:
            rospy.logerr(e.message)

        with sr.AudioFile(path) as source:
            audio = self.sr.record(source)
        try:
            self.facial.publish('smiling:Recognizing')
            rospy.loginfo(f"recognizing file: {path}")
            text = self.sr.recognize_google(audio, language=self.lang)
        except sr.UnknownValueError:
            self.facial.publish('crying:Voice Recognition could not understand audio')
            rospy.logerr("Voice Recognition could not understand audio")
        except sr.RequestError as e:
            self.facial.publish('suspicious:Could not request results from Voice recognition service')
            rospy.logerr("Could not request results from Voice Recognition service; {0}".format(e))
        rospy.loginfo(f"{path}: {text}")
        if len(text) > 0:
            voice = Voice()
            voice.time = rospy.Time.now()
            voice.text = text
            voice.direction = direction
            self.pub_voice.publish(voice)
        else:
            rospy.logerr("nothing")

        self.facial.publish('happy-1:Listening')


if __name__ == "__main__":
    rospy.init_node("voice")
    SpeechToTextNode()
    rospy.loginfo(rospy.get_name() + " OK.")
    rospy.spin()
