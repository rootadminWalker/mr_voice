#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import speech_recognition as sr
from threading import Thread
import rospy
from std_msgs.msg import String
from mr_voice.msg import Voice
from std_srvs.srv import Trigger, TriggerResponse


class SpeechToTextNode(object):
    def __init__(self):
        self.topic_audio_path = rospy.get_param("topic_audio_path", "/respeaker/audio_path")
        self.topic_text = rospy.get_param("topic_text", "~text")
        self.lang = rospy.get_param("lang", "en")

        rospy.Subscriber(self.topic_audio_path, String, self.callback_audio_path)
        self.pub_voice = rospy.Publisher(self.topic_text, Voice)

        self.sr = sr.Recognizer()
        self.threads = []

        rospy.Service('~lock', Trigger, self.lock_handler)
        rospy.set_param('~lock', False)

    def lock_handler(self, req):
        rospy.set_param('~lock', not rospy.get_param('~lock'))
        return TriggerResponse()

    def callback_audio_path(self, msg):
        t = Thread(target=self._recognize_thread, args=(msg.data,))
        t.start()

    def _recognize_thread(self, path):
        if rospy.get_param('~lock'):
            return

        text = ""
        direction = 0
        try:
            direction = int(path.split(".")[0].split("-")[1])
        except Exception as e:
            rospy.logerr(e.message)

        with sr.AudioFile(path) as source:
            audio = self.sr.record(source)
        try:
            rospy.loginfo("recognizing file: %s" % path)
            text = self.sr.recognize_google(audio, language=self.lang)
        except sr.UnknownValueError:
            rospy.logerr("Voice Recognition could not understand audio")
        except sr.RequestError as e:
            rospy.logerr("Could not request results from Voice Recognition service; %s" % str(e))
        rospy.loginfo("%s: %s" % (path, text))
        if len(text) > 0:
            voice = Voice()
            voice.time = rospy.Time.now()
            voice.text = text
            voice.direction = direction
            self.pub_voice.publish(voice)
        else:
            rospy.logerr("nothing")


if __name__ == "__main__":
    rospy.init_node("voice")
    SpeechToTextNode()
    rospy.loginfo(rospy.get_name() + " OK.")
    rospy.spin()
