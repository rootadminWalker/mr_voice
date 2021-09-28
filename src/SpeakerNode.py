#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import pyttsx3
import rospy
from std_msgs.msg import String
from mr_voice.srv import SpeakerSrv, SpeakerSrvResponse


class Speaker(object):
    def __init__(self, rate: int = 170, volume: float = 1.0, lang: str = "en-US"):
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", rate)
        self.engine.setProperty("volume", volume)
        self.engine.setProperty("voice", lang)

        self.engine.connect('started-utterance', self.on_start)
        self.engine.connect('started-word', self.on_word)
        self.engine.connect('finished-utterance', self.on_end)

        self.is_running = False
        self.buffer = []

    def on_start(self, name):
        self.is_running = True

    def on_word(self, name, location, length):
        pass

    def on_end(self, name, completed):
        self.is_running = False

    def say(self, message: str) -> bool:
        self.buffer.append(message)
        if not self.is_running:
            while len(self.buffer) > 0:
                self.engine.say(self.buffer.pop(0))
                self.engine.runAndWait()
                self.engine.stop()
            return False
        return True


class SpeakerNode(object):
    def __init__(self):
        self.param_is_saying = "~is_saying"
        self.topic_say = "~say"

        self.rate = rospy.get_param("rate", 170)
        self.volume = rospy.get_param("volume", 1.0)
        self.lang = rospy.get_param("lang", "en-US")

        self.speaker = Speaker(self.rate, self.volume, self.lang)

        rospy.set_param(self.param_is_saying, False)
        self.facial = rospy.Publisher('/home_edu/facial', String, queue_size=1)
        rospy.Subscriber(self.topic_say, String, self.callback_say)
        rospy.Service('~text', SpeakerSrv, self.srv_callback)

    def callback_say(self, msg: String):
        rospy.set_param(self.param_is_saying, True)
        self.facial.publish(f'happy-2:{msg.data}')
        is_saying = self.speaker.say(msg.data)
        self.facial.publish(f'happy:')
        rospy.set_param(self.param_is_saying, is_saying)

    def srv_callback(self, req):
        rospy.set_param(self.param_is_saying, True)
        is_saying = self.speaker.say(req.text)
        rospy.set_param(self.param_is_saying, is_saying)
        return SpeakerSrvResponse(True)


if __name__ == "__main__":
    rospy.init_node("speaker")
    SpeakerNode()
    rospy.loginfo(rospy.get_name() + " OK.")
    rospy.spin()
