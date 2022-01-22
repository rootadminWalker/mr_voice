#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import subprocess

import rospy
from mr_voice.srv import SpeakerSrv, SpeakerSrvResponse
from std_msgs.msg import String


class Speaker(object):
    def __init__(self):
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
                text = self.buffer.pop(0)
                mp3_raw = subprocess.Popen(['gtts-cli', '--lang=en', f'{text}'], stdout=subprocess.PIPE).stdout
                subprocess.run(['play', '-t',  'mp3',  '-'], stdin=mp3_raw)
            return False
        return True


class GTTSSpeakerNode(object):
    def __init__(self):
        self.param_is_saying = "~is_saying"
        self.topic_say = "~say"

        self.speaker = Speaker()

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
    GTTSSpeakerNode()
    rospy.loginfo(rospy.get_name() + " OK.")
    rospy.spin()
