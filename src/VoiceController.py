#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import rospy
from mr_voice.msg import Voice 


class VoiceController(object):
    def __init__(self):
        self.t_voice_text = rospy.get_param("~voice_text", "/voice/text")
        rospy.Subscriber(self.t_voice_text, Voice, self.callback_voice_text)
    
    def callback_voice_text(msg: Voice):
        s = msg.text


if __name__ == "__main__":
    vc = VoiceController()
