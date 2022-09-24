#!/usr/bin/env python3
"""
MIT License

Copyright (c) 2020 rootadminWalker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""
from os import path

import actionlib
import rospy
from audioplayer import AudioPlayer
from home_robot_msgs.msg import IntentManagerAction, IntentManagerGoal
from mr_voice.msg import Voice
from rospkg import RosPack
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from core.Nodes import Node


class FlowManager(Node):
    def __init__(self):
        super(FlowManager, self).__init__("flow_manager", anonymous=False)

        base = RosPack().get_path('mr_voice')
        self.snd_wakeup = AudioPlayer(
            path.join(base, 'snd/snd_wakeup.mp3')
        )
        self.snd_recognized = AudioPlayer(
            path.join(base, 'snd/snd_recognized.mp3')
        )

        rospy.set_param('/flow_manager/notify_sound_playing', False)

        self.intent_manager_proxy = actionlib.SimpleActionClient(
            'intent_manager',
            IntentManagerAction,
        )

        self.on_session = False

        self.speech_lock = rospy.ServiceProxy('/voice/lock', Trigger)
        self.hotword_lock = rospy.ServiceProxy('/hotword_manager/lock', Trigger)
        self.reset()

        self.main()

    def play_wakeup(self):
        rospy.set_param('/flow_manager/notify_sound_playing', True)
        self.snd_wakeup.play()
        rospy.set_param('/flow_manager/notify_sound_playing', False)

    def lock_hotword(self):
        if not rospy.get_param('/hotword_manager/lock'):
            self.hotword_lock()

    def release_hotword(self):
        if rospy.get_param('/hotword_manager/lock'):
            self.hotword_lock()

    def lock_speech(self):
        if not rospy.get_param('/voice/lock'):
            self.speech_lock()

    def release_speech(self):
        if rospy.get_param('/voice/lock'):
            self.speech_lock()

    def play_recognized(self):
        self.snd_recognized.play()

    def wait_for_hotword(self):
        if self.on_session:
            return 1
        return rospy.wait_for_message('/hotword_manager/hotword_idx', Int16).data

    def main(self):
        while not rospy.is_shutdown():
            hotword_idx = self.wait_for_hotword()
            if hotword_idx >= 0:
                self.play_wakeup()
                rospy.loginfo('Hotword detected, recognizing text')
                self.lock_hotword()
                self.release_speech()

                recognized_text = rospy.wait_for_message('/voice/text', Voice).text
                self.play_recognized()
                self.lock_speech()
                goal = IntentManagerGoal()
                goal.text = recognized_text
                self.intent_manager_proxy.send_goal(goal)
                self.intent_manager_proxy.wait_for_result()
                self.on_session = self.intent_manager_proxy.get_result().on_session
                rospy.loginfo(self.on_session)
                if not self.on_session:
                    self.release_hotword()

            self.rate.sleep()

    def reset(self):
        self.lock_speech()
        self.release_hotword()


if __name__ == '__main__':
    node = FlowManager()
