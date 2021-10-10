#!/usr/bin/env python3

import glob
import os
from os import path

import rospy
import watchdog.events
import watchdog.observers


class TmpSpeechFilesWatcher(watchdog.events.FileSystemEventHandler):
    def __init__(self, speeches_path, limit_amount):
        self.tmp_speeches = self.__find_exist_speeches(speeches_path)
        self.limit_amount = limit_amount

    def on_created(self, event: watchdog.events.FileCreatedEvent):
        if not event.is_directory:
            src_path = event.src_path
            if src_path.endswith('.wav'):
                rospy.loginfo(f'Tmp speech file {src_path} created')
                self.tmp_speeches.append(src_path)
                if len(self.tmp_speeches) > self.limit_amount:
                    remove_speech = self.tmp_speeches.pop(0)
                    rospy.logwarn(f'Tmp speeches amount already > then {self.limit_amount}, removing {remove_speech}')
                    os.remove(remove_speech)

    @staticmethod
    def __find_exist_speeches(speeches_path):
        exists_speeches = glob.glob(path.join(speeches_path, '*.wav'))
        return exists_speeches


if __name__ == '__main__':
    rospy.init_node('tmp_files_watcher')
    rate = rospy.Rate(35)

    speeches_path = rospy.get_param('~speeches_path')
    limit_amount = rospy.get_param('~limit_amount')

    if not path.exists(speeches_path):
        os.mkdir(speeches_path)

    event_handler = TmpSpeechFilesWatcher(speeches_path=speeches_path, limit_amount=limit_amount)

    observer = watchdog.observers.Observer()
    observer.schedule(event_handler=event_handler, path=speeches_path, recursive=True)
    observer.start()

    while not rospy.is_shutdown():
        rate.sleep()

    observer.stop()
    observer.join()
