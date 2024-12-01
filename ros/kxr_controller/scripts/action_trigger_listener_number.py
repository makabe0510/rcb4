#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt16, String
from ros_speak import play_sound
from pathlib import Path  # pathlibをインポート

# 動作を実行する関数
def perform_action(neck_motion_message, eyebrow_status_message, sound_path):
    # 首と眉の動きのパブリッシュ
    neck_motion_pub.publish(neck_motion_message)
    eyebrow_status_pub.publish(eyebrow_status_message)
    rospy.loginfo("Published '{}' to /neck_motion".format(neck_motion_message))
    rospy.loginfo("Published '{}' to /eyebrow_status".format(eyebrow_status_message))

    # サウンド再生
    try:
        play_sound(sound_path, topic_name='robotsound_jp', wait=True)
        rospy.loginfo("Playing sound: {}".format(sound_path))
    except Exception as e:
        rospy.logerr("Failed to play sound: {}".format(e))

# 条件に基づいて動作を選択する関数
def action_callback(msg):
    global story, section

    trigger = msg.data  # UInt16の値を取得
    rospy.loginfo("Received trigger: {}".format(trigger))

    # 条件に基づく動作
    if story == 1 and section == 1 and trigger == 1:
        perform_action("disagree", "angry", 'package://kxr_controller/resources/ask_balloon.wav')
    elif story == 2 and section == 3 and trigger == 4:
        perform_action("tilt", "sad", 'package://kxr_controller/resources/ask_balloon.wav')
    else:
        rospy.loginfo("No action defined for story={}, section={}, trigger={}".format(story, section, trigger))

def main():
    global story, section
    rospy.init_node('action_listener')

    # 初期値を設定
    story = rospy.get_param('/story', 1)
    section = rospy.get_param('/section', 1)

    # パブリッシャーの設定
    global neck_motion_pub, eyebrow_status_pub
    neck_motion_pub = rospy.Publisher('/neck_motion', String, queue_size=1)
    eyebrow_status_pub = rospy.Publisher('/eyebrow_status', String, queue_size=1)

    # サブスクライバーの設定
    rospy.Subscriber('/action_trigger', UInt16, action_callback)

    rospy.loginfo("Action listener node started, waiting for messages...")

    # 外部からパラメータを変更可能にするループ
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        story = rospy.get_param('/story', story)
        section = rospy.get_param('/section', section)
        rospy.loginfo("Current story={}, section={}".format(story, section))
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
