#!/usr/bin/env python3

from __future__ import print_function

import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from kxr_controller.pooh_interface import PoohROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
from skrobot.model import RobotModel
import numpy as np

speak_flag = False

def callback(msg):
    global speak_flag
    speak_flag = msg.data

# ROSノードの初期化
rospy.init_node('pooh_motion_control', anonymous=True)

sub = rospy.Subscriber("/is_speaking", Bool, queue_size=1,
                       callback=callback)

# URDFのダウンロードとロボットモデルの初期化
namespace = ''
download_urdf_mesh_files(namespace)
robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    namespace + '/robot_description_viz')
rospy.loginfo("Init Real Robot Interface")
ri = PoohROSRobotInterface(robot_model, namespace=None, controller_timeout=60.0)
rospy.loginfo("Init Real Robot Interface Done")

# サーボをONにし、init_pose
ri.servo_on()
rospy.sleep(1.0)
ri.send_stretch(30)
rospy.sleep(1.0)
ri.angle_vector(robot_model.init_pose())
rospy.loginfo('init_pose')

# nod動作
def nod(send_time=1):
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    robot_model.head_neck_p.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_p.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

# disagree動作
def disagree(send_time=1):
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    robot_model.head_neck_y.joint_angle(np.deg2rad(-30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    #robot_model.head_neck_y.joint_angle(np.deg2rad(30))
    #ri.angle_vector(robot_model.angle_vector(), send_time)
    #ri.wait_interpolation()
    robot_model.head_neck_y.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

# tilt動作
def tilt(send_time=1):
    global speak_flag
    rospy.loginfo("tilt")
    controller_type = 'head_controller'
    #ri.angle_vector(robot_model.init_pose(),
    #                controller_type=controller_type)
    robot_model.head_neck_r.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")        
    robot_model.head_neck_r.joint_angle(np.deg2rad(0))
    ri.angle_vector(robot_model.angle_vector(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()    

def right_hand_up(send_time=1):
    controller_type='rarm_controller'
    ri.angle_vector([-0.11780956, -0.01119175, -0.00235602, -1.3312497 ,  0.9943143 ,-1.2829478 ,  0.31808645,  0.33988124, -0.29157892], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    rospy.sleep(2.0)
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type=controller_type)
    ri.wait_interpolation()

def left_hand_chin():
    global speak_flag
    controller_type='larm_controller'
    ri.angle_vector([-0.08423378,  0.02179497, -0.00235602, -0.24209882,  0.02238402,0.02474022,  0.31808645,  0.88062793, -1.9085175 ], 2, controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 3, controller_type=controller_type)
    ri.wait_interpolation()

def right_hand_mouth():
    global speak_flag
    controller_type='rarm_controller'                                          
    ri.angle_vector([ 0.00530161,  0.19497527,  0.00235637, -0.9136143 , -0.92009383, 0.99725956,  0.4023204 ,  0.09424796, -0.12841243], 2, controller_type=controller_type)
    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 3, controller_type=controller_type)                
    ri.wait_interpolation()

def banzai(send_time=1):
    ri.angle_vector([ 0.00530161, -0.00530126, -0.00235602, -2.0675607 ,  0.7257081 , -0.9236281 ,  1.9444498 ,  0.6738718 ,  0.36933368], send_time, controller_type='rarm_controller')
    ri.angle_vector([ 0.00530161, -0.00530126, -0.00235602, -2.0675607 ,  0.7257081 , -0.9236281,  1.9444498 ,  0.6738718 ,  0.36933368], send_time, controller_type='larm_controller')
    ri.wait_interpolation()
    rospy.sleep(2.0)
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type='rarm_controller')
    ri.angle_vector(robot_model.init_pose(), send_time,
                    controller_type='larm_controller')
    ri.wait_interpolation()

def onegai():
    global speak_flag
    ri.angle_vector([ 0.01708259, -0.18555015, -0.00235602, -0.24798931, -1.0903288 ,  1.1150693 ,  0.22207151,  0.88062793, -0.9112581 ], 2, controller_type='rarm_controller')
    ri.angle_vector([ 0.01708259, -0.18555015, -0.00235602, -0.24798931, -1.0903288 ,  1.1150693 ,  0.22207151,  0.88062793, -0.9112581 ], 2, controller_type='larm_controller')


    ri.wait_interpolation()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)
    rospy.loginfo("while")
    while not rospy.is_shutdown() and speak_flag is True:
        rospy.loginfo("Waiting is_speaking is False")
        rate.sleep()
    rospy.loginfo("while end")

    ri.angle_vector(robot_model.init_pose(), 2, controller_type='rarm_controller')
    ri.angle_vector(robot_model.init_pose(), 2, controller_type='larm_controller')
    ri.wait_interpolation()

def left_hand_point(send_time=1):
    controller_type = 'larm_controller'
    ri.angle_vector([ 1.7555017e-07,  1.7555017e-07,  2.3563702e-03, -1.3665912e-01,-5.5606174e-01,  1.9379719e-01,  6.6680324e-01,  1.1892895e+00,-3.0335987e-01], send_time, controller_type=controller_type)
    ri.wait_interpolation()
    ri.angle_vector(robot_model.init_pose(), send_time, controller_type='larm_controller')
    ri.wait_interpolation()

# def 




    
# コールバック関数
def neck_motion_callback(msg):
    command = msg.data.lower()
    if command == 'nod':
        rospy.loginfo('Executing nod motion')
        nod()
    elif command == 'disagree':
        rospy.loginfo('Executing disagree motion')
        disagree()
    elif command == 'tilt':
        rospy.loginfo('Executing tilt motion')
        tilt()
    elif command == 'test':  # testメッセージに対応
        rospy.loginfo('Executing test motions')
        test()
    elif command == '':
        rospy.loginfo('no neck motion')
    else:
        rospy.logwarn(f'Unknown command received: {command}')

def arm_motion_callback(msg):
    command = msg.data.lower()
    if command == 'right_hand_up':
        rospy.loginfo('Executing right_hand_up motion')
        right_hand_up()
    elif command == 'left_hand_chin':
        rospy.loginfo('Executing left_hand_chin motion')
        left_hand_chin()
    elif command == 'right_hand_mouth':
        rospy.loginfo('Executing right_hand_mouth motion')
        right_hand_mouth()
    elif command == 'banzai':  
        rospy.loginfo('Executing banzai motions')
        banzai()
    elif command == 'onegai':
        rospy.loginfo('Executing onegai motion')
        onegai()
    elif command == 'left_hand_point':
        rospy.loginfo('Executing left_hand_point motion')
        left_hand_point()
    elif command == 'right_hand_bye':
        rospy.loginfo('Executing right_hand_bye motion')
        right_hand_bye()
    elif command == '':
        rospy.loginfo('no arm motion')
    else:
        rospy.logwarn(f'Unknown command received: {command}')

        
# テスト用関数
def test():
    ri.angle_vector(robot_model.init_pose())
    ri.wait_interpolation()
    nod()
    ri.wait_interpolation()    
    disagree()
    ri.wait_interpolation()    
    tilt()

# メイン関数
def main():
    # /neck_motionトピックを購読し、neck_motion_callback関数をコールバックに指定
    rospy.Subscriber('/neck_motion', String, neck_motion_callback)
    rospy.Subscriber('/arm_motion', String, arm_motion_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



