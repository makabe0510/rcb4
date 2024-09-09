#!/usr/bin/env python

from __future__ import print_function

import argparse
import numpy as np
import IPython
from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
import rospy
from skrobot.model import RobotModel

rospy.init_node('kxr_interface', anonymous=True)

namespace = ''
download_urdf_mesh_files(namespace)

robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    namespace + '/robot_description_viz')
ri = KXRROSRobotInterface(  # NOQA
    robot_model, namespace=namespace, controller_timeout=60.0)

#define nod, disagree, tilt and then define test
