#!/usr/bin/env python

from __future__ import print_function

import argparse

import IPython
from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
import rospy
from skrobot.model import RobotModel
