# -*- coding: utf-8 -*-
'''
Copyright (c) 2021, Trustworthy AI, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.  Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2.  Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3.  Neither the name of the copyright holder(s) nor the names of any contributors
may be used to endorse or promote products derived from this software without
specific prior written permission. No license is granted to the trademarks of
the copyright holders even if such marks are included in this software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
###############################################################################
# testware.py
# Purpose: contains encapsulating class and helper methods for running
#   Testware agents.
# Notes:
#   to make sure my editor saves in utf-8 here is a nice character: é
###############################################################################

from __future__ import print_function
import math
import numpy as np
import os
import subprocess
from scipy.spatial.transform import Rotation
import time

import rospy
from autoware_vehicle_msgs.msg import VehicleCommand, Steering, ShiftStamped, TurnSignal
from autoware_system_msgs.msg import AutowareState
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, Imu
from std_msgs.msg import Bool, Float32
import tf

from .utils.vehicle_model import VehicleModel
from .utils import CarlaUtils, AutowareUtils, transforms

PATH = os.path.abspath(__file__)
DIR_PATH = os.path.dirname(PATH)

AUTOWARE_CLOCK_SUBSTEP_SIZE = 100
SPEED_UP_FACTOR = 1


def autoware_state_callback(data, args):
    args['status'] = data.state


def autoware_current_pose_callback(data, args):
    start_pose = args['start_pose']  # np array
    if not args['pose_is_set']:
        if np.allclose(np.array([data.pose.position.x, data.pose.position.y]), start_pose[:2], atol=0.1):
            args['pose_is_set'] = True


def autoware_current_control_callback(data, args):
    has_control = args['control_is_set']
    if not has_control:
        args['control_cmd'] = data
        args['control_is_set'] = True


class Testware(object):
    SENSOR_FILE = DIR_PATH + '/sensor_kit.json'
    LOW_SPEED = 7.5  # m/s
    HIGH_SPEED = 12.5  # m/s

    def __init__(self, allcams):
        self.allcams = allcams
        FNULL = open(os.devnull, 'w')
        subprocess.Popen(["roscore"], shell=True, executable="/bin/bash", stdout=FNULL, stderr=subprocess.STDOUT)
        rospy.init_node('trustworthy_worker_node', anonymous=True)

        print('starting autoware')
        FNULL = open(os.devnull, 'w')
        executable = "/launch_autoware_allcams.sh" if allcams else "/launch_autoware.sh"
        self.p = subprocess.Popen([DIR_PATH + executable], shell=True, executable="/bin/bash", stdout=FNULL, stderr=subprocess.STDOUT)

        self.cumulative_time = 0.
        self.clock_pub = rospy.Publisher('clock', Clock, queue_size=1)
        self.pose_pub = rospy.Publisher('trustworthy_pose', PoseWithCovarianceStamped, queue_size=1)
        self.goal_pub = rospy.Publisher('/planning/mission_planning/trustworthy_goal', PoseStamped, queue_size=1)
        self.currentpose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.tfb = tf.TransformBroadcaster()
        self.shift_pub = rospy.Publisher('/vehicle/status/shift', ShiftStamped, queue_size=1)
        self.turnsignal_pub = rospy.Publisher('/vehicle/status/turn_signal', TurnSignal, queue_size=1)
        self.engage_pub = rospy.Publisher('/autoware/engage', Bool, queue_size=1)
        self.steering_pub = rospy.Publisher('/vehicle/status/steering', Steering, queue_size=1)
        self.speed_pub = rospy.Publisher('/vehicle/status/velocity', Float32, queue_size=1)
        self.twist_pub = rospy.Publisher('/vehicle/status/twist', TwistStamped, queue_size=1)
        self.caminfo_pubs = [rospy.Publisher('/camera_info' + str(i), CameraInfo, queue_size=1) for i in range(1, 7)]
        self.camimage_pubs = [rospy.Publisher('/image_raw' + str(i), Image, queue_size=1) for i in range(1, 7)]
        self.cam_frame_ids = ['camera' + str(i) for i in range(1, 7)]
        if not allcams:
            self.caminfo_pubs = self.caminfo_pubs[0:1]
            self.camimage_pubs = self.camimage_pubs[0:1]
            self.cam_frame_ids = ['camera5']  # only front camera
        self.lidar_pubs = [rospy.Publisher('/sensing/lidar/' + x + '/pointcloud_raw_ex', PointCloud2, queue_size=1)
                           for x in ['top', 'left', 'right']]
        self.lidar_frame_ids = ['velodyne_top', 'velodyne_left', 'velodyne_right']
        self.imu_pub = rospy.Publisher('/sensing/imu/tamagawa/imu_raw', Imu, queue_size=1)
        self.autoware_callback_dict = {'status': 'TrustworthyStart',
                                       'pose_is_set': False,
                                       'start_pose': None,
                                       'control_is_set': False,
                                       'control_cmd': None}
        self.system_sub = rospy.Subscriber('/autoware/state', AutowareState, autoware_state_callback, self.autoware_callback_dict)
        self.pose_sub = rospy.Subscriber('/current_pose', PoseStamped, autoware_current_pose_callback, self.autoware_callback_dict)
        self.cmd_sub = rospy.Subscriber('/control/vehicle_cmd', VehicleCommand, autoware_current_control_callback, self.autoware_callback_dict)

        self.VM = VehicleModel(mass=1517.1876220703125, rotationalInertia=2594.637451171875,
                               wheelbase=2.700000047683716, centerToFront=1.187999963760376,
                               tireStiffnessFront=118582.7265625, tireStiffnessRear=147286.15625,
                               steerRatio=1.0, steerRatioRear=0.0)
        self.vehicle_type = 'vehicle.toyota.prius'
        self.height = 0.7686808109283447
        self.com_to_base = {'x': -1.39, 'y': 0., 'z': 0.34, 'roll': 0., 'pitch': 0., 'yaw': 0.}
        self.v0 = 0.
        self.speed = 0.
        self.steering_angle = 0.
        # right handed convention x,y,z,roll,pitch,yaw
        self.goal_state = [-492.75, -105.1, 0.768 - 0.404, 0, 0, np.pi / 2.]
        print('done starting autoware')

    def setPose(self, waypoints, client, world):
        idx = np.random.choice(3, size=1, replace=False) + np.random.randint(len(waypoints) - 3)
        idx = idx[0]
        pose = waypoints[idx].transform
        actor_list = CarlaUtils.poses_batch([pose], [self.vehicle_type], [self.height], client, world)
        assert(len(actor_list) == 1)
        self.carla_actor = actor_list[0]

    def setSensors(self, draw, client, world):
        return CarlaUtils.set_sensors(self.carla_actor, self.allcams, draw, client, world, Testware.SENSOR_FILE)

    def start(self):
        self.v0 = np.random.uniform(Testware.LOW_SPEED, Testware.HIGH_SPEED)

    def stop(self, fps):
        self._autoware_stop(fps)

    def _send_vehicle_state(self):
        AutowareUtils.publish_steering(self.steering_pub, self.steering_angle)
        AutowareUtils.publish_speed(self.speed_pub, self.speed)
        AutowareUtils.publish_twist(self.twist_pub,
                                    self.carla_actor.get_velocity(),
                                    self.carla_actor.get_angular_velocity(),
                                    self.carla_actor.get_transform().rotation,
                                    self.com_to_base)
        AutowareUtils.publish_shift(self.shift_pub)
        AutowareUtils.publish_turnsignal(self.turnsignal_pub)

    def _autoware_initialize(self, carla_transform_init_pose, fps, camera_outs, lidar_outs):
        callback_dict = self.autoware_callback_dict
        while callback_dict['status'] != 'Driving':
            AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
            AutowareUtils.publish_lidars(self.lidar_pubs, lidar_outs, self.lidar_frame_ids)
            AutowareUtils.publish_cameras(self.caminfo_pubs, self.camimage_pubs, camera_outs, self.cam_frame_ids)
            AutowareUtils.publish_currentpose(self.currentpose_pub, self.tfb, carla_transform_init_pose, self.com_to_base, self.height)
            self._send_vehicle_state()
            if callback_dict['status'] == 'InitializingVehicle':  # happens on autoware startup
                AutowareUtils.initialize_pose(self.pose_pub, carla_transform_init_pose, self.com_to_base)
            if callback_dict['status'] == 'WaitingForRoute':
                AutowareUtils.initialize_goal(self.goal_pub, self.goal_state)
            if callback_dict['status'] == 'WaitingForEngage':
                self.engage_pub.publish(True)
            self.cumulative_time += 1. / fps
            time.sleep(0.01)

    def _autoware_stop(self, fps):
        clock_pub = self.clock_pub
        callback_dict = self.autoware_callback_dict
        while callback_dict['status'] != 'WaitingForEngage':
            AutowareUtils.increment_clock(clock_pub, self.cumulative_time)
            self.engage_pub.publish(False)
            self.cumulative_time += 1. / fps
            time.sleep(0.01)
        callback_dict['pose_is_set'] = False

    def _autoware_reset(self, carla_transform_init_pose, fps, lidar_outs, camera_outs):
        callback_dict = self.autoware_callback_dict
        while callback_dict['status'] != 'Driving':
            AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
            AutowareUtils.publish_lidars(self.lidar_pubs, lidar_outs, self.lidar_frame_ids)
            AutowareUtils.publish_cameras(self.caminfo_pubs, self.camimage_pubs, camera_outs, self.cam_frame_ids)
            AutowareUtils.publish_currentpose(self.currentpose_pub, self.tfb, carla_transform_init_pose, self.com_to_base, self.height)
            self._send_vehicle_state()
            if not callback_dict['pose_is_set']:
                AutowareUtils.initialize_pose(self.pose_pub, carla_transform_init_pose, self.com_to_base)
            if callback_dict['pose_is_set'] and callback_dict['status'] == 'WaitingForEngage':
                AutowareUtils.initialize_goal(self.goal_pub, self.goal_state)
                self.engage_pub.publish(True)
            self.cumulative_time += 1. / fps
            time.sleep(0.01)

    def _autoware_start(self, fps, lidar_outs, camera_outs):
        carla_transform_init_pose = self.carla_actor.get_transform()
        temp = np.array([carla_transform_init_pose.location.x,
                         -carla_transform_init_pose.location.y,
                         carla_transform_init_pose.location.z])
        r = Rotation.from_euler('xyz', transforms.carla_rotation_to_RPY(carla_transform_init_pose.rotation), degrees=False)
        base_shift = r.apply(np.array([self.com_to_base['x'],
                                       self.com_to_base['y'],
                                       self.com_to_base['z']]))
        self.autoware_callback_dict['start_pose'] = temp + base_shift
        if self.autoware_callback_dict['status'] in ['TrustworthyStart', 'InitializingVehicle']:
            self._autoware_initialize(carla_transform_init_pose, fps, camera_outs, lidar_outs)
        else:
            if self.autoware_callback_dict['status'] != 'WaitingForEngage':
                print('current status', self.autoware_callback_dict['status'])
                self._autoware_stop(fps)
            self._autoware_reset(carla_transform_init_pose, fps, lidar_outs, camera_outs)

    def _autoware_receive_control(self, fps):
        for counter in range(AUTOWARE_CLOCK_SUBSTEP_SIZE - 1):
            # this order is reveresed from startup since we have already incremented once
            self.cumulative_time += 1. / fps / AUTOWARE_CLOCK_SUBSTEP_SIZE
            AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
            time.sleep(1. / fps / AUTOWARE_CLOCK_SUBSTEP_SIZE / SPEED_UP_FACTOR)
            if self.autoware_callback_dict['control_is_set']:
                break
        # add the last substep and whatever change is left
        self.cumulative_time += (AUTOWARE_CLOCK_SUBSTEP_SIZE - counter - 1.) / fps / AUTOWARE_CLOCK_SUBSTEP_SIZE
        if not self.autoware_callback_dict['control_is_set']:
            print('Using old control')

    def step(self, cnt, fps, camera_outs, lidar_outs):
        if cnt == 0:
            self.speed = self.v0
            self.steering_angle = 0.
            for _ in range(5):
                AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
                self.cumulative_time += 1. / fps
            self._autoware_start(fps=fps, lidar_outs=lidar_outs, camera_outs=camera_outs)
            AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
            AutowareUtils.publish_currentpose(self.currentpose_pub, self.tfb, self.carla_actor.get_transform(), self.com_to_base, self.height)
            self._send_vehicle_state()
            AutowareUtils.publish_cameras(self.caminfo_pubs, self.camimage_pubs, camera_outs, self.cam_frame_ids)
            AutowareUtils.publish_lidars(self.lidar_pubs, lidar_outs, self.lidar_frame_ids)
            AutowareUtils.publish_imu(self.imu_pub, self.carla_actor, Testware.SENSOR_FILE)
            self.cumulative_time += 1. / fps
            return None
        AutowareUtils.increment_clock(self.clock_pub, self.cumulative_time)
        AutowareUtils.publish_currentpose(self.currentpose_pub, self.tfb, self.carla_actor.get_transform(), self.com_to_base, self.height)
        self._send_vehicle_state()
        AutowareUtils.publish_cameras(self.caminfo_pubs, self.camimage_pubs, camera_outs, self.cam_frame_ids)
        AutowareUtils.publish_lidars(self.lidar_pubs, lidar_outs, self.lidar_frame_ids)
        AutowareUtils.publish_imu(self.imu_pub, self.carla_actor, Testware.SENSOR_FILE)
        self._autoware_receive_control(fps)
        control_cmd = self.autoware_callback_dict['control_cmd']
        if control_cmd is not None:
            inputs = [max(control_cmd.control.acceleration * 1. / fps + self.speed, 0.0), control_cmd.control.steering_angle]
        else:
            inputs = [self.speed, self.steering_angle]
        self.speed = inputs[0]
        self.steering_angle = inputs[1]
        yawrate = self.VM.yaw_rate(self.steering_angle, self.speed)
        VM = self.VM
        speed = self.speed
        if self.speed > 1.:
            beta = (VM.aR+VM.aF*VM.chi - VM.m*speed**2/VM.cF/VM.cR/VM.l*(VM.cF*VM.aF-VM.cR*VM.aR*VM.chi))  # noqa: E226
            beta *= yawrate/(1-VM.chi)/speed  # noqa: E226
        else:
            beta = np.arctan(np.tan(self.steering_angle) / 2.)
        inputs.extend([math.degrees(yawrate), math.degrees(beta)])
        self.autoware_callback_dict['control_is_set'] = False
        return np.asarray(inputs)

    def kill(self):
        ros_pid = subprocess.check_output(["pgrep", "-P", str(self.p.pid)]).strip().decode()
        os.system("pgrep -P " + ros_pid + " | xargs kill")
        os.system("kill -9 " + str(self.p.pid))
        os.system("pkill -9 ros")
