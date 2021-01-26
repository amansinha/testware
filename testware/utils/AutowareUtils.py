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
# AutowareUtils.py
# Purpose: utilities for interacting with Autoware stack
# Notes:
#   to make sure my editor saves in utf-8 here is a nice character: é
###############################################################################
import math
import numpy as np
import json
from rospy.rostime import Time
from rosgraph_msgs.msg import Clock
import tf
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import CameraInfo, Imu, PointField
from sensor_msgs.point_cloud2 import create_cloud
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped, TwistStamped, TransformStamped
from autoware_vehicle_msgs.msg import Steering, Shift, ShiftStamped, TurnSignal
import transforms
from scipy.spatial.transform import Rotation

CAMERA_PARAMS = {'width': 800,
                 'height': 400,
                 'fov': 74.4845}


def increment_clock(clock_pub, cumulative_time):
    clock_signal = Clock()
    clock_signal.clock = Time.from_sec(cumulative_time)
    increment_clock.time = clock_signal.clock
    clock_pub.publish(clock_signal)
    return cumulative_time


def publish_lidars(pubs, measurements, frame_ids):
    for p, m, f in zip(pubs, measurements, frame_ids):
        _publish_lidar(p, m, f)


def _publish_lidar(pub, carla_lidar_measurement, frame_id):
    header = Header()
    header.stamp = increment_clock.time
    header.frame_id = frame_id
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
    lidar_data = np.fromstring(carla_lidar_measurement.raw_data, dtype=np.float32)
    lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))
    lidar_data[:, 1] *= -1
    point_cloud_msg = create_cloud(header, fields, lidar_data)
    pub.publish(point_cloud_msg)


def publish_cameras(pub_infos, pub_images, carla_images, frame_ids):
    for pi, pm, cm, f in zip(pub_infos, pub_images, carla_images, frame_ids):
        _publish_camera(pi, pm, cm, f)


def _publish_camera(pub_info, pub_image, carla_image, frame_id):
    # assumes all cameras the same
    if not hasattr(_publish_camera, 'cv_bridge'):
        _publish_camera.cv_bridge = CvBridge()
    cv_bridge = _publish_camera.cv_bridge
    cam_info = _build_camera_info(frame_id)
    if (carla_image.height != cam_info.height) or (carla_image.width != cam_info.width):
        assert("Camera received image not matching configuration")
    image_data_array, encoding = _get_carla_image_data_array(carla_image)
    img_msg = cv_bridge.cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header.stamp = increment_clock.time
    img_msg.header.frame_id = frame_id + '/camera_optical_link'
    pub_info.publish(cam_info)
    pub_image.publish(img_msg)


def _build_camera_info(frame_id):
    camera_info = CameraInfo()
    camera_info.header.stamp = increment_clock.time
    camera_info.header.frame_id = frame_id + '/camera_optical_link'
    camera_info.width = CAMERA_PARAMS['width']
    camera_info.height = CAMERA_PARAMS['height']
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = camera_info.width / (2.0 * math.tan(np.deg2rad(CAMERA_PARAMS['fov'])))
    fy = fx
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.D = [0, 0, 0, 0, 0]
    camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
    return camera_info


def _get_carla_image_data_array(carla_image):
    carla_image_data_array = np.ndarray(
        shape=(carla_image.height, carla_image.width, 4),
        dtype=np.uint8, buffer=carla_image.raw_data)
    return carla_image_data_array, 'bgra8'


def publish_currentpose(pub, tfb, carla_transform, com_to_base, height):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = increment_clock.time
    pose.pose.position = transforms.carla_location_to_ros_point(carla_transform.location)
    pose.pose.position.z += height  # center of body height
    r = Rotation.from_euler('xyz', transforms.carla_rotation_to_RPY(carla_transform.rotation), degrees=False)
    base_shift = r.apply(np.array([com_to_base['x'],
                                   com_to_base['y'],
                                   com_to_base['z']]))
    pose.pose.position.x += base_shift[0]
    pose.pose.position.y += base_shift[1]
    pose.pose.position.z += base_shift[2]
    pose.pose.orientation = transforms.carla_rotation_to_ros_quaternion(carla_transform.rotation)
    pub.publish(pose)
    broadcast_tf(tfb, pose.pose)


def broadcast_tf(tfb, pose):
    odom_trans = TransformStamped()
    odom_trans.header.stamp = increment_clock.time
    odom_trans.header.frame_id = "map"
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = pose.position.x
    odom_trans.transform.translation.y = pose.position.y
    odom_trans.transform.translation.z = pose.position.z
    odom_trans.transform.rotation = pose.orientation
    tfb.sendTransformMessage(odom_trans)


def publish_steering(pub, steering_angle_rad):
    steer = Steering()
    steer.header.frame_id = "base_link"
    steer.header.stamp = increment_clock.time
    steer.data = steering_angle_rad
    pub.publish(steer)


def publish_speed(pub, speed):
    speedmsg = Float32()
    speedmsg.data = speed
    pub.publish(speedmsg)


def publish_twist(pub, carla_vel, carla_omega, carla_rot, com_to_base):
    twist = TwistStamped()
    twist.header.frame_id = "base_link"
    twist.header.stamp = increment_clock.time

    r = Rotation.from_euler('xyz', transforms.carla_rotation_to_RPY(carla_rot), degrees=False)
    body_vel = r.inv().apply(transforms.carla_velocity_to_numpy_vector(carla_vel))
    body_omega = r.inv().apply(np.deg2rad(np.array([carla_omega.x, -carla_omega.y, -carla_omega.z])))
    base_omega = body_omega
    base_vel = body_vel + np.cross(base_omega, np.array([com_to_base['x'],
                                                         com_to_base['y'],
                                                         com_to_base['z']]))
    twist.twist.linear.x = base_vel[0]
    twist.twist.linear.y = base_vel[1]
    twist.twist.linear.z = base_vel[2]
    twist.twist.angular.x = base_omega[0]
    twist.twist.angular.y = base_omega[1]
    twist.twist. angular.z = base_omega[2]
    pub.publish(twist)


def publish_shift(pub):
    shift_msg = ShiftStamped()
    shift_msg.header.frame_id = "base_link"
    shift_msg.header.stamp = increment_clock.time
    shift_msg.shift.data = Shift.DRIVE
    pub.publish(shift_msg)


def publish_turnsignal(pub):
    msg = TurnSignal()
    msg.header.frame_id = "base_link"
    msg.header.stamp = increment_clock.time
    msg.data = TurnSignal.NONE
    pub.publish(msg)


def initialize_pose(pub, carla_transform, com_to_base):
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = increment_clock.time
    init_pose.pose.pose = transforms.carla_transform_to_ros_pose(carla_transform)
    init_pose.pose.pose.position.z += 0.768  # center of body height
    r = Rotation.from_euler('xyz', transforms.carla_rotation_to_RPY(carla_transform.rotation), degrees=False)
    base_shift = r.apply(np.array([com_to_base['x'],
                                   com_to_base['y'],
                                   com_to_base['z']]))
    init_pose.pose.pose.position.x += base_shift[0]
    init_pose.pose.pose.position.y += base_shift[1]
    init_pose.pose.pose.position.z += base_shift[2]
    init_pose.pose.covariance = [0.] * 36
    pub.publish(init_pose)


def initialize_goal(pub, goal_state):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = increment_clock.time
    ros_point = Point()
    ros_point.x = goal_state[0]
    ros_point.y = goal_state[1]
    ros_point.z = goal_state[2]
    goal.pose.position = ros_point
    roll = goal_state[3]
    pitch = goal_state[4]
    yaw = goal_state[5]
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    ros_quaternion = transforms.numpy_quaternion_to_ros_quaternion(quat)
    goal.pose.orientation = ros_quaternion
    pub.publish(goal)


def publish_imu(pub, ego_vehicle, sensor_file):
    if not hasattr(publish_imu, 'imu_rot'):
        with open(sensor_file) as f:
            json_sensors = json.loads(f.read())
            for sensor_spec in json_sensors['sensors']:
                if 'imu' not in sensor_spec['type']:
                    continue
                temp = np.array([sensor_spec['roll'], -sensor_spec['pitch'], -sensor_spec['yaw']])
                publish_imu.imu_rot = Rotation.from_euler('xyz', temp, degrees=True)
    carla_accel = ego_vehicle.get_acceleration()
    carla_omega = ego_vehicle.get_angular_velocity()
    carla_rot = ego_vehicle.get_transform().rotation
    msg = Imu()
    msg.header.frame_id = "tamagawa/imu_link"
    msg.header.stamp = increment_clock.time
    r_body = Rotation.from_euler('xyz', transforms.carla_rotation_to_RPY(carla_rot), degrees=False)
    r = r_body * publish_imu.imu_rot
    imu_accel = r.inv().apply(transforms.carla_velocity_to_numpy_vector(carla_accel))  # func wants vel, but its the same for accel
    imu_omega = r.inv().apply(np.deg2rad(np.array([carla_omega.x, -carla_omega.y, -carla_omega.z])))
    msg.linear_acceleration.x = imu_accel[0]
    msg.linear_acceleration.y = imu_accel[1]
    msg.linear_acceleration.z = imu_accel[2] - 9.8  # they want gravity subtracted
    msg.angular_velocity.x = imu_omega[0]
    msg.angular_velocity.y = imu_omega[1]
    msg.angular_velocity.z = imu_omega[2]

    re = r.as_euler('xyz', degrees=False)
    quat = tf.transformations.quaternion_from_euler(re[0], re[1], re[2])
    ros_quaternion = transforms.numpy_quaternion_to_ros_quaternion(quat)
    msg.orientation = ros_quaternion
    pub.publish(msg)
