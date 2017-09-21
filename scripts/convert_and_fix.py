#!/usr/bin/env python
#

from copy             import deepcopy

import rospy

from sensor_msgs.msg  import Imu
from nav_msgs.msg    import Odometry

from vectornav.msg    import sensors
from vectornav.msg      import ins

from tf.transformations import quaternion_from_euler

import math

def sub_imuCB(msg_in):
  global pub_imu

  global msg_imu

  msg_imu.header.stamp          = msg_in.header.stamp
  msg_imu.header.frame_id       = 'base_footprint'
  msg_imu.angular_velocity.x    = msg_in.Gyro.x
  msg_imu.angular_velocity.y    = msg_in.Gyro.y
  msg_imu.angular_velocity.z    = msg_in.Gyro.z
  msg_imu.linear_acceleration.x = msg_in.Accel.x
  msg_imu.linear_acceleration.y = msg_in.Accel.y
  msg_imu.linear_acceleration.z = msg_in.Accel.z
  msg_imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
  msg_imu.angular_velocity_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
  pub_imu.publish(msg_imu)


def sub_odomCB(msg_in):
  global pub_odom
  global msg_odom

  msg_odom = msg_in
  msg_odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  msg_odom.header.frame_id = "odom"

  pub_odom.publish(msg_in)


def sub_insCB(msg_in):
  global msg_imu  
  global pub_imu

  msg_imu.header.stamp          = msg_in.header.stamp
  msg_imu.header.frame_id       = 'base_footprint'

  q = quaternion_from_euler(msg_in.RPY.x/180.0 * math.pi, msg_in.RPY.y/180.0 * math.pi, msg_in.RPY.z/180.0 * math.pi)
  
  msg_imu.orientation.x = q[0] 
  msg_imu.orientation.y = q[1]
  msg_imu.orientation.z = q[2]
  msg_imu.orientation.w = q[3]

  msg_imu.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

  pub_imu.publish(msg_imu)


if __name__ == '__main__':
  rospy.init_node('vectornav_sensor_msgs')

  global pub_odom
  global pub_imu

  global msg_imu
  global msg_odom

  msg_odom = Odometry()
  msg_imu = Imu()

  pub_odom  = rospy.Publisher("/vo", Odometry, queue_size=10)
  pub_imu  = rospy.Publisher("/imu_data", Imu, queue_size=10)

  rospy.Subscriber("/zed/odom", Odometry,  sub_odomCB)
#  rospy.Subscriber("/vectornav/imu", sensors,  sub_imuCB)
  rospy.Subscriber("/vectornav/ins", ins,  sub_insCB)

  rospy.spin()
