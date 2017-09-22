#!/usr/bin/env python
#

from copy             import deepcopy

import rospy

from sensor_msgs.msg  import Imu
from nav_msgs.msg    import Odometry

from vectornav.msg    import sensors
from vectornav.msg      import ins
from vectornav.msg      import gps

from tf.transformations import quaternion_from_euler

import math


def sub_gpsCB(msg_in):
  global pub_gps 
  global msg_gps

  msg_gps.header.stamp = msg_in.header.stamp         # time of gps measurement
  msg_gps.header.frame_id = 'base_footprint'         # the tracked robot frame
  msg_gps.pose.pose.position.x = msg_in.LLA.x        # x measurement GPS.
  msg_gps.pose.pose.position.y = msg_in.LLA.y        # y measurement GPS.
  msg_gps.pose.pose.position.z = msg_in.LLA.z        # z measurement GPS.
  msg_gps.pose.pose.orientation.x = -1               # identity quaternion: 0
  msg_gps.pose.pose.orientation.y = -1               # identity quaternion: 0
  msg_gps.pose.pose.orientation.z = -1               # identity quaternion: 0
  msg_gps.pose.pose.orientation.w = -1               # identity quaternion: 1
  msg_gps.pose.covariance = {1, 0, 0, 0, 0, 0,       # covariance on gps_x
                             0, 1, 0, 0, 0, 0,       # covariance on gps_y
                             0, 0, 1, 0, 0, 0,       # covariance on gps_z
                             0, 0, 0, 99999, 0, 0,   # large covariance on rot x
                             0, 0, 0, 0, 99999, 0,   # large covariance on rot y
                             0, 0, 0, 0, 0, 99999}   # large covariance on rot z
  pub_gps.publish(msg_gps)


def sub_odomCB(msg_in):
  global pub_odom
  global msg_odom

  msg_odom = msg_in
  msg_odom.pose.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
			      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
			      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
			      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
			      0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  msg_odom.header.frame_id = "odom"
  pub_odom.publish(msg_odom)


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

  msg_imu.orientation_covariance = [1.0, 0.0, 0.0, 
				    0.0, 1.0, 0.0, 
				    0.0, 0.0, 1.0]

  pub_imu.publish(msg_imu)


if __name__ == '__main__':
  rospy.init_node('vectornav_sensor_msgs')

  global pub_odom
  global pub_imu
  global pub_gps

  global msg_imu
  global msg_odom
  global msg_gps

  msg_odom = Odometry()
  msg_imu = Imu()
  msg_gps = Odometry()

  pub_odom  = rospy.Publisher("/vo", Odometry, queue_size=10)
  pub_imu  = rospy.Publisher("/imu_data", Imu, queue_size=10)
  # pub_gps  = rospy.Publisher("/odom", Odometry, queue_size=10)

  rospy.Subscriber("/zed/odom", Odometry,  sub_odomCB)
  # rospy.Subscriber("/vectornav/gps", gps,  sub_gpsCB)
  rospy.Subscriber("/vectornav/ins", ins,  sub_insCB)

  rospy.spin()
