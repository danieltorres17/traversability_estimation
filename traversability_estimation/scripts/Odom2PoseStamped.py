#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def odom_callback(msg):
  pose = PoseWithCovarianceStamped()
  pose.header.frame_id = msg.header.frame_id
  pose.header.stamp = msg.header.stamp
  pose.pose = msg.pose
  pub.publish(pose)

if __name__ == '__main__':
  rospy.init_node("odom_to_pose_stamped")
  # name of topic to publish stamped pose messages to
  pose_topic = rospy.get_param("~pose_topic_name")
  # name of odometry messages topic
  odom_topic = rospy.get_param("~odom_topic")
  pub = rospy.Publisher(pose_topic, PoseWithCovarianceStamped, queue_size=100)
  rospy.Subscriber(odom_topic, Odometry, callback=odom_callback)
  rospy.spin()