#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

# def callback(data):
#     try:
#         robot_base_idx = data.name.index("robot_base")  # 获取机器人基座在links中的索引
#     except ValueError:
#         # 如果在links中找不到robot_base，则直接返回
#         return
#
#     # 获取机器人基座的姿态信息
#     robot_pose = data.pose[robot_base_idx]
#     br = tf.TransformBroadcaster()
#
#     # 发送TF消息
#     br.sendTransform((robot_pose.position.x, robot_pose.position.y, robot_pose.position.z),
#                      (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
#                       robot_pose.orientation.w),
#                      rospy.Time.now(),
#                      "robot_base",
#                      "world")
#     self.tf_pub.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
#                           rospy.Time.now(), self.link_name, "map")


class GazeboLinkPose:

    def __init__(self, robot_name, link_name):
        self.robot_name = robot_name
        self.link_name = link_name
        self.states_sub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.callback)
        self.pose_pub = rospy.Publisher(
            "/gazebo/" + (self.robot_name + '__' + self.link_name), Pose, queue_size=10)
        self.link_pose = Pose()

    def callback(self, data):
        # print(data.name)
        robot_base_idx = data.name.index('course_agv::robot_base')
        self.link_pose = data.pose[robot_base_idx]

    def publish_tf(self):
        # 发送TF消息
        br = tf.TransformBroadcaster()
        br.sendTransform((self.link_pose.position.x, self.link_pose.position.y, self.link_pose.position.z),
                         (self.link_pose.orientation.x, self.link_pose.orientation.y, self.link_pose.orientation.z,
                          self.link_pose.orientation.w),
                         rospy.Time.now(), self.link_name, "map")


if __name__ == '__main__':
    rospy.init_node('robot_pose_tf_publisher')
    gp = GazeboLinkPose(rospy.get_param('~robot_name', 'course_agv'),
                        rospy.get_param('~link_name', 'robot_base'))
    publish_rate = rospy.get_param('~publish_rate', 100)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        gp.pose_pub.publish(gp.link_pose)
        gp.publish_tf()
        rate.sleep()
