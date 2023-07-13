#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Kinematics:
    def __init__(self):
        # 订阅机器人速度
        self.velocity_subscriber = rospy.Subscriber("/course_agv/velocity", Twist,
                                                    self.velocity_callback)

        # 初始化
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # 创建两个轮子速度的发布者
        self.left_wheel_publisher = rospy.Publisher("/course_agv/left_wheel_velocity_controller/command", Float64,
                                                    queue_size=10)
        self.right_wheel_publisher = rospy.Publisher("/course_agv/right_wheel_velocity_controller/command", Float64,
                                                     queue_size=10)

    def velocity_callback(self, msg):
        # 从订阅的消息中获取线速度和角速度
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # 这步可能有问题
        r = 0.08
        p = 0.1+r/6

        # 计算左右轮子的速度
        # 这步可能有问题

        # self.left_wheel_velocity = (self.linear_velocity - self.angular_velocity * self.wheel_track) / self.wheel_track
        # self.right_wheel_velocity = (self.linear_velocity + self.angular_velocity * self.wheel_track) / self.wheel_track
        self.left_wheel_velocity = (self.linear_velocity - self.angular_velocity * p) / r
        self.right_wheel_velocity = (self.linear_velocity + self.angular_velocity * p) / r
        # print("i can receive the dwa")

    def publish(self):
        # 发布左右轮子的速度
        self.left_wheel_publisher.publish(self.left_wheel_velocity)
        self.right_wheel_publisher.publish(self.right_wheel_velocity)


if __name__ == '__main__':
    rospy.init_node('course_agv_kinematics')
    kinematics = Kinematics()
    rate = rospy.Rate(rospy.get_param('~publish_rate', 200))
    while not rospy.is_shutdown():
        kinematics.publish()
        rate.sleep()
