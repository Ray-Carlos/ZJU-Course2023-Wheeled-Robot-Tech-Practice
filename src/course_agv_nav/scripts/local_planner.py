#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

import dwa

from threading import Lock, Thread
import time


def limitVal(minV, maxV, v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v


class LocalPlanner:
    def __init__(self):

        self.arrive = 0.1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.laser_lock = Lock()
        self.plan_config = dwa.Config()
        self.dwa = dwa.DWA(self.plan_config)
        c = self.plan_config
        self.threshold = c.max_speed * c.predict_time

        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path', Path, self.pathCallback)
        # print(self.path_sub)
        self.vel_pub = rospy.Publisher('/course_agv/velocity', Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal', PoseStamped, queue_size=1)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan', LaserScan, self.laserCallback)
        self.planner_thread = None

        self.need_exit = False
        pass

    def updateGlobalPose(self):
        # print("update plan")
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans, self.rot) = self.tf.lookupTransform('/map', '/robot_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

        self.goal_index=len(self.path.poses) - self.count
        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x - self.x, p.y - self.y)
        if dis < 0.6 and self.count<len(self.path.poses):
            self.count+=1
        # ind = self.goal_index
        # self.goal_index = len(self.path.poses) - 1
        # while ind < len(self.path.poses)-1:
        #     p = self.path.poses[ind].pose.position
        #     dis = math.hypot(p.x - self.x, p.y - self.y)
        #     # print('mdgb;; ',len(self.path.poses),ind,dis)
        #     if dis < self.threshold:
        #         self.goal_index = ind
        #         if dis < 0.7:
        #             self.goal_index = ind + 1
        #         break
        #     ind += 1
        # print("self.goal_index",self.goal_index)
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        lgoal = self.tf.transformPose("/robot_base", goal)
        self.plan_goal = np.array([lgoal.pose.position.x, lgoal.pose.position.y])
        self.goal_dis = math.hypot(self.x - goal.pose.position.x,
                                   self.y - goal.pose.position.y)
        # self.goal_dis = math.hypot(self.x - self.path.poses[-1].pose.position.x,
        #                            self.y - self.path.poses[-1].pose.position.y)
        # print("x,y:",self.x,self.y)
        # print("path:",self.path.poses)
        print("goal_dis:", self.goal_dis)
        # print("plan_goal:", self.plan_goal)

    def laserCallback(self, msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100, 100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        # print("amount of obstacles is",len(msg.ranges))
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment * i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a) * r, math.sin(a) * r])
        self.laser_lock.release()
        pass

    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob)
        self.laser_lock.release()
        pass

    def pathCallback(self, msg):
        self.need_exit = True
        time.sleep(0.01)
        self.path = msg
        self.planner_thread = Thread(target=self.planThreadFunc)
        self.initPlanning()
        self.planner_thread.start()

    def initPlanning(self):
        print("init plan")
        self.count=1
        self.goal_index = 0
        self.vx = 0.0
        self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.goal = np.array([cx[0], cy[0]])
        self.plan_cx, self.plan_cy = np.array(cx), np.array(cy)
        self.plan_goal = np.array([cx[-1], cy[-1]])
        self.plan_x = np.array([0.0, 0.0, 0.0, self.vx, self.vw])
        pass

    def planThreadFunc(self):
        print("running planning thread!")
        self.need_exit = False
        while not self.need_exit:
            # print("where are you")
            self.planOnce()
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
            time.sleep(0.01)
        print("exit planning thread!")
        self.publishVel(True)
        self.planner_thread = None
        pass

    def planOnce(self):
        # print("plan once dwa")
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [0.0, 0.0, 0.0, self.vx, self.vw]
        # Update obstacle
        self.updateObstacle()
        # print("plan_goal",self.plan_goal)
        u, _ = self.dwa.plan(self.plan_x, self.plan_goal, self.plan_ob)
        # print(u)
        # print("v ,w:", u[0], u[1])

        alpha = 0.5
        self.vx = u[0] * alpha + self.vx * (1 - alpha)
        self.vw = u[1] * alpha + self.vw * (1 - alpha)
        # print("mdbg; ",u)

        self.publishVel()
        pass

    def publishVel(self, zero=False):
        if zero:
            self.vx = 0
            self.vw = 0
        print("v ,w:", self.vx, self.vw)
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)


def main():
    print("this is LOCAL PLANNER")
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()
    pass


if __name__ == '__main__':
    main()
