#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt


class Config(object):
    def __init__(self):
        # self.max_speed=
        # self.predict_time=
        self.max_speed = 5  # [m/s]
        self.min_speed = -self.max_speed  # [m/s]
        self.max_accel = 0.2  # *4  # [m/ss]

        self.max_yawrate = 1  # [rad/s]
        self.min_yawrate = -self.max_yawrate  # [rad/s]
        self.max_dyawrate = 0.2  # [rad/ss]

        self.dt = 1.0  # [s] Time tick for motion prediction
        self.predict_time = 1.0

        self.v_reso = 0.05  # [m/s]
        self.yawrate_reso = 0.05  # [rad/s]


class DWA(object):
    def __init__(self, planConfig):
        # planX np.array([0.0, 0.0, 0.0, self.vx, self.vw])
        # planConfig是系列配置
        # planGoal是1x2的数组
        # planOb是nx2的数组
        self.config = planConfig

        self.x = 0
        self.y = 0
        self.orientation = 0

    def set(self, planX, planGoal, planOb):
        self.targetPointX = planGoal[0]
        self.targetPointY = planGoal[1]
        self.obstaclePosition = planOb

    def path(self, v, w):
        n = 8
        dTime = self.config.dt / n
        pathList = np.zeros((n + 1, 3))
        pathList[0] = [self.x, self.y, self.orientation]
        cosVal = math.cos(pathList[0][2])
        sinVal = math.sin(pathList[0][2])
        for i in range(n):
            x = pathList[i][0] + dTime * v * cosVal
            y = pathList[i][1] + dTime * v * sinVal
            r = pathList[i][2] + dTime * w
            pathList[i + 1] = [x, y, r]
            cosVal = math.cos(r)
            sinVal = math.sin(r)
        self.motion = pathList

    def goalCost(self):
        dX = self.targetPointX - self.motion[-1][0]
        dY = self.targetPointY - self.motion[-1][1]
        distance = math.sqrt(dX * dX + dY * dY)
        if distance > 5:
            return 5
        return distance

    def angleCost(self):
        angle1 = math.atan2((self.targetPointY - self.motion[-1][1]),
                            (self.targetPointX - self.motion[-1][0]))
        angle2 = self.motion[-1][2]
        dRotation = angle1 - angle2
        # dRotation = abs(angle1 - angle2)
        # if dRotation > math.pi:
        #     dRotation = 2*math.pi - dRotation
        return abs(dRotation)

    def velocityCost(self, v):
        return abs(v)

    # def obstacleCost(self,v):
    #     minDistance=999999
    #     delta = self.obstaclePosition[:, np.newaxis, :] - self.motion[np.newaxis, :, :2]
    #     # `delta`的形状为 (n_obstacles, n_motion_points, 2)
    #     distance=np.sqrt((delta ** 2).sum(axis=2))
    #     minDist = distance[:,3:].min(axis=0)
    #     # 沿着第三个维度（即每个点与障碍物的距离）取最小值，形状为 (n_motion_points,)
    #     if np.any(minDist < 500):
    #         # print("小心尾巴")
    #         return minDistance
    #     return 1 / min_dist.min()

    def plan(self, planX, planGoal, palnOb):
        minScore = 9999999
        mingoalCost = 9999999
        minobstacleCost = 9999999
        minvelocityCost = 9999999
        minangleCost = 9999999
        bestV, bestW = 0, 0

        self.set(planX, planGoal, palnOb)

        # 生成网格坐标
        vmin = max(self.config.min_speed, planX[3] - self.config.max_accel * self.config.dt)
        vmax = min(self.config.max_speed, planX[3] + self.config.max_accel * self.config.dt)
        wmin = max(-self.config.max_yawrate, planX[4] - self.config.max_dyawrate * self.config.dt)
        wmax = min(self.config.max_yawrate, planX[4] + self.config.max_dyawrate * self.config.dt)
        # vmin = self.config.min_speed
        # vmax = self.config.max_speed
        # wmin = self.config.min_yawrate
        # wmax = self.config.max_yawrate

        plt.ion()  # turn on interactive mode
        for v in np.arange(vmin, vmax, self.config.v_reso):
            for w in np.arange(wmin, wmax, self.config.yawrate_reso):
                self.path(v, w)
                angleCost = 10 * self.angleCost()
                goalCost = 1 * self.goalCost()
                # obstacleCost=   1 *self.obstacleCost(v)
                velocityCost = 0 * self.velocityCost(v)
                score = goalCost + velocityCost + angleCost
                # print(round(v, 2), round(w, 2), round(goalCost, 2), round(angleCost, 2), round(score, 2))
                # print('score:'+str(score)+'    minScore:'+str(minScore))
                if score < minScore:
                    minScore = score
                    # mingoalCost = goalCost
                    # minobstacleCost = obstacleCost
                    # minvelocityCost = velocityCost
                    # minangleCost = angleCost
                    # bestV = v
                    # bestW = w
                    bestMotion = self.motion
                    bestU = np.array(np.array([v, w]))

        plt.clf()  # clear the current figure
        plt.xlim(-4, 4)
        plt.ylim(-4, 4)
        plt.plot(bestMotion[:, 0], bestMotion[:, 1], 'b-')
        plt.plot(planGoal[0], planGoal[1], 'rx')
        plt.draw()  # draw the current figure
        plt.pause(0.01)  # slight pause to allow update of the figure
        # print(bestU)
        # time.sleep(1000)
        return bestU, bestMotion


if __name__ == '__main__':
    config = Config()
    dwa = DWA(config)
    x = [0, 0, 0, 0, 0]
    goal = np.array([10, 10])
    ob = np.array([
        [5, 5],
        [2, 6],
        [5, 6]
    ])
    u, m = dwa.plan(x, goal, ob)
    print(u)
    plt.plot(m[:, 0], m[:, 1], 'b-')
    plt.plot(goal[0], goal[1], 'rx')
    plt.show()
