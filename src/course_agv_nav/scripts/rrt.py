# -*- coding: utf-8 -*-
from scipy.spatial import KDTree
import numpy as np
import math
import random


class Node:
    def __init__(self, x, y, cost=0, parent=None, next=[]):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
        self.next = next


class RRT:
    '''
    一些在我的rrt中需要使用到的变量与缓存
    start_node 起点 node
    goal_node 终点 node
    newnode 通过stree生成的像random_node靠近的点 node
    nearestnode 距离random_node最近的点 node
    random_node 随机采样生成的点 node
    barrier_tree 障碍物 KDtree
    route_tree 路径里面的所有点 KDtree
    route_dic 为什么用dictionary呢，因为如果用列表来存储相应的信息，点的信息添加一遍就可以，不能够重复 字典
    route_position 一个列表，每个元素包括一个[x,y]
    finalroute_x ， finalroute_y 其中用于最后传递给action的按顺序的路径
    '''
    def __init__(self, ox, oy, avoid_buffer, robot_radius):
        self.barrier_tree = None
        self.route_tree = None
        self.route_dic = {}
        self.route_position = []
        self.final_route_x = []
        self.final_route_y = []

        self.ox = ox
        self.oy = oy

        self.min_x = -10
        self.max_x = 10
        self.min_y = -10
        self.max_y = 10
        self.robot_size = robot_radius
        self.avoid_dist = 0
        self.step = 0.7
    '''
    构建route_position用于存储点坐标相应的信息
    用route_dic来存储所有的node的具体信息
    构建barrier和routetree来构建相应的KD树
    输入：起点的坐标，终点的坐标，当前场上的情况
    输出：构建起点+终点node，
    '''
    def init_tree(self, start_x, start_y, goal_x, goal_y):
        # Obstacles
        barrier_positions = np.vstack((self.ox, self.oy)).T
        self.barrier_tree = KDTree(barrier_positions)
        self.route_tree = KDTree([(start_x, start_y)])
        start_node = Node(start_x, start_y)
        goal_node = Node(goal_x, goal_y)
        self.route_dic[(start_x, start_y)] = start_node
        self.route_position.append((start_x, start_y))
        return start_node, goal_node
    '''
    随机的生成一个点
    输入：无
    输出：一个在画布内产生的随机的random_node
    '''
    def sample(self):
        return Node(random.randint(self.min_x, self.max_x), random.randint(self.min_y, self.max_y))
    '''
    根据生成的random_node，寻找目前的节点中离它最近的节点
    输入：random_node (靠近的点） 
    输出：nearest_node
    '''
    def nearest(self, random_node):
        _, index = self.route_tree.query(np.array([random_node.x, random_node.y]))
        nearest_node = self.route_dic[self.route_position[index]]
        return nearest_node
    '''
    根据机器人现有的nearest_node和random_node生成一个靠近random_node的路径,
    如果可以直接连到random_node，那么路径直接连接，如果不可以，那么在中间以step为步长取点
    输入：nearest_node random_node
    输出：new_node
    '''
    def steer(self, random_node, nearest_node):
        x1, y1 = random_node.x, random_node.y
        x2, y2 = nearest_node.x, nearest_node.y
        distance = math.hypot(x1 - x2, y1 - y2)
        if distance > self.step:
            ratio = self.step / distance
            new_node = Node(x2 + (x1 - x2) * ratio, y2 + (y1 - y2) * ratio)
        else:
            new_node = random_node
        return new_node
    '''
    无障碍检测，判断从两个点的连线中间没有障碍物，其中每次判断的依据是每个step为长度的点中是否有落在障碍物中的
    输入：new_node nearest_node 
    输出：true 没有障碍 flase 有障碍
    '''
    def collision_free(self, new_node, nearest_node):
        # x1, y1 = new_node.x, new_node.y
        # x2, y2 = nearest_node.x, nearest_node.y
        # distance = math.hypot(x1 - x2, y1 - y2)
        # if distance > 0:
        #     x = np.linspace(x2, x1, int(distance/50)+1)[1:]
        #     y = np.linspace(y2, y1, int(distance/50)+1)[1:]
        #     for i, j in zip(x, y):
        #         if self.barrier_tree.query(np.array([i, j]))[0] < self.robot_size+self.avoid_dist:
        #             return False
        # return True
        dx = nearest_node.x - new_node.x
        dy = nearest_node.y - new_node.y
        ang = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        x1 = new_node.x
        y1 = new_node.y
        step_size = self.robot_size
        steps = int(round(dis / step_size,0))
        for i in range(steps + 1):
            distance, index = self.barrier_tree.query(np.array([x1, y1]))
            if distance <= self.robot_size + self.avoid_dist:
                return False
            x1 += step_size * math.cos(ang)
            y1 += step_size * math.sin(ang)
        return True

    '''
    找到某个点周围的离它比较近的节点，一般的方式是其周围画圈来进行寻找，选取一定距离范围内的节点，如果该距离范围内没有，就返回离它最近的节点
    输入：new_node radius
    输出：一个nodes的列表
    '''
    def find_near_nodes(self, new_node, radius):
        NEAR_NODES = []
        idx_list = self.route_tree.query_ball_point([new_node.x, new_node.y], radius)
        if len(idx_list)!=0:
            for idx in idx_list:
                near_node = self.route_dic[self.route_position[idx]]
                NEAR_NODES.append(near_node)
        else:
            _, indices = self.route_tree.query(np.array([new_node.x, new_node.y]),1)
            indices = [indices]
            for i in indices:
                near_nodes = self.route_dic[self.route_position[i]]
                NEAR_NODES.append(near_nodes)
        return NEAR_NODES
    '''
    传入newnode和nearnodes，要看newnode选择哪个作为父节点，能够使其到初始点的距离最小
    然后需要使用obstree，如果两个节点之间不能够直接连在一起，需要舍弃
    由于如果没有最小的节点的话，可能会没有返回值，所以一般要把nearestnode作为保底的节点，能够永远有一个节点被return
    输入：NEAR_NODES new_node barrier_tree nearestnode
    输出：
    '''
    def ChooseParent(self, NEAR_NODES, new_node, nearest_node):
        min_cost = nearest_node.cost + math.hypot(new_node.x - nearest_node.x, new_node.y - nearest_node.y)
        min_node = nearest_node
        for near_node in NEAR_NODES:
            cost = near_node.cost + math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)
            if self.collision_free(new_node, near_node) and cost < min_cost:
                min_node = near_node
                min_cost = cost
        return min_node, min_cost
    '''
    实现的功能是把之前的newnode和他的父节点加入到缓存中
    输入：min_node, new_node, min_cost, 
    输出：更新后的 route_position, route_tree ,route_dic
    '''
    def addNodeEdge(self, min_node, new_node, min_cost):
        new_node.cost = min_cost
        new_node.parent = min_node

        min_node.next.append([new_node])
        new_node.next = []

        self.route_dic[(new_node.x, new_node.y)] = new_node
        self.route_dic[(min_node.x, min_node.y)] = min_node

        self.route_position.extend([(new_node.x, new_node.y), (min_node.x, min_node.y)])
        self.route_tree = KDTree(self.route_position)

    '''
    对之前的NEAR_NODES做重新布线，如果其以new_node作为父节点，能够减少cost，则new_node为其父节点，其转换为子节点
    如果没有更新，则直接返回原node
    输入：NEAR_NODES, new_node, barrier
    输出：near_node, new_node
    '''
    def rewire(self, NEAR_NODES, new_node):
        for near_node in NEAR_NODES:
            cost = new_node.cost + math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)
            if self.collision_free(new_node, near_node) and cost < near_node.cost:

                near_node.parent = new_node

                near_node.cost = new_node.cost + math.hypot(new_node.x - near_node.x,new_node.y - near_node.y)
                new_node.next.append(near_node)
                return near_node, new_node

        return near_node, new_node

    def get_tree(self, last_node):
        # Initialize variables
        node_list = []
        current_node = last_node
        self.route_dic[(last_node.x, last_node.y)] = last_node

        # Traverse the tree backwards to retrieve the path
        while current_node is not None:
            node_list.insert(0,current_node)
            current_node = current_node.parent

        # Extract the path coordinates from the nodes
        route_x = [node.x for node in node_list]
        route_y = [node.y for node in node_list]

        # Remove any nodes that violate collision constraints
        i = 0
        while i < len(route_x) - 2:
            if self.collision_free(self.route_dic[(route_x[i], route_y[i])],self.route_dic[(route_x[i + 2], route_y[i + 2])]):
                del route_x[i + 1], route_y[i + 1]
                # print('del')
            else:
                i += 1

        return route_x, route_y
    '''
    算法的流程与核心步骤
    '''
    def search(self,start_x,start_y,goal_x,goal_y):
        start_node, goal_node=self.init_tree( start_x, start_y, goal_x, goal_y)
        # print("start_x="+str(start_x)+"start_y"+str(start_y))
        # print("goal_x=" + str(goal_x) + "goal_y" + str(goal_y))
        #循环

        for counttime in range(0, 1000):
            #采样
            random_node=self.sample()
            #判断最近
            nearest_node=self.nearest(random_node)
            #生成新节点和新轨迹
            new_node=self.steer(random_node,nearest_node)
            #无碰撞检测
            if self.collision_free(new_node,nearest_node):
                NEAR_NODES = self.find_near_nodes( new_node, 3)
                min_node, min_cost = self.ChooseParent(NEAR_NODES, new_node,nearest_node)
                self.addNodeEdge(min_node, new_node, min_cost)
            #如果到达目标点附近
                if self.collision_free(new_node, goal_node) and  math.hypot(new_node.x - goal_node.x, new_node.y - goal_node.y)<self.step:
                    new_node.next = goal_node
                    goal_node.parent = new_node
                    break
                near_node, new_node= self.rewire(NEAR_NODES, new_node)
                self.route_dic[(near_node.x, near_node.y)] = near_node
                self.route_dic[(new_node.x, new_node.y)] = new_node
        if counttime >999:
            print('No route found!')
        finalroute_x, finalroute_y = self.get_tree(goal_node)
        print(finalroute_x)
        print(finalroute_y)
        return finalroute_x, finalroute_y




