#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time
import math
from geometry_msgs.msg import Point

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        self.robot_goal_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sub_point = self.create_subscription(Point, 'trenutna_tocka', self.calculate, 10)
        

        #self.create_timer(1.0 / 10, self.calculate)


    def calculate(self, msg: Point):
        x = msg.x
        y = msg.y
        z = msg.z
        if z >= 0.09: z = 0.09
        h1 = 0.09 - z
        # x, y i z vrha markera na papiru, a, b, g kutevi izračunati inverznom kinematikom, z ne može biti veći od 0.09m
        if x != 0.0: a = math.atan(y/x)
        if x == 0.0: a = math.pi/2
        r = math.sqrt(x**2 + y**2)
        if r <= 0.05: r = 0.05
        if h1 != 0.0: b1 = math.atan(r/h1)
        if h1 == 0.0: b1 = math.pi/2
        a1 = math.atan(h1/r)
        r1 = math.sqrt(r**2 + h1**2)
        b2 = math.acos((0.2**2 + r1**2 - 0.2531**2) / (2 * r1 * 0.2))
        g1 = math.acos((0.2**2 + 0.2531**2 - r1**2) / (2 * 0.2 * 0.2531))
        b = math.pi - b1 - b2
        g = math.pi - g1
        #if g < 0.0: g = 0.0
        #if b < math.pi /2 : b = math.pi / 2
        if a < math.pi * -0.7: a = math.pi * -0.7
        if a > math.pi * 0.7: a = math.pi * 0.7
        a -= math.pi/2
        b = math.pi/2 - b
        a = wrap_to_pi(a)
        b = wrap_to_pi(b)
        g = wrap_to_pi(g)
        
        self.move_robot([1.56, 1.56, -1.56], [b, g, a])
        print("kutevi: ", b, ", ", g, ", ", a)
        return
    
    def move_robot(self, q, q2):
       
        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('joint1') # 90 link1 b
        goal_trajectory.joint_names.append('joint2') # flomaster g
        goal_trajectory.joint_names.append('joint3') # baza a

        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(q2[0])
        goal_point.positions.append(q2[1])
        goal_point.positions.append(q2[2])
        goal_point.time_from_start = Duration(seconds=0.01).to_msg()

        goal_trajectory.points.append(goal_point)

        self.robot_goal_publisher_.publish(goal_trajectory)
        return

def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()