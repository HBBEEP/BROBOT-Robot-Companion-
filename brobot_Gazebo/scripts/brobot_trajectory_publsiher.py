#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from BrobotPos import BrobotPos
from std_msgs.msg import Bool
from std_msgs.msg import Int8

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('brobot_trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_position_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        
        self.actionStatus = self.create_publisher(Bool,"/action_status",10)
        self.create_subscription(Int8, "/action_pub", self.action_callback, 10)

        self.action = 0
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.brobotPos = BrobotPos()
        self.message = Bool()

    def action_callback(self, msg):
        if self.brobotPos.status and msg.data <8 and msg.data >0 :
            self.action = msg.data
            self.brobotPos.countStatus=1
            self.brobotPos.status = False

    def timer_callback(self):
        if self.brobotPos.countStatus>0:
            self.brobotPos.managePos(self.action)  
        elif self.brobotPos.pose :
            self.brobotPos.setpoint_position = [0.0,0.0,0.0,0.0]
            self.brobotPos.pose = False

        self.message.data = self.brobotPos.status
        self.actionStatus.publish(self.message)

        brobot_trajectory_msg = JointTrajectory()
        brobot_trajectory_msg.joint_names = self.brobotPos.jointName

        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.brobotPos.setpoint_position
        point.time_from_start = Duration(sec=1)
        point.velocities = self.brobotPos.jointVelocity

        brobot_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(brobot_trajectory_msg)
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()