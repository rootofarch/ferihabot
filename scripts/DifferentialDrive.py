#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
import math

class FerihaBotControl:
    def __init__(self):
        self.cmd_vel = rospy.Publisher("/alpha/cmd_vel", Twist, queue_size=1)
        self.Pose2D_sub = rospy.Subscriber("/alpha/pose2d", Pose2D, pose2d_callback, queue_size=1)
        self.rate = rospy.Rate(10)
        self.My_Pose2D
        
class target:
    x = 5
    y = 5
        
def movement(self, ang_z, lin_x):   
    movement_msg = Twist()
    
    movement_msg.linear.x = lin_x
    movement_msg.angular.z = ang_z
    
    self.cmd_vel.publish(movement_msg)

def pose2d_callback(self, data):
    self.My_Pose2D = data

def breakes(self):
    movement(0, 0) #break

def am_I_close(self, target, radius):
    #(x-a)^2+(y-b)^2 = radius^2
    my_x = self.My_Pose2D.x
    my_y = self.My_Pose2D.y
    my_pose = self.My_Pose2D.theta
    
    t_x = target.x
    t_y = target.y
   
    if (((t_x-my_x)**2)+((t_y-my_y)**2)) < radius**2:
        return True
    else:
        return False

def find_target_direction(target, my_x, my_y):
    norm_x = target.x - my_x
    norm_y = target.y - my_y
    
    target_direction = math.atan2(norm_y, norm_x)
    
    return target_direction
    
def GoToPointLine(self, target):
    my_x = self.My_Pose2D.x
    my_y = self.My_Pose2D.y
    my_pose = self.My_Pose2D.theta
    
    t_x = target.x
    t_y = target.y
    target_direction = find_target_direction(target, my_x, my_y)
    
    while math.fabs(my_pose - target_direction) > 0.1:
        movement(math.radians(20), 0)
    breakes()
    while not(am_I_close(target, 0.1)):
        movement(0, 1)
    breakes()
    
def GoToPointCurve(self, target):
    my_x = self.My_Pose2D.x
    my_y = self.My_Pose2D.y
    my_pose = self.My_Pose2D.theta
    
    t_x = target.x
    t_y = target.y
    
    while not(am_I_close(target, 0.1)):
        target_direction = find_target_direction(target, my_x, my_y)
        mlt_prmtr = math.fabs(my_pose - target_direction)
        movement(1*mlt_prmtr, 1)
