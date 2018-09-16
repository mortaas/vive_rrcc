#!/usr/bin/env python  
#import rospy
import rosbag

# ROS msgs
from geometry_msgs.msg import TransformStamped

import sys
import numpy as np
import math3d as m3d
import matplotlib.pyplot as plt

A, B1, B2 = [], [], []

def GetTransform(tf_msg_):
    quat_ = m3d.UnitQuaternion(tf_msg_.transform.rotation.w,
                               tf_msg_.transform.rotation.x,
                               tf_msg_.transform.rotation.y,
                               tf_msg_.transform.rotation.z)
    orient_ = quat_.get_orientation()

    pos_ = m3d.Vector(tf_msg_.transform.translation.x,
                      tf_msg_.transform.translation.y,
                      tf_msg_.transform.translation.z)
    
    return m3d.Transform(orient_, pos_)

def ReadBag(bag):
    if '.bag' in bag:
        bag_ = rosbag.Bag(bag)

        for msg_ in bag_.read_messages(topics='A'):
            A.append(GetTransform(msg_[1]).get_pos().get_array() )
        for msg_ in bag_.read_messages(topics='B1'):
            B1.append(GetTransform(msg_[1]).get_pos().get_array() )
        for msg_ in bag_.read_messages(topics='B2'):
            B2.append(GetTransform(msg_[1]).get_pos().get_array() )
        
        bag_.close()
    else:
        print("The provided path is not a .bag file")

if __name__ == "__main__":
    if len(sys.argv) == 2:
        ReadBag(sys.argv[1])
        error1 = np.linalg.norm(B1, axis=1) - np.linalg.norm(A, axis=1)
        error2 = np.linalg.norm(B2, axis=1) - np.linalg.norm(A, axis=1)

        # print(np.mean(error1) )
        # print(np.std(error1) )
        # print(np.mean(error2) )
        # print(np.std(error2) )

        plt.violinplot(np.abs(error1) )
        plt.grid()
        plt.show()
    
        # plt.violinplot(np.abs(error2) )
        # plt.grid()
        # plt.show()
    else:
        print("Path to .bag file was not provided")