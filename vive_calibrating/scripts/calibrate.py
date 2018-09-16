#!/usr/bin/env python  
#import rospy
import rosbag

# ROS msgs
from geometry_msgs.msg import TransformStamped

import sys
import numpy as np
from numpy import dot, eye, zeros, outer
from numpy.linalg import inv
import math3d as m3d
import matplotlib.pyplot as plt

A, B1, B2 = [], [], []

# Python implementation of method for solving AX = XB by Torstein A. Myhre
def log(R):
    # Rotation matrix logarithm
    theta = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1.0)/2.0)
    return np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) * theta / (2*np.sin(theta))

def invsqrt(mat):
    u,s,v = np.linalg.svd(mat)
    return u.dot(np.diag(1.0/np.sqrt(s))).dot(v)

def calibrate(A, B):
    #transform pairs A_i, B_i
    N = len(A)
    M = np.zeros((3,3))
    for i in range(N):
        Ra, Rb = A[i][0:3, 0:3], B[i][0:3, 0:3]
        M += outer(log(Rb), log(Ra))

    Rx = dot(invsqrt(dot(M.T, M)), M.T)

    C = zeros((3*N, 3))
    d = zeros((3*N, 1))
    for i in range(N):
        Ra,ta = A[i][0:3, 0:3], A[i][0:3, 3]
        Rb,tb = B[i][0:3, 0:3], B[i][0:3, 3]
        C[3*i:3*i+3, :] = eye(3) - Ra
        d[3*i:3*i+3, 0] = ta - dot(Rx, tb)

    tx = dot(inv(dot(C.T, C)), dot(C.T, d))
    return Rx, tx.flatten()


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
            A.append(GetTransform(msg_[1]).get_array() )
        for msg_ in bag_.read_messages(topics='B1'):
            B1.append(GetTransform(msg_[1]).get_array() )
        for msg_ in bag_.read_messages(topics='B2'):
            B2.append(GetTransform(msg_[1]).get_array() )
        
        bag_.close()
    else:
        print("The provided path is not a .bag file")

if __name__ == "__main__":
    if len(sys.argv) == 2:
        ReadBag(sys.argv[1])

        # Solve AX=XB and return X as 4x4 homogeneous transformation matrix
        X = np.concatenate( (calibrate(A, B1)[0], calibrate(A, B1)[1].reshape([3, 1]) ), 1).reshape([3, 4])
        X = np.append(X, [0, 0, 0, 1]).reshape([4, 4])
        
        # np.set_printoptions(precision=32)
        print("The solution to AX=XB is:")
        print(X)
    else:
        print("Path to .bag file was not provided")