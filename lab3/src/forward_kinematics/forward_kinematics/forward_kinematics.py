#!usr/bin/env python

import numpy as np
import scipy as sp
from .kin_func_skeleton import prod_exp

def ur7e_foward_kinematics_from_angles(joint_angles):
    """
    Calculate the orientation of the ur7e's end-effector tool given
    the joint angles of each joint in radians

    Parameters:
    ------------
    joint_angles ((6x) np.ndarray): 6 joint angles (s0, s1, e0, w1, w2, w3)

    Returns: 
    ------------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    q0 = np.ndarray((3, 6)) # Points on each joint axis in the zero config
    w0 = np.ndarray((3, 6)) # Axis vector of each joint axis in the zero config


    q0[:, 0] = [0., 0., 0.1625] # shoulder pan joint - shoulder_link
    q0[:, 1] = [0., 0., 0.1625] # shoulder lift joint - upper_arm_link
    q0[:, 2] = [0.425, 0., 0.1625] # elbow_joint - forearm_link
    q0[:, 3] = [0.817, 0.1333, 0.1625] # wrist 1 - wrist_1_link
    q0[:, 4] = [0.817, 0.1333, 0.06285] # wrist 2 - wrist_2_link
    q0[:, 5] = [0.817, 0.233, 0.06285] # wrist 3 - wrist_3_link

    w0[:, 0] = [0., 0., 1] # shoulder pan joint
    w0[:, 1] = [0, 1., 0] # shoulder lift joint
    w0[:, 2] = [0., 1., 0] # elbow_joint
    w0[:, 3] = [0., 1., 0] # wrist 1
    w0[:, 4] = [0., 0., -1] # wrist 2 
    w0[:, 5] = [0., 1., 0] # wrist 3

    # Rotation matrix from base_link to wrist_3_link in zero config
    R = np.array([[-1., 0., 0.],
                  [0., 0., 1.], 
                  [0., 1., 0.]])

    # YOUR CODE HERE (Task 1)
    twist_list = np.zeros((6, 7))

    #shoulder pan joint
    qw0 = (np.cross(q0[:, 0], w0[:, 0])).T
    omega0 = w0[:, 0].T
    print("omega dimension: " + str(qw0.shape))
    xi0 = np.vstack([qw0, omega0])
    print("xi0 dimension: " + str(qw0.shape))
    twist_list[:, 0] = xi0

    #shoulder lift joint
    qw1 = (np.cross(q0[:, 1], w0[:, 1])).T
    omega1 = w0[:, 1].T
    xi1 = np.vstack([qw1, omega1])
    twist_list[:, 1] = xi1

    #elbow joint
    qw2 = (np.cross(q0[:, 2], w0[:, 2])).T
    omega1 = w0[:, 2].T
    xi2 = np.vstack([qw2, omega2])
    twist_list[:, 2] = xi2

    #wrist1
    qw3 = (np.cross(q0[:, 3], w0[:, 3])).T
    omega3 = w0[:, 3].T
    xi3 = np.vstack([qw3, omega3])
    twist_list[:, 3] = xi3

    #wrist2
    qw4 = (np.cross(q0[:, 4], w0[:, 4])).T
    omega4 = w0[:, 4].T
    xi4 = np.vstack([qw4, omega4])
    twist_list[:, 4] = xi4

    #wrist3
    qw5 = (np.cross(q0[:, 5], w0[:, 5])).T
    omega5 = w0[:, 5].T
    xi5 = np.vstack([qw5, omega5])
    twist_list[:, 5] = xi5

    g0_temp = np.hstack([R, q0[:, 5]])
    g0 = np.vstack([g0_temp, [0, 0, 0, 1]])
    twist_list[:, 6] = g0

    return prod_exp(twist_list, joint_angles)


def ur7e_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the ur7e's end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of ur7e robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(6)
    # YOUR CODE HERE (Task 2)
    for i in range(6):
        curr = joint_state.name[i]
        curr_int = 0

        if (curr == "shoulder_lift_joint"):
            curr_int = 1

        elif(curr == "elbow_joint"):
            curr_int = 2

        elif (curr == "wrist_1_joint"):
            curr_int = 3

        elif (curr == "wrist_2_joint"):
            curr_int = 4

        elif (curr == "wrist_3_joint"):
            curr_int = 5

        elif (curr == "shoulder_pan_joint"):
            curr_int = 0

        angles[curr_int] = joint_state.position[i]

    return ur7e_foward_kinematics_from_angles(joint_state)

    # END YOUR CODE HERE
