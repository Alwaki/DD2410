#! /usr/bin/env python3

"""
    # {Alexander Wallen Kiessling}
    # {akie@kth.se}
"""

# SETUP
# ---------------------

# Import packages
import math as m
import numpy as np

# PART E - SCARA
# ---------------------

# Description: calculates the inverse kinematics of a 3-DOF robot arm (RRP)
# In: a coordinate point vector
# Out: a joint variable vector (in radians & L. distance)
def scara_IK(point):
    # Reformat input as coordinates
    x = point[0]
    y = point[1]
    z = point[2]

    # Declare and initialize output vector
    q = [0.0, 0.0, 0.0]

    # Define link lengths
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    # Calculations
    r = m.hypot(x-l0, y)
    cos_phi = (r**2 - l1**2 - l2**2)/(2*l1*l2)
    sin_phi = m.sqrt(1-cos_phi**2)
    q[0] = m.atan2(y, x-l0) - m.atan2(l2*sin_phi, l1 + l2*cos_phi)
    q[1] = m.atan2(sin_phi, cos_phi)
    q[2] = z

    return q

# PART C - KUKA
# ---------------------

# Description: An auxiliary function which transforms dh parameters into a transform matrix
# In: dh parameters
# Out: transform mapping matrix
def transform(alpha, d, a, theta):

    A = np.array(
        [[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)],
         [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(alpha)],
         [0,m.sin(alpha), m.cos(alpha), d],
         [0,0,0,1]
        ])

    return A

# Description: Differential kinematics calculation function for 7 DOF robot arm (RRRRRRR)
# In: A coordinate point vector, rotation matrix, and previous joint values as array.
# Out: Calculated joint values as array
def kuka_IK(point, R, joint_positions):

    # Declarations
    L = 0.4
    M = 0.39
    N = 0.311
    O = 0.078
    tolerance = 0.01
    iterations = 1

    q = joint_positions #it must contain 7 elements

    # Calculate A matrices from given dh parameters and current joint variables
    A1 = transform(m.pi/2,  0, 0, q[0])
    A2 = transform(-m.pi/2, 0, 0, q[1])
    A3 = transform(-m.pi/2, L, 0, q[2])
    A4 = transform(m.pi/2,  0, 0, q[3])
    A5 = transform(m.pi/2,  M, 0, q[4])
    A6 = transform(-m.pi/2, 0, 0, q[5])
    A7 = transform(0,       0, 0, q[6])

    # Create transfer matrices
    T0 = np.identity(4) #T0 is displacement factor due to bottom link length
    T0[2][3] = N
    T1 = np.ndarray.round(A1, 5)
    T2 = np.ndarray.round(np.linalg.multi_dot([T1, A2]), 5)
    T3 = np.ndarray.round(np.linalg.multi_dot([T2, A3]), 5)
    T4 = np.ndarray.round(np.linalg.multi_dot([T3, A4]), 5)
    T5 = np.ndarray.round(np.linalg.multi_dot([T4, A5]), 5)
    T6 = np.ndarray.round(np.linalg.multi_dot([T5, A6]), 5)
    T7 = np.ndarray.round(np.linalg.multi_dot([T6, A7]), 5)
    T8 = np.identity(4) #T8 is displacement factor due to end effector length
    T8[2][3] = O
    T8 = np.linalg.multi_dot([T7,T8])
    T_complete = np.linalg.multi_dot([T0,T8]) # The complete transform from base to end

    # Creating z (rotational part of revolute joint)
    z1 = [0, 0, 1]
    z2 = T1[0:3,2]
    z3 = T2[0:3,2]
    z4 = T3[0:3,2]
    z5 = T4[0:3,2]
    z6 = T5[0:3,2]
    z7 = T6[0:3,2]

    # Creating p (linear part of revolute joint)
    p1 = [0, 0, 0]
    p2 = T1[0:3,3]
    p3 = T2[0:3,3]
    p4 = T3[0:3,3]
    p5 = T4[0:3,3]
    p6 = T5[0:3,3]
    p7 = T6[0:3,3]
    p8 = T7[0:3,3] # excessive, p7 is undisplaced version of p
    p  = T8[0:3,3].T

    # Create jacobian columns
    J1 = np.hstack((np.cross(z1,p-p1), z1))
    J2 = np.hstack((np.cross(z2,p-p2), z2))
    J3 = np.hstack((np.cross(z3,p-p3), z3))
    J4 = np.hstack((np.cross(z4,p-p4), z4))
    J5 = np.hstack((np.cross(z5,p-p5), z5))
    J6 = np.hstack((np.cross(z6,p-p6), z6))
    J7 = np.hstack((np.cross(z7,p-p7), z7))

    # Concatenate columns into jacobian matrix, and transpose to get 6x7 dimensions
    J = np.ndarray.round(np.transpose(np.vstack((J1, J2, J3, J4, J5, J6, J7))), 5)

    # Solve inverse jacobian (pseudo-inverse due to non-square matrix), becomes 7x6
    J_inv = np.ndarray.round(np.linalg.pinv(J),5)

    # Create positional error by comparing desired position and forward kinematics
    x_error = point[0] - T_complete[0][3]
    y_error = point[1] - T_complete[1][3]
    z_error = point[2] - T_complete[2][3]
    position_error = np.hstack((x_error, y_error, z_error))

    # Create orientation error by comparing desired rotation and forward kinematics
    # Note: See page 139 from robotics book for description and formula
    nd = np.transpose(R)[0]
    sd = np.transpose(R)[1]
    ad = np.transpose(R)[2]
    print(T_complete)
    ne = np.transpose(T_complete)[0:2]
    print(ne)
    se = T_complete[1]
    ad = T_complete[2]


    orientation_error = (np.cross(ne.T, nd.T).T + np.cross(se.T, sd.T).T + np.cross(ae.T, ad.T).T)/2

    # From Lecture 4, iterative solution:
    # Step 1: x_hat = K(theta_hat)
    #x_hat = J.dot(q)
    # Step 2: epsilon_x = x_hat - x
    #epsilon_x = x_hat - pose
    # Step 3: epsilon_theta = inv_J(theta_hat) * epsilon_x
    #epsilon_theta = J_inv.dot(epsilon_x)
    # Step 4: theta_hat = theta_hat - epsilon_theta
    #q = q - epsilon_theta
    # Step 5: check if epsilon_x <= tolerance

    return None

# DRIVER CODE (Testing purposes)
# ---------------------

if __name__ == '__main__':
    kuka_IK([-0.217, 0, 0.84], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], [0, 1.12, 0, 1.71, 0, 1.84, 0])

