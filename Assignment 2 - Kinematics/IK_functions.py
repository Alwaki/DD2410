#! /usr/bin/env python3

"""
    # {Alexander Wallen Kiessling}
    # {akie@kth.se}
"""

# SETUP
# ---------------------

# Import packages
import math as m


# FUNCTION DEFINITIONS
# ---------------------

# Description: calculates the inverse kinematics of a 3-DOF robot (RRP)
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

# Description:
# In:
# Out:
def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q

# DRIVER CODE (Testing purposes)
# ---------------------

#if __name__ == '__main__':
#    print(scara_IK([0.57,-0.15,0.1]))
