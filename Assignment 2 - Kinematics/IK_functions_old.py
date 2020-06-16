#! /usr/bin/env python3

"""
    # {Alexander Wallen Kiessling}
    # {akie@kth.se}
"""

# SETUP
# ---------------------

# Import math packages
import math


# FUNCTION DEFINITIONS
# ---------------------

# Description: calculates the inverse kinematics of a 3-DOF robot (RRP)
# In: a coordinate point vector
# Out: a joint variable vector (in radians)
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

    # Calculate secondary angles
    r = math.hypot(x-l0, y)
    phi1 = math.atan2(y, x-l0)
    phi2 = math.acos((r**2 - l1**2 - l2**2)/(-2 * l1 * l2))
    phi3 = math.acos((l2**2 - r**2 - l1**2)/(-2 * r * l1))

    # Calculate primary angles with secondary angles
    theta1 = (phi1 - phi3)
    theta2 = (math.pi - phi2)

    # Calculate linear distance (z-axis)
    d3 = z

    # Format output vector with calculated values
    q = [theta1, theta2, d3]

    return q

#     vertices = [[0.27, -0.15, 0], [0.57, -0.15, 0.1], [0.57, 0.15, 0.1], [0.27, 0.15, 0]]

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
