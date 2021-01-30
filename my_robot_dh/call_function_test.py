import inverseKinematics as IK
import numpy as np
import math

DH_table = np.array([[0,         34.5,   8.0,     math.pi/2],
                     [math.pi/2, 0,      27.0,    0],
                     [0,         0,      9.0,     math.pi/2],
                     [0,         29.5,   0,       -math.pi/2],
                     [0,         0,      0,       math.pi/2],
                     [0,         10.2,   0,       0]])

InitialPose = np.array(
    [-109.063000000000, -58.4261775843947, -71.842000000000])
InitialPosition = np.array(
    [42.856538283717398, -0.605789981061624, 71.754203625228271])

[a, b] = IK.InverseKinemetics(InitialPose, InitialPosition, DH_table)

# print(a)
#
# print(b)
