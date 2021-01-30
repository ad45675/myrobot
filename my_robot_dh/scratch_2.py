import sim as vrep
import time
import math
import numpy as np
import cv2 as cv
import PIL.Image as Image
import os
from Rot2RPY import Rot2RPY,euler2mat,Rot2RPY_version2

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

#----------------------------第一台相機的handle
_,camera_handle=vrep.simxGetObjectHandle(clientID,'kinect', vrep.simx_opmode_blocking)
_,kinectRGB_handle=vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_blocking)
_,kinectDepth_handle=vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_blocking)

#----------------------------myrobot的handle
_,myrobot_handle=vrep.simxGetObjectHandle(clientID,'my_robot_base', vrep.simx_opmode_blocking)
_,Cuboid_handle=vrep.simxGetObjectHandle(clientID,'Cuboid', vrep.simx_opmode_blocking)

_,cuboid_position=vrep.simxGetObjectPosition(clientID,Cuboid_handle,kinectRGB_handle,vrep.simx_opmode_blocking)
_,cam_position=vrep.simxGetObjectPosition(clientID,kinectRGB_handle,-1,vrep.simx_opmode_blocking)

print('---------------word frame----------------')
print('cam',cam_position)

matrix=np.array([
    [1.4841556549072e-05, 1, -8.9979948825203e-06, -0.033098947256804],
    [1, -1.5020370483398e-05, 1.6886435332708e-05, -0.048389494419098],
    [1.6886300727492e-05, -8.998245903058e-06, -1.0000002384186, 1.0709483623505],
    [0,0,0,1]
])
matrixM=np.array([
    [0, 1, 0, 0.60238933563232],
    [1, 0, 0, -0.010899987071753],
    [0, 0, -1, 1.1207495927811],
    [0,0,0,1]
])


cub=np.array([[0.5539997220039368, -0.04399874433875084, 0.04980131983757019,1]])
cub=np.reshape(cub,(4,1))
cub_in_cam=np.array([cuboid_position[0],cuboid_position[1],cuboid_position[2],1])
print('cub_in_cam',cub_in_cam)
cub_in_world=np.dot(matrixM,cub_in_cam)
print('cub_in_world',cub_in_world)


euler_angle=[-3.14,0,-1.57]
rotm=euler2mat(-3.14,0,-1.57)
print('rotm',rotm)
new_world_cub=np.dot(rotm,cuboid_position)+cam_position
print('new_world_cuboid',new_world_cub)