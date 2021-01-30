import sim as vrep
import time
import numpy as np
from Rot2RPY import Rot2RPY,euler2mat,Rot2RPY_version2
from Kinematics import ForwardKinemetics as FK
import math

DegToRad = math.pi/ 180

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
# vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Forward Kinematics
# A set of joint angles are given
joint_1_angle = np.pi/2
joint_2_angle = np.pi/2
joint_3_angle = np.pi/2
joint_4_angle = np.pi/2
joint_5_angle = np.pi/2
joint_6_angle = np.pi/2
joint_angles = np.array([joint_1_angle,joint_2_angle,joint_3_angle,joint_4_angle,joint_5_angle,joint_6_angle])

# plane 位置
result, plane= vrep.simxGetObjectHandle(clientID, 'Plane', vrep.simx_opmode_blocking)
vrep.simxSetObjectPosition(clientID, plane, -1, [0,0,0], vrep.simx_opmode_oneshot)




# Get the R and p for the base frame (Jaco) w.r.t the world frame
# Get the handle of Jaco object
# result, my_robot_handle = vrep.simxGetObjectHandle(clientID, 'my_robot', vrep.simx_opmode_blocking)
# result, link1 = vrep.simxGetObjectHandle(clientID, 'link1', vrep.simx_opmode_blocking)
# result, link2 = vrep.simxGetObjectHandle(clientID, 'link2', vrep.simx_opmode_blocking)
# result, link3 = vrep.simxGetObjectHandle(clientID, 'link3', vrep.simx_opmode_blocking)
# result, link4 = vrep.simxGetObjectHandle(clientID, 'link4', vrep.simx_opmode_blocking)
# result, link5 = vrep.simxGetObjectHandle(clientID, 'link5', vrep.simx_opmode_blocking)
# result, link6 = vrep.simxGetObjectHandle(clientID, 'link6', vrep.simx_opmode_blocking)
#
# # set the position of base frame [0,0,0]
# # vrep.simxSetObjectPosition(clientID, my_robot_handle, plane, [0,0,0], vrep.simx_opmode_blocking)
# result, p = vrep.simxGetObjectPosition(clientID,my_robot_handle,plane,vrep.simx_opmode_blocking)
# result, p1 = vrep.simxGetObjectPosition(clientID,link1,plane,vrep.simx_opmode_blocking)
# result, p2 = vrep.simxGetObjectPosition(clientID,link2,plane,vrep.simx_opmode_blocking)
# result, p3 = vrep.simxGetObjectPosition(clientID,link3,plane,vrep.simx_opmode_blocking)
# result, p4 = vrep.simxGetObjectPosition(clientID,link4,plane,vrep.simx_opmode_blocking)
#
# err, minval = vrep.simxGetObjectFloatParameter(clientID, my_robot_handle, vrep.sim_objfloatparam_modelbbox_min_z,vrep.simx_opmode_blocking)
# err, maxval = vrep.simxGetObjectFloatParameter(clientID, my_robot_handle, vrep.sim_objfloatparam_modelbbox_max_z,vrep.simx_opmode_blocking)
# hight=(maxval - minval) / 2
#
# p4[0]=p3[0]+0.10854+0.113685
# vrep.simxSetObjectPosition(clientID, link4, plane, p4, vrep.simx_opmode_blocking)
# result, p = vrep.simxGetObjectPosition(clientID,link1,plane,vrep.simx_opmode_blocking)
# print('p',p )

# Get the orientation of base w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, my_robot_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get Jaco orientation')

# R_base_in_world = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
# p_base_in_world = np.reshape(p,(3,1))
R_base_in_world = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
p_base_in_world = np.reshape(p,(3,1))

#加吸嘴要加上0.0535
DH_table = np.array([[0, 0.345, 0.08, math.pi / 2],
					 [0+math.pi / 2, 0, 0.27, 0],
					 [0, 0, 0.09, math.pi / 2],
					 [0, 0.295, 0, -math.pi / 2],
					 [0, 0, 0, math.pi / 2],
					 [0, 0.102+0.0535, 0, 0]])

JointAngle=[0,0,0,0,0,0]
Info,EulerAngle,Position=FK(DH_table , JointAngle)

position0 = np.zeros((3,), np.float32)
position1 = np.zeros((3,), np.float32)
position2 = np.zeros((3,), np.float32)
position3 = np.zeros((3,), np.float32)
position4 = np.zeros((3,), np.float32)
position5 = np.zeros((3,), np.float32)
position6 = np.zeros((3,), np.float32)
joint_dir0 = np.zeros((3,), np.float32)
joint_dir1 = np.zeros((3,), np.float32)
joint_dir2 = np.zeros((3,), np.float32)
joint_dir3 = np.zeros((3,), np.float32)
joint_dir4 = np.zeros((3,), np.float32)
joint_dir5 = np.zeros((3,), np.float32)
joint_dir6 = np.zeros((3,), np.float32)
EulerAngle0 = np.zeros(3)
for i in range(3):
	position0[i] = Info[0][0][i]
	position1[i] = Info[0][1][i]
	position2[i] = Info[0][2][i]
	position3[i] = Info[0][3][i]
	position4[i] = Info[0][4][i]
	position5[i] = Info[0][5][i]
	position6[i] = Info[0][6][i]

joint_dir0 = [0.0, 0.0, 0.0]
# joint_dir1[0], joint_dir1[1], joint_dir1[2] = Rot2RPY(Info[1][1])
# joint_dir2[0], joint_dir2[1], joint_dir2[2] = Rot2RPY(Info[1][2])
# joint_dir3[0], joint_dir3[1], joint_dir3[2] = Rot2RPY(Info[1][3])
# joint_dir4[0], joint_dir4[1], joint_dir4[2] = Rot2RPY(Info[1][4])
# joint_dir5[0], joint_dir5[1], joint_dir5[2] = Rot2RPY(Info[1][5])
# joint_dir6[0], joint_dir6[1], joint_dir6[2] = Rot2RPY(Info[1][6])

joint_dir1[0], joint_dir1[1], joint_dir1[2] = Rot2RPY_version2(Info[1][1])
joint_dir2[0], joint_dir2[1], joint_dir2[2] = Rot2RPY_version2(Info[1][2])
joint_dir3[0], joint_dir3[1], joint_dir3[2] = Rot2RPY_version2(Info[1][3])
joint_dir4[0], joint_dir4[1], joint_dir4[2] = Rot2RPY_version2(Info[1][4])
joint_dir5[0], joint_dir5[1], joint_dir5[2] = Rot2RPY_version2(Info[1][5])
joint_dir6[0], joint_dir6[1], joint_dir6[2] = Rot2RPY_version2(Info[1][6])

for i in range(3):
	# joint_dir1[i]=DegToRad*joint_dir1[i]
	# joint_dir2[i]=DegToRad*joint_dir2[i]
	# joint_dir3[i]=DegToRad*joint_dir3[i]
	# joint_dir4[i]=DegToRad*joint_dir4[i]
	# joint_dir5[i]=DegToRad*joint_dir5[i]
	# joint_dir6[i]=DegToRad*joint_dir6[i]
	joint_dir1[i]=joint_dir1[i]
	joint_dir2[i]=joint_dir2[i]
	joint_dir3[i]=joint_dir3[i]
	joint_dir4[i]=joint_dir4[i]
	joint_dir5[i]=joint_dir5[i]
	joint_dir6[i]=joint_dir6[i]

dummy_pos=[position0,position1,position2,position3,position4,position5,position6]
dummy_dir=[joint_dir0,joint_dir1,joint_dir2,joint_dir3,joint_dir4,joint_dir5,joint_dir6]

print('dir',dummy_dir)
# print('info',position6)
# print('EulerAngle ',EulerAngle)
# print('Position',Position)


# Second, obtain the initial pose of the end effector (dummy7 frame)
# Get the handle of dummy7 object
dummy_handle=np.zeros((7,),np.float32)
joint_handle=np.zeros((7,),np.float32)
Rjoint_handle=np.zeros((7,),np.float32)
ori=np.zeros((7,3),np.float32)

#position
for i in range(7):
	result, dummy_handle[i] = vrep.simxGetObjectHandle(clientID, 'Dummy'+str(i), vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get dummy7 handle')
	# result, joint_handle[i] = vrep.simxGetObjectHandle(clientID, 'joint' + str(i+1), vrep.simx_opmode_blocking)
	result, Rjoint_handle[i] = vrep.simxGetObjectHandle(clientID, 'joint' + str(i+1 ), vrep.simx_opmode_blocking)
# print(dummy_dir)
for i in range(7):
	result = vrep.simxSetObjectPosition(clientID, dummy_handle[i], plane, dummy_pos[i], vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get dummy ball handle')
	# vrep.simxSetObjectPosition(clientID, joint_handle[i], -1, dummy_pos[i], vrep.simx_opmode_blocking)
	vrep.simxSetObjectPosition(clientID, Rjoint_handle[i], plane, dummy_pos[i], vrep.simx_opmode_blocking)


##-------------------------orientation------------------------------##

for i in range(7):

	# res,ori[i]=vrep.simxGetObjectOrientation(clientID,dummy_handle[i],dummy_handle[0],vrep.simx_opmode_blocking)
	vrep.simxSetObjectOrientation(clientID,dummy_handle[i], plane, dummy_dir[i], vrep.simx_opmode_oneshot)
	vrep.simxSetObjectOrientation(clientID, Rjoint_handle[i],plane, dummy_dir[i], vrep.simx_opmode_oneshot)
	res,ori[i]=vrep.simxGetObjectOrientation(clientID,Rjoint_handle[i],plane,vrep.simx_opmode_blocking)
print(ori)


time.sleep(1)
# Next, set all the joints to the given joint angles
# Obtain the handles of the six joints by calling the helper function GetJointsHandle in another file 'get_handle.py'
# joint_one_handle,joint_two_handle,joint_three_handle,joint_four_handle,joint_five_handle,joint_six_handle = GetJointsHandle(clientID)

time.sleep(1)
# create a list to store the current joint angles of all six joints
current_theta = np.zeros(6)

# Iterate the following code block 6 times to move each individual joint one by one
# for i in range(6):
# 	if i == 0:
# 		curr_handle = joint_one_handle
# 	elif i == 1:
# 		curr_handle = joint_two_handle
# 	elif i == 2:
# 		curr_handle = joint_three_handle
# 	elif i == 3:
# 		curr_handle = joint_four_handle
# 	elif i == 4:
# 		curr_handle = joint_five_handle
# 	elif i == 5:
# 		curr_handle = joint_six_handle
#
# 	# Get the current value of the current joint
# 	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
# 	if result != vrep.simx_return_ok:
# 		raise Exception('could not get joint variable #{}'.format(i+1))
#
# 	# Wait two seconds
# 	time.sleep(1)
#
# 	# Set the desired value of the current joint variable
# 	vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + joint_angles[i], vrep.simx_opmode_oneshot)
#
# 	# Wait two seconds
# 	time.sleep(1)
#
# 	# Get the new value of the current joint
# 	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
# 	current_theta[i] = theta
# 	if result != vrep.simx_return_ok:
# 		raise Exception('could not get joint variable #{}'.format(i+1))
# 	print('New value - joint #{}: theta = {:f}'.format(i+1,theta))
#
#
#
# time.sleep(1)
#
# # Print the actual pose of tool frame (dummy7) in the end
# # Get the position of dummy7 frame w.r.t the world frame
# result, p = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
# 	raise Exception('could not get dummy7 position')
#
# # Get the orientation of dummy7 w.r.t the world frame
# result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
# 	raise Exception('could not get dummy7 orientation')
#
# p_end = np.reshape(p,(3,1))
# R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
# T_actual = T_from_R_p(R_end,p_end)
# pprint(T_actual)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)