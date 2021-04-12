import numpy as np
import pybullet as p
import math


def get_point_parameters(curr, final, t):
	inst = np.array(curr[:9]) + t * (np.array(final) - np.array(curr[:9]))
	return inst


def panda_camera_view(panda_id):
	end_effector_pos, end_effector_orient, _, _, _, _ = p.getLinkState(panda_id, 11, computeForwardKinematics=False)
	rot_matrix = p.getMatrixFromQuaternion(end_effector_orient)
	rot_matrix = np.array(rot_matrix).reshape(3, 3)

	init_camera_vector = (0, 0, 1)  # z-axis
	init_up_vector = (0, 1, 0)  # y-axis

	camera_vector = rot_matrix.dot(init_camera_vector)
	up_vector = rot_matrix.dot(init_up_vector)

	view_matrix = p.computeViewMatrix(
		end_effector_pos,
		end_effector_pos + 0.1 * camera_vector,
		up_vector
	)

	return view_matrix


def getRayFromTo(mouseX, mouseY):
	width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
	)
	camPos = [
		camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
		camTarget[2] - dist * camForward[2]
	]
	farPlane = 10000
	rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
	lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[2])
	invLen = farPlane * 1. / lenFwd
	rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
	rayFrom = camPos
	oneOverWidth = float(1) / float(width)
	oneOverHeight = float(1) / float(height)
	dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
	dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
	rayToCenter = [
		rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
	]
	ortho = [
		-0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
		-0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
		-0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
	]
	rayTo = [
		rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
		rayFrom[2] + rayForward[2] + ortho[2]
	]
	lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
	alpha = math.atan(lenOrtho / farPlane)

	return rayFrom, rayTo, alpha


def get_joint_info(armId):
	lower_limit = []
	upper_limit = []
	for x in range(11):
		one_joint_info = p.getJointInfo(armId, x)
		lower_limit.append(one_joint_info[8])
		upper_limit.append(one_joint_info[9])

	return [lower_limit, upper_limit]


def get_point_cloud(far, height, img, near, width):
	depth_buffer = img[3]
	img_w = int(width / 10)
	img_h = int(height / 10)
	step_x = 5
	step_y = 5
	point_cloud = []
	for w in range(0, img_w, step_x):
		for h in range(0, img_h, step_y):
			ray_from, ray_to, alpha = getRayFromTo(w * (width / img_w), h * (height / img_h))
			rf = np.array(ray_from)
			rt = np.array(ray_to)
			vec = rt - rf
			depth_img = float(depth_buffer[h, w])
			depth = far * near / (far - (far - near) * depth_img)
			depth /= math.cos(alpha)
			new_to = (depth / np.sqrt(np.dot(vec, vec))) * vec + rf
			point_cloud.append(new_to)
	# print(point_cloud)
