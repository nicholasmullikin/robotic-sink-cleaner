import numpy as np
import pybullet as p
import math
from plyfile import PlyData, PlyElement


def get_point_parameters(curr, final, step, total):
	inst = np.array(curr[:9]) + (step / total) * (np.array(final) - np.array(curr[:9]))
	return inst


init_camera_vector = (0, 0, 1)  # z-axis
init_up_vector = (0, 1, 0)  # y-axis


def panda_camera_view(pandaId):
	end_effector_pos, end_effector_orient, _, _, _, _ = p.getLinkState(pandaId, 11, computeForwardKinematics=False)
	rot_matrix = p.getMatrixFromQuaternion(end_effector_orient)
	rot_matrix = np.array(rot_matrix).reshape(3, 3)

	camera_vector = rot_matrix.dot(init_camera_vector)
	up_vector = rot_matrix.dot(init_up_vector)

	view_matrix = p.computeViewMatrix(
		end_effector_pos,
		end_effector_pos + 0.1 * camera_vector,
		up_vector
	)

	return view_matrix


def from_opengl_depth_to_distance(depthImg, near, far):
	return far * near / (far - (far - near) * depthImg)


def distance_to_point_cloud(distances, fov, width, height):
	f = height / (2 * math.tan(fov / 2.0))
	px = np.tile(np.arange(width), [height, 1])
	x = (2 * (px + 0.5) - width) / f * distances / 2
	py = np.tile(np.arange(height), [width, 1]).T
	y = (2 * (py + 0.5) - height) / f * distances / 2
	point_cloud = np.stack((x, y, distances), axis=-1)
	return point_cloud


def get_point_cloud(img, near, far, fov, pandaId):
	depth = img[3]
	distances = from_opengl_depth_to_distance(np.array(depth), near, far)
	point_cloud = distance_to_point_cloud(
		distances, fov / 180.0 * np.pi, width=distances.shape[1], height=distances.shape[0])
	point_cloud = from_camera_frame_to_world_frame(point_cloud, pandaId)
	return point_cloud


def plot_point_cloud(pointCloud):
	visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[0, 0, 1, 1], radius=0.01)
	for i in range(pointCloud.shape[0]):
		for j in range(pointCloud.shape[1]):
			new_to = pointCloud[i, j, :]

			# print("plot one point from cloud at ", new_to)
			p.createMultiBody(
				baseMass=0,
				baseCollisionShapeIndex=-1,
				baseVisualShapeIndex=visualShapeId,
				basePosition=new_to,
				useMaximalCoordinates=True
			)


def from_camera_frame_to_world_frame(pointCloud, pandaId):
	camera_pos, camera_orient, _, _, _, _ = p.getLinkState(pandaId, 11, computeForwardKinematics=False)
	camera_to_world = p.multiplyTransforms(
		[0, 0, 0], camera_orient,
		[0, 0, 0], p.getQuaternionFromEuler([0, 0, np.pi])
	)[1]
	for i in range(pointCloud.shape[0]):
		for j in range(pointCloud.shape[1]):
			pointCloud[i, j, :] = (p.multiplyTransforms(
				camera_pos, camera_to_world,
				pointCloud[i, j, :], [0, 0, 0, 1])[0]
			)

	return pointCloud


def to_ply_file(np_array):
	data = np_array.transpose(2, 0, 1).reshape(3, -1).transpose(1, 0)

	ply_faces = np.empty(len(data), dtype=[('vertex_indices', 'i4', (3,))])
	ply_faces['vertex_indices'] = data

	el = PlyElement.describe(ply_faces, 'sink')
	PlyData([el]).write('sink_point_cloud.ply')


def get_dish_pos_and_orient(dishId):
	dishInfo = p.getBasePositionAndOrientation(dishId)
	return dishInfo


def get_all_joint_limits(pandaId):
	lowerLimits = []
	upperLimits = []
	for i in range(11):
		jointInfo = p.getJointInfo(pandaId, i)
		lowerLimits += [jointInfo[8]]
		upperLimits += [jointInfo[9]]
	return [lowerLimits, upperLimits]
