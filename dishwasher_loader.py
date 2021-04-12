from utils import *
import pybullet as p
import pybullet_data as p_data
from operator import add, sub
import numpy as np
from collisionUtils import get_collision_fn


sinkStartPos = [-0.25, 0, 0.5]
pandaStartPos = [-0.25, -0.75, 0]


# Connect to existing physics engine, if any, else create new physics engine with graphical front-end
# Set up other paths, etc.
def start_engine():
	connection_id = p.connect(p.SHARED_MEMORY)
	if connection_id < 0:
		p.connect(p.GUI)
	p.setAdditionalSearchPath(p_data.getDataPath())
	p.setPhysicsEngineParameter(numSolverIterations=10)
	p.setTimeStep(1. / 120.)
	p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
	p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)


def load_sink():
	pos = list(map(add, sinkStartPos, [0, 0, 0.5]))
	sink_start_orient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
	sink_id = p.loadURDF("custom-data/sink/sink.urdf", sinkStartPos, sink_start_orient, useFixedBase=True)

	# Sphere over sink for debugging
	visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
	p.createMultiBody(
		baseMass=0,
		baseCollisionShapeIndex=-1,
		baseVisualShapeIndex=visual_shape_id,
		basePosition=pos,
		useMaximalCoordinates=True
	)

	return sink_id


def load_plates():
	plate_start_pos = [0, 0, 3]
	plate_start_orient = p.getQuaternionFromEuler([90, 0, 0])
	plate_id = p.loadURDF("data/dinnerware/plate/plate.urdf", plate_start_pos, plate_start_orient, globalScaling=1.5)

	plate_start_pos2 = [0.1, 0, 2.25]
	plate_id2 = p.loadURDF("data/dinnerware/plate/plate.urdf", plate_start_pos2, plate_start_orient, globalScaling=1.5)

	plate_start_pos3 = [0.2, 0, 2.5]
	plate_id3 = p.loadURDF("data/dinnerware/plate/plate.urdf", plate_start_pos3, plate_start_orient, globalScaling=1.5)

	pos_offset_plate = [0, 0, 1]
	mesh_scale_plate = [0.04, 0.04, 0.04]
	visual_shape_id_plate = p.createVisualShape(
		shapeType=p.GEOM_MESH,
		fileName="custom-data/plate/plate.obj",
		rgbaColor=[1, 0.5, 1, 1],
		specularColor=[0.4, .4, 0],
		visualFramePosition=pos_offset_plate,
		meshScale=mesh_scale_plate
	)
	collision_shape_id_plate = p.createCollisionShape(
		shapeType=p.GEOM_MESH,
		fileName="custom-data/plate/plate.obj",
		collisionFramePosition=pos_offset_plate,
		collisionFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
		meshScale=mesh_scale_plate
	)
	plate_id4 = p.createMultiBody(
		baseMass=1,
		baseInertialFramePosition=pos_offset_plate,
		baseVisualShapeIndex=visual_shape_id_plate,
		baseCollisionShapeIndex=collision_shape_id_plate,
		basePosition=pos_offset_plate,
		useMaximalCoordinates=True
	)
	plate_id5 = p.createMultiBody(
		baseMass=1,
		baseInertialFramePosition=pos_offset_plate,
		baseVisualShapeIndex=visual_shape_id_plate,
		baseCollisionShapeIndex=collision_shape_id_plate,
		basePosition=pos_offset_plate,
		useMaximalCoordinates=True
	)

	return [plate_id, plate_id2, plate_id3, plate_id4, plate_id5]


def load_cup():
	cup_start_pos = [0, 0, 1.5]
	cup_start_orient = p.getQuaternionFromEuler([0, 0, 0])
	cup_id = p.loadURDF("data/dinnerware/cup/cup_small.urdf", cup_start_pos, cup_start_orient, globalScaling=2)
	return cup_id


def load_mug():
	mug_start_pos = [0, 0, 0.5]
	mug_start_orient = p.getQuaternionFromEuler([0, 0, 0])
	mug_id = p.loadURDF("data/dinnerware/mug/mug.urdf", mug_start_pos, mug_start_orient, globalScaling=1.5)
	return mug_id


def load_dishwasher():
	dishwasher_start_pos = [2, 0, 0.5]
	dishwasher_start_orient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
	dishwasher_id = p.loadURDF(
		"custom-data/dishwasher/dishwasher.urdf",
		dishwasher_start_pos,
		dishwasher_start_orient,
		useFixedBase=True
	)
	return dishwasher_id


def load_arm():
	panda_start_orient = p.getQuaternionFromEuler([0, 0, np.pi])
	panda_id = p.loadURDF("franka_panda/panda.urdf", pandaStartPos, panda_start_orient, globalScaling=1.5, useFixedBase=1)
	return panda_id


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
	print(point_cloud)


if __name__ == '__main__':
	start_engine()

	# Load what we are using as the floor
	planeId = p.loadURDF("data/plane/plane100.urdf", useMaximalCoordinates=True)

	# Load sink on left, dishes above sink, dishwasher on right, and robotic arm between sink and dishwasher
	sinkId = load_sink()
	plateIds = load_plates()
	cupId =  load_cup()
	mugId =  load_mug()
	dishwasherId = load_dishwasher()
	pandaId = load_arm()

	obstacles = [sinkId, dishwasherId]

	# Set initial joint configuration for panda arm (in radians)
	# initial_joint_config = [np.pi/4., np.pi/4., 0, -np.pi/2., 0 ,  3*-np.pi/4.,  -np.pi/4., 0, 0, 0.04, 0.04]
	initial_joint_config = [np.pi / 4., 0, 0, 0, np.pi, 0, 0, 0, 0, 0, 0]
	# initial_joint_config = np.random.rand(11) * np.pi * 2

	# Testing arm movement
	curr_config = initial_joint_config

	jointIndices = len(initial_joint_config)
	p.setJointMotorControlArray(
		pandaId,
		range(jointIndices),
		p.POSITION_CONTROL,
		targetPositions=initial_joint_config)

	joint_info = []
	for x in range(11):
		joint_info.append(p.getJointInfo(pandaId, x))

	xId = p.addUserDebugParameter(paramName="x", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=-np.pi)
	yId = p.addUserDebugParameter(paramName="y", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)
	zId = p.addUserDebugParameter(paramName="z", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0.05)

	# final_config = [np.pi / 2., np.pi / 4., 0, np.pi / 2., 0, 0]

	# Camera setup
	width = 1280
	height = 1280
	fov = 60
	aspect = width / height
	near = 0.02
	far = 1

	view_matrix = panda_camera_view(pandaId)
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

	# p.stopStateLogging(logId)
	p.setGravity(0, 0, -9.81)
	p.setRealTimeSimulation(1)
	t = 0
	previous_orient = [-1, -1, -1]
	while 1:
		# needed for macOS
		p.stepSimulation()

		current_user_orient = [
			p.readUserDebugParameter(xId),
			p.readUserDebugParameter(yId),
			p.readUserDebugParameter(zId)
		]

		if previous_orient != current_user_orient:
			final_config = p.calculateInverseKinematics(
				bodyUniqueId=pandaId,
				endEffectorLinkIndex=8,
				targetPosition=list(map(add, sinkStartPos, [0, 0, 0.5])),
				targetOrientation=p.getQuaternionFromEuler(current_user_orient),
				# residualThreshold=0.01,
				# maxNumIterations=20000
			)[0:9]

		previous_orient = current_user_orient

		q1, q2, q3, q4, q5, q6, q7, q8, q9, = get_point_parameters(curr_config, final_config, t)
		p.setJointMotorControlArray(
			pandaId,
			range(jointIndices),
			p.POSITION_CONTROL,
			targetPositions=[q1, q2, q3, q4, q5, q6, q7, q8, q9, curr_config[9], curr_config[10]]
		)

		img = p.getCameraImage(
			width,
			height,
			panda_camera_view(pandaId),
			projection_matrix,
			renderer=p.ER_BULLET_HARDWARE_OPENGL
		)

		if t < 1:
			t += 0.01
		else:
			# print("Goal:")
			goal_pos = list(map(add, sinkStartPos, [0, 0, 0.5]))
			# print(goal_pos, p.getQuaternionFromEuler([np.pi, 0, 0]))
			# print("Actual:")
			ls = p.getLinkState(pandaId, 11, computeForwardKinematics=False)
			# print(ls[0], ls[1])

			diff = list(map(sub, ls[0], goal_pos))
			print("Dist to target: ", np.linalg.norm(diff, 2))

			if np.linalg.norm(diff, 2) < 0.1:
				get_point_cloud(far, height, img, near, width)

				collision_fn = get_collision_fn(
					pandaId,
					[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
					obstacles=obstacles,
					attachments=[],
					self_collisions=True,
					disabled_collisions=set()
				)
				print(collision_fn([q1, q2, q3, q4, q5, q6, q7, q8, q9]))
