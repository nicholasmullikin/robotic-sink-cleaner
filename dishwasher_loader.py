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


if __name__ == '__main__':
	start_engine()

	# Load what we are using as the floor
	planeId = p.loadURDF("data/plane/plane100.urdf", useMaximalCoordinates=True)

	# Load sink on left, dishes above sink, dishwasher on right, and robotic arm between sink and dishwasher
	sinkId = load_sink()
	plateIds = load_plates()
	cupId = load_cup()
	mugId = load_mug()
	dishwasherId = load_dishwasher()
	pandaId = load_arm()

	obstacles = [sinkId, dishwasherId]

	# Set initial joint configuration for panda arm (in radians)
	initial_joint_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

	# Testing arm movement
	curr_config = initial_joint_config

	jointIndices = len(initial_joint_config)
	p.setJointMotorControlArray(
		pandaId,
		range(jointIndices),
		p.POSITION_CONTROL,
		targetPositions=initial_joint_config
	)

	joint_info = get_joint_info(pandaId)
	joint_lower_limits = joint_info[0]
	joint_upper_limits = joint_info[1]

	q1Id = p.addUserDebugParameter(paramName="q1", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=-1.587)
	q2Id = p.addUserDebugParameter(paramName="q2", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0.132)
	q3Id = p.addUserDebugParameter(paramName="q3", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=-0.066)
	q4Id = p.addUserDebugParameter(paramName="q4", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=-1.058)
	q5Id = p.addUserDebugParameter(paramName="q5", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=curr_config[4])
	q6Id = p.addUserDebugParameter(paramName="q6", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=1.058)
	q7Id = p.addUserDebugParameter(paramName="q7", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0.661)
	q8Id = p.addUserDebugParameter(paramName="q8", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=-0.066)
	q9Id = p.addUserDebugParameter(paramName="q9", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)
	q10Id = p.addUserDebugParameter(paramName="q10", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)
	q11Id = p.addUserDebugParameter(paramName="q11", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)

	# Camera setup
	width = 32
	height = 32
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

	collision_fn = get_collision_fn(
			pandaId,
			[0, 1, 2, 3, 4, 5, 6, 7, 8],
			obstacles=obstacles,
			attachments=[],
			self_collisions=True,
			disabled_collisions=set()
		)


	while 1:
		# needed for macOS
		p.stepSimulation()

		# final_config = [np.pi / 2., np.pi / 4., 0, np.pi / 2., 0, 0]

		# if previous_orient != current_user_orient:
		# 	final_config = p.calculateInverseKinematics(
		# 		bodyUniqueId=pandaId,
		# 		endEffectorLinkIndex=8,
		# 		targetPosition=np.array(sinkStartPos) + np.array([0, 0, 0.5]),
		# 		targetOrientation=p.getQuaternionFromEuler(desired_orient),
		# 		lowerLimits=joint_lower_limits,
		# 		upperLimits=joint_upper_limits
		# 		# residualThreshold=0.01,
		# 		# maxNumIterations=20000
		# 	)[0:9]
		#
		# previous_orient = current_user_orient

		target_pos = [
			p.readUserDebugParameter(q1Id),
			p.readUserDebugParameter(q2Id),
			p.readUserDebugParameter(q3Id),
			p.readUserDebugParameter(q4Id),
			p.readUserDebugParameter(q5Id),
			p.readUserDebugParameter(q6Id),
			p.readUserDebugParameter(q7Id),
			p.readUserDebugParameter(q8Id),
			p.readUserDebugParameter(q9Id),
			p.readUserDebugParameter(q10Id),
			p.readUserDebugParameter(q11Id),
		]

		final_config = [-1.587, 0.132, -0.066, -1.058, 0, 1.058, 0.661, -0.066, 0, 0, 0]
		q1, q2, q3, q4, q5, q6, q7, q8, q9, = get_point_parameters(curr_config, target_pos[:9], t)

		p.setJointMotorControlArray(
			pandaId,
			range(jointIndices),
			p.POSITION_CONTROL,
			targetPositions=[q1, q2, q3, q4, q5, q6, q7, q8, q9, 0, 0]
		)

		img = p.getCameraImage(
			width,
			height,
			panda_camera_view(pandaId),
			projection_matrix,
			renderer=p.ER_BULLET_HARDWARE_OPENGL
		)

		print("In collision: ", collision_fn([q1, q2, q3, q4, q5, q6, q7, q8, q9]))


		if t < 1:
			t += 0.03
		else:
			# print("Goal:")
			goal_pos = list(map(add, sinkStartPos, [0, 0, 0.5]))
			# print(goal_pos, p.getQuaternionFromEuler([np.pi, 0, 0]))
			# print("Actual:")
			ls = p.getLinkState(pandaId, 11, computeForwardKinematics=False)
			# print(ls[0], ls[1])

			diff = list(map(sub, ls[0], goal_pos))
			print("Dist to target: ", np.linalg.norm(diff, 2))

			if np.linalg.norm(diff, 2) < 0.05:
				val = get_pc(img, near, far, fov, pandaId)
				# get_point_cloud(img, width, height, far, near, end_effector_pos, target)
