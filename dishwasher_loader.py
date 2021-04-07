import pybullet as p
import pybullet_data as p_data
import time
import numpy as np
import matplotlib.pyplot as plt
from utils import *

# Connect to existing physics engine, if any, else create new physics engine
# with graphical front-end
connectionId = p.connect(p.SHARED_MEMORY)
if (connectionId < 0):
	p.connect(p.GUI)

p.setAdditionalSearchPath(p_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations = 10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "log.json")

# Load what we are using as the floor
planeId = p.loadURDF("data/plane/plane100.urdf", useMaximalCoordinates = True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

# Load sink
sinkStartPos = [-0.25, 0, 0.5]
sinkStartOrient = p.getQuaternionFromEuler([np.pi / 2,0,0])
sinkId = p.loadURDF("custom-data/sink/sink.urdf", sinkStartPos, sinkStartOrient, 
	useFixedBase = True)

# Load plates/bowls/cup above sink
plateStartPos = [0, 0, 3]
plateStartOrient = p.getQuaternionFromEuler([90, 0, 0])
plateId = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos, plateStartOrient, 
	globalScaling = 1.5)
plateStartPos2 = [0.1, 0, 2.25]
plateId2 = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos2, plateStartOrient, 
	globalScaling = 1.5)
plateStartPos3 = [0.2, 0, 2.5]
plateId2 = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos3, plateStartOrient, 
	globalScaling = 1.5)

posOffsetPlate = [0, 0, 1]
meshScalePlate = [0.04, 0.04, 0.04]
visualShapeIdPlate = p.createVisualShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/plate/plate.obj",
	rgbaColor = [1, 0.5, 1, 1],
	specularColor = [0.4, .4, 0],
	visualFramePosition = posOffsetPlate,
	meshScale = meshScalePlate
)
collisionShapeIdPlate = p.createCollisionShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/plate/plate.obj",
	collisionFramePosition = posOffsetPlate,
	collisionFrameOrientation = p.getQuaternionFromEuler([0,0,0]),
	meshScale = meshScalePlate
)
for i in range(2):
	p.createMultiBody(
		baseMass = 1,
		baseInertialFramePosition = posOffsetPlate,
		baseVisualShapeIndex = visualShapeIdPlate,
		baseCollisionShapeIndex = collisionShapeIdPlate,
		basePosition = posOffsetPlate,
		useMaximalCoordinates = True
	)

cupStartPos = [0, 0, 1.5]
cupStartOrient = p.getQuaternionFromEuler([0, 0, 0])
cupId = p.loadURDF("data/dinnerware/cup/cup_small.urdf", cupStartPos, cupStartOrient,
	globalScaling = 2)

mugStartPos = [0, 0, 0.5]
mugStartOrient = p.getQuaternionFromEuler([0, 0, 0])
mugId = p.loadURDF("data/dinnerware/mug/mug.urdf", mugStartPos, mugStartOrient, 
	globalScaling = 1.5)

# Load dishwasher to right of sink
dishwasherStartPos = [1.5, 0, 0.5]
dishwasherStartOrient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
dishwasherId = p.loadURDF("custom-data/dishwasher/dishwasher.urdf", dishwasherStartPos, 
	dishwasherStartOrient, useFixedBase = True)

# Load panda arm
pandaStartPos = [0.5, -0.75, 0]
pandaStartOrient = p.getQuaternionFromEuler([0, 0, 0])
pandaId = p.loadURDF("franka_panda/panda.urdf", pandaStartPos, pandaStartOrient,
	globalScaling = 1.5, useFixedBase = 1)

# Set initial joint configuration for panda arm
initial_joint_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.pi]
jointIndices = len(initial_joint_config)
p.setJointMotorControlArray(
pandaId, 
range(jointIndices),
p.POSITION_CONTROL, 
targetPositions = initial_joint_config # in radians
)

# Testing arm movement
curr_config = initial_joint_config
final_config = [np.pi / 2, np.pi / 4, 0, np.pi / 2, 0, 0]

# Camera setup
width = 128
height = 128
fov = 60
aspect = width / height
near = 0.02
far = 1

view_matrix = panda_camera_view(pandaId)
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

images = p.getCameraImage(
	width,
	height,
	view_matrix,
	projection_matrix,
	shadow = True,
	renderer = p.ER_BULLET_HARDWARE_OPENGL
)

# images = p.getCameraImage(
# 	width,
# 	height,
# 	view_matrix,
# 	projection_matrix,
# 	shadow=True,
# 	renderer=p.ER_TINY_RENDERER
# )

# p.stopStateLogging(logId)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

while 1:
	p.stepSimulation() # needed for macOS
	# images = p.getCameraImage(
	# 	width,
	# 	height,
	# 	view_matrix,
	# 	projection_matrix,
	# 	shadow=True,
	# 	renderer=p.ER_BULLET_HARDWARE_OPENGL
	# )
	t = 0
	while t<=1:
		q1, q2, q3, q4, q5, q6 = get_point_parameters(curr_config, final_config, t)
		p.setJointMotorControlArray(
			pandaId, 
			range(jointIndices), 
			p.POSITION_CONTROL, 
			targetPositions = [q1, q2, q3, q4, q5, q6, curr_config[6], curr_config[7], curr_config[8], curr_config[9], curr_config[10]])
		t += 0.00003  
		
		view_matrix = panda_camera_view(pandaId)
		images = p.getCameraImage(
			width,
			height,
			view_matrix,
			projection_matrix,
			shadow = True,
			renderer = p.ER_BULLET_HARDWARE_OPENGL
		) 
		time.sleep(0.1)
			
	time.sleep(0.01)

p.disconnect()