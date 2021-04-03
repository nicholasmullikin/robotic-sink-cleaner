import pybullet as p
import pybullet_data as p_data
import time
import numpy as n
import matplotlib.pyplot as plt

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

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

# Load sink
posOffsetSink = [1, 0, 0.25]
meshScaleSink = [1.5, 1.5, 1.5]
visualShapeIdSink = p.createVisualShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/sink/sink.obj",
	rgbaColor = [1, 1, 1, 1],
	specularColor = [0.4, .4, 0],
	visualFramePosition = posOffsetSink,
	visualFrameOrientation = p.getQuaternionFromEuler([90,0,0]),
	meshScale = meshScaleSink
)
collisionShapeIdSink = p.createCollisionShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/sink/sink_vhacd.obj",
	collisionFramePosition = posOffsetSink,
	collisionFrameOrientation = p.getQuaternionFromEuler([90,0,0]),
	meshScale = meshScaleSink
)
p.createMultiBody(
	baseMass = 100,
	baseInertialFramePosition = posOffsetSink,
	baseVisualShapeIndex = visualShapeIdSink,
	baseCollisionShapeIndex = collisionShapeIdSink,
	basePosition = [-1, 0, 0.3],
	useMaximalCoordinates = True
)

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

mugStartPos = [0, 0, 1.5]
mugStartOrient = p.getQuaternionFromEuler([0, 0, 0])
mugId = p.loadURDF("data/dinnerware/mug/mug.urdf", mugStartPos, mugStartOrient, 
	globalScaling = 1.5)

# Load dishwasher to right of sink
posOffsetDishwasher = [1.5, 0, 0.3]
meshScaleDishwasher = [0.2, 0.2, 0.2]
visualShapeIdDishwasher = p.createVisualShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/dishwasher/dishwasher.obj",
	rgbaColor = [1, 1, 1, 1],
	specularColor = [0.4, .4, 0],
	visualFramePosition = posOffsetDishwasher,
	visualFrameOrientation = p.getQuaternionFromEuler([90,0,0]),
	meshScale = meshScaleDishwasher
)
collisionShapeIdDishwasher = p.createCollisionShape(
	shapeType = p.GEOM_MESH,
	fileName = "custom-data/dishwasher/dishwasher_vhacd.obj",
	collisionFramePosition = posOffsetDishwasher,
	collisionFrameOrientation = p.getQuaternionFromEuler([90,0,0]),
	meshScale = meshScaleDishwasher
)
p.createMultiBody(
	baseMass=1000,
	baseInertialFramePosition=posOffsetDishwasher,
	baseCollisionShapeIndex=collisionShapeIdDishwasher,
	baseVisualShapeIndex=visualShapeIdDishwasher,
	basePosition=[0, 0, 0.3],
	useMaximalCoordinates=True
)

# Load panda arm
pandaStartPos = [0.75, -0.75, 0]
pandaStartOrient = p.getQuaternionFromEuler([0, 0, 0])
pandaId = p.loadURDF("franka_panda/panda.urdf", pandaStartPos, pandaStartOrient,
	globalScaling = 1)

# Set initial joint configuration for panda arm
initial_joint_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
p.setJointMotorControlArray(
pandaId, 
range(len(initial_joint_config)),
p.POSITION_CONTROL, 
targetPositions = initial_joint_config
)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while 1:
	p.stepSimulation() # needed for macOS
	time.sleep(0.01)

p.disconnect()