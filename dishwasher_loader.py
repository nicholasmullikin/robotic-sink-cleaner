import pybullet as p
import pybullet_data as p_data
import time

# Connect to existing physics engine, if any, else create new physics engine
# with graphical front-end
connectionId = p.connect(p.SHARED_MEMORY)
if (connectionId < 0):
	p.connect(p.GUI)

p.setPhysicsEngineParameter(numSolverIterations = 10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "log.json")

# Load what we are using as the floor
planeId = p.loadURDF("data/plane/plane100.urdf", useMaximalCoordinates = True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

# Load sink
posOffsetSink = [0, 0, 0]
meshScaleSink = [1, 1, 1]
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
	baseInertialFramePosition = [0, 0, 0],
	baseVisualShapeIndex = visualShapeIdSink,
	baseCollisionShapeIndex = collisionShapeIdSink,
	basePosition = [-1, 0, 0.3],
	useMaximalCoordinates = True
)

# Load plates above sink
plateStartPos = [0, 0, 0.5]
plateStartOrient = p.getQuaternionFromEuler([90, 0, 0])
plateId = p.loadURDF("data/dinnerware/plate.urdf", plateStartPos, plateStartOrient)

posOffsetPlate = [0, 0, 0]
meshScalePlate = [0.02, 0.02, 0.02]
visualShapeIdPlate = p.createVisualShape(
	shapeType=p.GEOM_MESH,
	fileName="custom-data/plate/plate.obj",
	rgbaColor=[1, 0.5, 1, 1],
	specularColor=[0.4, .4, 0],
	visualFramePosition=posOffsetPlate,
	meshScale=meshScalePlate
)
collisionShapeIdPlate = p.createCollisionShape(
	shapeType=p.GEOM_MESH,
	fileName="custom-data/plate/plate.obj",
	collisionFramePosition=posOffsetPlate,
	meshScale=meshScalePlate
)

rangeI = 2
rangeJ = 2
for i in range(rangeI):
	for j in range(rangeJ):
		p.createMultiBody(
			baseMass = 5,
			baseInertialFramePosition = [0, 0, 0],
			baseVisualShapeIndex = visualShapeIdPlate,
			baseCollisionShapeIndex = collisionShapeIdPlate,
			basePosition = posOffsetPlate + [i / 4., j / 4., 0],
			useMaximalCoordinates = True
		)

# Load dishwasher to right of sink
posOffsetDishwasher = [0, 0, 0]
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
	baseMass=100,
	baseInertialFramePosition=[0, 0, 0],
	baseCollisionShapeIndex=collisionShapeIdDishwasher,
	baseVisualShapeIndex=visualShapeIdDishwasher,
	basePosition=[0, 0, 0.3],
	useMaximalCoordinates=True
)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

while 1:
	p.stepSimulation() # needed for macOS
	time.sleep(0.01)