import pybullet as p
import time
import math
import pybullet_data



cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)


#cubeStartPos = [0, 0, 1]
#cubeStartOrientation = p.getQuaternionFromEuler([90, 0, 0])
#boxId = p.loadURDF("dinnerware/plate.urdf", cubeStartPos, cubeStartOrientation)

shiftPlate = [0, 0.02, 0]
meshScalePlate = [.02, .02, .02]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="plate.obj",
                                    rgbaColor=[1, 0.5, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shiftPlate,
                                    meshScale=meshScalePlate)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="custom-data/plate.obj",
                                          collisionFramePosition=shiftPlate,
                                          meshScale=meshScalePlate)
										  
shiftSink = [0, 0.02, 0]
meshScaleSink = [1, 1, 1]									  
visualShapeId1 = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="custom-data/sink.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shiftSink,
									visualFrameOrientation=p.getQuaternionFromEuler([90,0,0]),
                                    meshScale=meshScaleSink,
									)
collisionShapeId1 = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="custom-data/sink.obj",
                                          collisionFramePosition=shiftSink,
										  collisionFrameOrientation=p.getQuaternionFromEuler([90,0,0]),
                                          meshScale=meshScaleSink)
										  
	
shiftDishwasher = [0, 0, 0]
meshScaleDishwasher = [.2, .2, .2]	
visualDishwasher = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="custom-data/dishwasher/dishwasher.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shiftDishwasher,
									visualFrameOrientation=p.getQuaternionFromEuler([90,0,0]),
                                    meshScale=meshScaleDishwasher)
collisionDishwasher = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="custom-data/dishwasher/dishwasher.obj",
                                          collisionFramePosition=shiftDishwasher,
										  collisionFrameOrientation=p.getQuaternionFromEuler([90,0,0]),
                                          meshScale=meshScaleDishwasher)	
p.createMultiBody(baseMass=100,
				  baseInertialFramePosition=[0, 0, 0],
				  baseCollisionShapeIndex=collisionDishwasher,
				  baseVisualShapeIndex=visualDishwasher,
				  basePosition=[0, 0, 0.3],
				  useMaximalCoordinates=True)





p.createMultiBody(baseMass=100,
				  baseInertialFramePosition=[0, 0, 0],
				  baseCollisionShapeIndex=collisionShapeId1,
				  baseVisualShapeIndex=visualShapeId1,
				  basePosition=[-1, 0, 0.3],
				  useMaximalCoordinates=True)





rangex = 2
rangey = 2
for i in range(rangex):
  for j in range(rangey):
    p.createMultiBody(baseMass=20,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[-1+i/4,
                                    0+j/4, 2],
                      useMaximalCoordinates=True)
					  
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
currentColor = 0

while (1):
  time.sleep(1./240.)