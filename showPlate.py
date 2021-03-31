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

shiftPlate = [0, 0.02, 0]
meshScalePlate = [.05, .05, .05]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="plate.obj",
                                    rgbaColor=[1, 0.5, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shiftPlate,
                                    meshScale=meshScalePlate)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="plate.obj",
                                          collisionFramePosition=shiftPlate,
                                          meshScale=meshScalePlate)
										  
shiftSink = [0, 0.02, 0]
meshScaleSink = [1, 1, 1]									  
visualShapeId1 = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="sink.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shiftSink,
                                    meshScale=meshScaleSink)
collisionShapeId1 = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="sink.obj",
                                          collisionFramePosition=shiftSink,
                                          meshScale=meshScaleSink)
										  
								  

rangex = 1
rangey = 1
for i in range(rangex):
  for j in range(rangey):
    p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId1,
                      baseVisualShapeIndex=visualShapeId1,
                      basePosition=[((-rangex / 2) + i) * meshScaleSink[0] * 2,
                                    (-rangey / 2 + j) * meshScaleSink[1] * 2, 1],
                      useMaximalCoordinates=True)





rangex = 2
rangey = 2
for i in range(rangex):
  for j in range(rangey):
    p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[((-rangex / 2) + i) * meshScalePlate[0] * 2,
                                    (-rangey / 2 + j) * meshScalePlate[1] * 2, 1],
                      useMaximalCoordinates=True)
					  
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
currentColor = 0

while (1):
  time.sleep(1./240.)