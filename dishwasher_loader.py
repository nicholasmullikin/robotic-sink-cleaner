import pybullet as p
import pybullet_data as p_data
import time
import numpy as np
import matplotlib.pyplot as plt
from operator import add
from utils import *

sinkStartPos = [-0.25, 0, 0.5]
pandaStartPos = [-0.25, -0.75, 0]


def start_engine():
    # Connect to existing physics engine, if any, else create new physics engine
    # with graphical front-end
    connectionId = p.connect(p.SHARED_MEMORY)
    if connectionId < 0:
        p.connect(p.GUI)

    p.setAdditionalSearchPath(p_data.getDataPath())
    p.setPhysicsEngineParameter(numSolverIterations=10)
    p.setTimeStep(1. / 120.)
    logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)


def load_sink():
    sinkStartOrient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
    sinkId = p.loadURDF("custom-data/sink/sink.urdf", sinkStartPos, sinkStartOrient,
                        useFixedBase=True)


def load_plates():
    plateStartPos = [0, 0, 3]
    plateStartOrient = p.getQuaternionFromEuler([90, 0, 0])
    plateId = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos, plateStartOrient,
                         globalScaling=1.5)
    plateStartPos2 = [0.1, 0, 2.25]
    plateId2 = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos2, plateStartOrient,
                          globalScaling=1.5)
    plateStartPos3 = [0.2, 0, 2.5]
    plateId2 = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos3, plateStartOrient,
                          globalScaling=1.5)

    posOffsetPlate = [0, 0, 1]
    meshScalePlate = [0.04, 0.04, 0.04]
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
        collisionFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        meshScale=meshScalePlate
    )
    for i in range(2):
        p.createMultiBody(
            baseMass=1,
            baseInertialFramePosition=posOffsetPlate,
            baseVisualShapeIndex=visualShapeIdPlate,
            baseCollisionShapeIndex=collisionShapeIdPlate,
            basePosition=posOffsetPlate,
            useMaximalCoordinates=True
        )


def load_cup():
    cupStartPos = [0, 0, 1.5]
    cupStartOrient = p.getQuaternionFromEuler([0, 0, 0])
    cupId = p.loadURDF("data/dinnerware/cup/cup_small.urdf", cupStartPos, cupStartOrient,
                       globalScaling=2)


def load_mug():
    mugStartPos = [0, 0, 0.5]
    mugStartOrient = p.getQuaternionFromEuler([0, 0, 0])
    mugId = p.loadURDF("data/dinnerware/mug/mug.urdf", mugStartPos, mugStartOrient,
                       globalScaling=1.5)


def load_dishwasher():
    dishwasherStartPos = [2, 0, 0.5]
    dishwasherStartOrient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
    dishwasherId = p.loadURDF("custom-data/dishwasher/dishwasher.urdf", dishwasherStartPos,
                              dishwasherStartOrient, useFixedBase=True)


def load_arm():
    pandaStartOrient = p.getQuaternionFromEuler([0, 0, np.pi])
    pandaId = p.loadURDF("franka_panda/panda.urdf", pandaStartPos, pandaStartOrient,
                         globalScaling=1.5, useFixedBase=1)
    return pandaId


def main():
    start_engine()

    # Load what we are using as the floor
    planeId = p.loadURDF("data/plane/plane100.urdf", useMaximalCoordinates=True)

    # Load sink
    load_sink()
    load_plates()
    # Load plates/bowls/cup above sink
    load_cup()
    load_mug()
    # Load dishwasher to right of sink
    load_dishwasher()
    # Load panda arm
    pandaId = load_arm()

    # Set initial joint configuration for panda arm
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
        targetPositions=initial_joint_config)  # in radians

    xId = p.addUserDebugParameter(paramName="x", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=np.pi)
    yId = p.addUserDebugParameter(paramName="y", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)
    zId = p.addUserDebugParameter(paramName="z", rangeMin=-np.pi * 2, rangeMax=np.pi * 2, startValue=0)

    # final_config = [np.pi / 2., np.pi / 4., 0, np.pi / 2., 0, 0]

    # Camera setup
    width = 128
    height = 128
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

        p.stepSimulation()  # needed for macOS

        current_user_orient = [p.readUserDebugParameter(xId),
                               p.readUserDebugParameter(yId),
                               p.readUserDebugParameter(zId)]

        if previous_orient != current_user_orient:
            final_config = p.calculateInverseKinematics(bodyUniqueId=pandaId,
                                                        endEffectorLinkIndex=8,
                                                        targetPosition=list(map(add, sinkStartPos, [0, 0, 1])),
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
            targetPositions=[q1, q2, q3, q4, q5, q6, q7, q8, q9, curr_config[9], curr_config[10]])

        if t < 1:
            t += 0.01
        else:
            print("Goal:")
            print(list(map(add, sinkStartPos, [0, 0, 1])), p.getQuaternionFromEuler([np.pi, 0, 0]))
            print("Actual:")
            ls = p.getLinkState(pandaId, 11, computeForwardKinematics=False)
            print(ls[0], ls[1])
            # break

        p.getCameraImage(
            width,
            height,
            panda_camera_view(pandaId),
            projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

    # p.disconnect()


if __name__ == '__main__':
    main()
