import pybullet as p
import pybullet_data as p_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from operator import add, sub
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
    pos = list(map(add, sinkStartPos, [0, 0, 0.5]))
    # sphere over sink
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
    collisionShapeId = -1
    mb = p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=collisionShapeId,
                           baseVisualShapeIndex=visualShapeId,
                           basePosition=pos,
                           useMaximalCoordinates=True)


    sinkStartOrient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
    return p.loadURDF("custom-data/sink/sink.urdf", sinkStartPos, sinkStartOrient,
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
    plateId3 = p.loadURDF("data/dinnerware/plate/plate.urdf", plateStartPos3, plateStartOrient,
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
    return [plateId, plateId2, plateId3]


def load_cup():
    cupStartPos = [0, 0, 1.5]
    cupStartOrient = p.getQuaternionFromEuler([0, 0, 0])
    return p.loadURDF("data/dinnerware/cup/cup_small.urdf", cupStartPos, cupStartOrient,
                       globalScaling=2)


def load_mug():
    mugStartPos = [0, 0, 0.5]
    mugStartOrient = p.getQuaternionFromEuler([0, 0, 0])
    return p.loadURDF("data/dinnerware/mug/mug.urdf", mugStartPos, mugStartOrient,
                       globalScaling=1.5)


def load_dishwasher():
    dishwasherStartPos = [2, 0, 0.5]
    dishwasherStartOrient = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
    return p.loadURDF("custom-data/dishwasher/dishwasher.urdf", dishwasherStartPos,
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
    sinkId = load_sink()
    plateIds = load_plates()
    # Load plates/bowls/cup above sink
    cupId =  load_cup()
    mugId =  load_mug()
    # Load dishwasher to right of sink
    dishwasherId = load_dishwasher()
    # Load panda arm
    pandaId = load_arm()

    obstacles = [sinkId, dishwasherId]

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

        p.stepSimulation()  # needed for macOS

        current_user_orient = [p.readUserDebugParameter(xId),
                               p.readUserDebugParameter(yId),
                               p.readUserDebugParameter(zId)]

        if previous_orient != current_user_orient:
            final_config = p.calculateInverseKinematics(bodyUniqueId=pandaId,
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
            targetPositions=[q1, q2, q3, q4, q5, q6, q7, q8, q9, curr_config[9], curr_config[10]])

        img = p.getCameraImage(
            width,
            height,
            panda_camera_view(pandaId),
            projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        print([p.getBodyUniqueId(i)
            for i in range(p.getNumBodies())])


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


        from collisionUtils import get_collision_fn
        collision_fn = get_collision_fn(pandaId, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], obstacles=obstacles,
                                        attachments=[], self_collisions=True,
                                        disabled_collisions=set())

        print(collision_fn([q1, q2, q3, q4, q5, q6, q7, q8, q9]))




    # p.disconnect()


def get_point_cloud(far, height, img, near, width):
    depthBuffer = img[3]
    imgW = int(width / 10)
    imgH = int(height / 10)
    stepX = 5
    stepY = 5
    point_cloud = []
    for w in range(0, imgW, stepX):
        for h in range(0, imgH, stepY):
            rayFrom, rayTo, alpha = getRayFromTo(w * (width / imgW), h * (height / imgH))
            rf = np.array(rayFrom)
            rt = np.array(rayTo)
            vec = rt - rf
            depthImg = float(depthBuffer[h, w])
            depth = far * near / (far - (far - near) * depthImg)
            depth /= math.cos(alpha)
            newTo = (depth / np.sqrt(np.dot(vec, vec))) * vec + rf
            point_cloud.append(newTo)
    print(point_cloud)


if __name__ == '__main__':
    main()
