import numpy as np
import pybullet as p
import math


def get_point_parameters(curr, final, t):
    inst = np.array(curr[:9]) + t * (np.array(final) - np.array(curr[:9]))
    return inst


init_camera_vector = (0, 0, 1)  # z-axis
init_up_vector = (0, 1, 0)  # y-axis


def panda_camera_view(panda_id):
    end_effector_pos, end_effector_orient, _, _, _, _ = p.getLinkState(panda_id, 11, computeForwardKinematics=False)
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


def getRayFromTo(mouseX, mouseY, width, height, camTarget, camForward, horizonal, vertical, dist):
    # width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
    # )

    camPos = [
        camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
        camTarget[2] - dist * camForward[2]
    ]

    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[2])
    invLen = farPlane * 1. / lenFwd
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizonal[0] * oneOverWidth, horizonal[1] * oneOverWidth, horizonal[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [
        rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
    ]
    ortho = [
        -0.5 * horizonal[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
        -0.5 * horizonal[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
        -0.5 * horizonal[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
    ]
    rayTo = [
        rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
        rayFrom[2] + rayForward[2] + ortho[2]
    ]
    lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
    alpha = math.atan(lenOrtho / farPlane)

    return rayFrom, rayTo, alpha


def get_joint_info(armId):
    lower_limit = []
    upper_limit = []
    for x in range(11):
        one_joint_info = p.getJointInfo(armId, x)
        lower_limit.append(one_joint_info[8])
        upper_limit.append(one_joint_info[9])

    return [lower_limit, upper_limit]


def get_point_cloud(img, width, height, far, near, camPos, camTarget):
    depth_buffer = img[3]
    img_w = int(width / 10)
    img_h = int(height / 10)
    step_x = 5
    step_y = 5
    point_cloud = []
    for w in range(0, img_w, step_x):
        for h in range(0, img_h, step_y):
            ray_from, ray_to, alpha = getRayFromTo(w * (width / img_w), h * (height / img_h), width, height, camPos,
                                                   camTarget, )
            rf = np.array(ray_from)
            rt = np.array(ray_to)
            vec = rt - rf
            depth_img = float(depth_buffer[h, w])
            depth = far * near / (far - (far - near) * depth_img)
            depth /= math.cos(alpha)
            new_to = (depth / np.sqrt(np.dot(vec, vec))) * vec + rf
            point_cloud.append(new_to)

            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
            collisionShapeId = -1
            mb = p.createMultiBody(baseMass=0,
                                   baseCollisionShapeIndex=collisionShapeId,
                                   baseVisualShapeIndex=visualShapeId,
                                   basePosition=new_to,
                                   useMaximalCoordinates=True)

            color = img[2][h, w]
            color = [color[0] / 255., color[1] / 255., color[2] / 255., 1]
            p.changeVisualShape(mb, -1, rgbaColor=color)


# print(point_cloud)


def get_pc(img, near, far, fov, panda_id):
    depth = img[3]
    distances = from_opengl_depth_to_distance(
        depth=np.array(depth), near=near, far=far)

    point_cloud = distance_map_to_point_cloud(
        distances=distances, fov = fov / 180.0 * np.pi, width=distances.shape[1],
        height = distances.shape[0])

    # plot_pc(point_cloud, [1, 1, 1, 1])


    # point_cloud = transform_point_cloud_from_camera_to_world_frame(
    #     point_cloud, panda_id)

    plot_pc(point_cloud, [0, 0, 1, 1])
    return point_cloud


def plot_pc(pc, color):
    for i in range(pc.shape[0]):
        for j in range(pc.shape[1]):
            new_to = pc[i, j, :]
            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=color, radius=0.009)
            collisionShapeId = -1
            p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=collisionShapeId,
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=new_to,
                              useMaximalCoordinates=True)


def transform_point_cloud_from_camera_to_world_frame(point_cloud, panda_id):
    """Project a sample in the depth image to a 3D point in the world frame.
    Args:
      point_cloud: A numpy array of shape (height, width, 3) that represents
        a point cloud in the camera frame.
    Returns:
      The transformed point cloud in the world frame.
    """

    end_effector_pos, end_effector_orient, _, _, _, _ = p.getLinkState(panda_id, 11, computeForwardKinematics=False)
    rot_matrix = p.getMatrixFromQuaternion(end_effector_orient)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)

    camera_link_pos = end_effector_pos
    # camera_link_ori = p.getQuaternionFromEuler(camera_vector)
    camera_link_ori = p.getQuaternionFromEuler([-camera_vector[0], -camera_vector[1], -camera_vector[2]])
    # if self._parent_link_id == -1:
    # 	camera_link_pos, camera_link_ori = (
    # 		self._pybullet_client.getBasePositionAndOrientation(self._body_id))
    # else:
    # 	parent_link_state = p.getLinkState(
    # 		self._body_id, self._parent_link_id, computeForwardKinematics=True)
    # 	camera_link_pos = parent_link_state[0]
    # 	camera_link_ori = parent_link_state[1]

    # relative_translation = end_effector_pos
    # relative_rotation = camera_link_ori
    # camera_position_world, camera_orientation_world = (
    #     p.multiplyTransforms(camera_link_pos,
    #                          camera_link_ori,
    #                          relative_translation,
    #                          relative_rotation))

    # camera_position_world = [-camera_link_pos[0], -camera_link_pos[1], -camera_link_pos[2],]
    camera_position_world = [0, 0, 0]
    camera_orientation_world = camera_link_ori

    _CAMERA_Z_TO_WORLD_Z = p.getQuaternionFromEuler([0, 0, 0])
    #_CAMERA_Z_TO_WORLD_Z = p.getQuaternionFromEuler([-up_vector[0], -up_vector[1], -up_vector[2]])
    # camera_space_to_world_orientation = [-camera_link_ori[0], -camera_link_ori[1],
    #                                      -camera_link_ori[2], -camera_link_ori[3]]



    _, camera_space_to_world_orientation = (
        p.multiplyTransforms([0, 0, 0],
                             camera_link_ori,
                             [0, 0, 0],
                             _CAMERA_Z_TO_WORLD_Z))



    for i in range(point_cloud.shape[0]):
        for j in range(point_cloud.shape[1]):
            point_cloud[i, j, :] = (
                p.multiplyTransforms(
                    camera_position_world, camera_space_to_world_orientation,
                    point_cloud[i, j, :], [0, 0, 0, 1])[0])
    return point_cloud


def from_opengl_depth_to_distance(depth, near, far):
    return far * near / (far - (far - near) * depth)


def distance_map_to_point_cloud(distances, fov, width, height):
    """Converts from a depth map to a point cloud.
  Args:
    distances: An numpy array which has the shape of (height, width) that
      denotes a distance map. The unit is meter.
    fov: The field of view of the camera in the vertical direction. The unit
      is radian.
    width: The width of the image resolution of the camera.
    height: The height of the image resolution of the camera.
  Returns:
    point_cloud: The converted point cloud from the distance map. It is a numpy
      array of shape (height, width, 3).
  """
    f = height / (2 * math.tan(fov / 2.0))
    px = np.tile(np.arange(width), [height, 1])
    x = (2 * (px + 0.5) - width) / f * distances / 2
    py = np.tile(np.arange(height), [width, 1]).T
    y = (2 * (py + 0.5) - height) / f * distances / 2
    point_cloud = np.stack((x, y, distances), axis=-1)
    return point_cloud
