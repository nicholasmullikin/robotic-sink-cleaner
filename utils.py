import numpy as np
import pybullet as p

def get_point_parameters(curr, final, t):
	inst = np.array(curr[:9]) + t*(np.array(final) - np.array(curr[:9]))
	return inst

def panda_camera_view(panda_id):
  endEffectorPos, endEffectorOrient, _, _, _, _ = p.getLinkState(panda_id, 11, 
  	computeForwardKinematics = False)
  rot_matrix = p.getMatrixFromQuaternion(endEffectorOrient)
  rot_matrix = np.array(rot_matrix).reshape(3, 3)
  
  init_camera_vector = (0, 0, 1) # z-axis
  init_up_vector = (0, 1, 0) # y-axis
  
  camera_vector = rot_matrix.dot(init_camera_vector)
  up_vector = rot_matrix.dot(init_up_vector)
  
  view_matrix = p.computeViewMatrix(
  	endEffectorPos, 
  	endEffectorPos + 0.1 * camera_vector, 
  	up_vector
	)

	
  return view_matrix