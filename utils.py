import numpy as np

def get_point_parameters(curr, final, t):
	inst = np.array(curr[:6]) + t*(np.array(final) - np.array(curr[:6]))
	return inst
