import argparse
import pybullet as p
import pybullet_data as p_data

if __name__ == '__main__':

	argsParser = argparse.ArgumentParser()
	argsParser.add_argument(
		'-f', 
		'--file', 
		dest = 'fileName', 
		type = str,
		required = True,
		help = 'File name of .obj file for which the V-HACD will be calculated.'
	)
	argsParser.add_argument(
		'-O', 
		'--output',
		dest = 'output',
		type = str,
		required = True,
		help = 'Desired name of V-HACD file.'
	)
	argsParser.add_argument(
		'-r',
		'--resolution',
		dest = 'resolution',
		type = int, 
		required = False,
		help = 'Maximum number of voxels generated during voxelization stage.'
	)
	args = argsParser.parse_args()

	p.connect(p.DIRECT)
	name_in = args.fileName
	name_out = args.output
	name_log = "vhacd_log.txt"

	if (args.resolution != None):
		res = args.resolution
	else:
		res = 100_000

	p.vhacd(
		name_in, 
		name_out, 
		name_log, 
		resolution = res
	)
