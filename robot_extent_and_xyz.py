import cv2
import numpy as np
from pyassimp import *
import sys
robot_name = sys.argv[1]

if robot_name == 'panda_arm':
	models = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'link8', 'hand', 'finger', 'finger', 'camera']
elif robot_name == 'baxter':
	models = ['dummy1', 'dummy2','S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2']
obj_paths = [
	'../ycb_render/robotPose/{}_models/{}.DAE'.format(robot_name,item) for item in models]
model_filename = '../ycb_render/robotPose/{}_models/models.txt'.format(robot_name)
extent_filename = '../ycb_render/robotPose/{}_models/extents.txt'.format(robot_name)
offset_filename = '../ycb_render/robotPose/{}_models/center_offset.txt'.format(robot_name)

def homotrans(M, p):
    p = np.asarray(p)
    if p.shape[-1] == M.shape[1] - 1:
        p = np.append(p, np.ones_like(p[..., :1]), -1)
    p = np.dot(p, M.T)
    return p[..., :-1] / p[..., -1:]

def recursive_load(node, vertices):
    if node.meshes:
        transform = node.transformation 
        for idx,mesh in enumerate(node.meshes):
            mesh_vertex = homotrans(transform,mesh.vertices)
            vertices.append(mesh_vertex)
    for child in node.children:
        recursive_load(child, vertices)
    return vertices

for i, path in enumerate(obj_paths):
	if not os.path.exists(path):
		f = open(offset_filename, "a+")#dummy for empty link
		f.write('%f %f %f\n'%(0,0,0))
		f = open(model_filename, "a+")
		f.write("{}\n".format(path[3:]))
		continue
	scene = load(path)
	pts =  np.zeros(3000, dtype=np.float32)
	vertices = recursive_load(scene.rootnode, [])
	vertices = np.concatenate(vertices,axis=0)
	#extent = 2*np.max(np.absolute(vertices),axis=0) #rect max-min
	extent = np.max(vertices,axis=0) - np.min(vertices,axis=0)
	center = (np.max(vertices,axis=0) + np.min(vertices,axis=0))/2
	idx = np.random.choice(np.arange(vertices.shape[0]),min(3000,vertices.shape[0]) , replace=False)
	xyz = vertices[idx] - center #subtract the offset
	print 'range', np.max(vertices,axis=0),np.min(vertices,axis=0)
	print xyz.shape
	print extent
	f = open(extent_filename, "a+") #continue writing
	f.write('%f %f %f\n'%(extent[0],extent[1],extent[2]))
	f = open(offset_filename, "a+")
	f.write('%f %f %f\n'%(center[0],center[1],center[2]))
	f = open(model_filename, "a+")
	f.write("{}\n".format(path[3:]))
	xyz_file = '../ycb_render/robotPose/{}_models/{}.xyz'.format(robot_name,models[i])
	np.savetxt(xyz_file, xyz, fmt="%f")
