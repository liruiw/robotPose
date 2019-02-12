import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import re
import sys
from shutil import copyfile
holder = []
scale_flag = False
model_path = sys.argv[1]
with open("mesh/{}.wrl".format(model_path), "rb") as vrml, open("mesh/{}_new.wrl".format(model_path), "w+") as vrml2:
	for lines in vrml:
		line = lines.strip()
		line = line.split()
		if scale_flag and len(line) == 12:
			line = [val.rstrip(',') for val in line]
			vrml2.write('\t\t\t\t\t\t')
			arr = [float(val) * 1000 for val in line]
			holder.append(arr[:3]);holder.append(arr[3:6]);holder.append(arr[6:])
			text = str(arr) #scale
			vrml2.write(text.replace("[", "").replace("]", ","))
			vrml2.write("\n")
		else: 
			if lines.strip().startswith('point'):
				scale_flag = True
			if lines.strip().startswith(']'):
				scale_flag = False
			vrml2.write(lines)

holder_array = np.array(holder) #if you want numpy array
print holder_array.shape

#3D Plotting
# x,y,z = zip(*holder)
# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot(x,y,z)
# plt.show()
with open("mesh/{}.xml".format(model_path), "w+") as xml:
	xml.write('<?xml version="1.0" ?>\n')
	xml.write('<root>\n')
	xml.write('\t<material>plastic</material>\n')
	xml.write('\t<mass>300</mass>\n')
	xml.write('\t<cog>0 0 0</cog>\n')
	xml.write('\t<geometryFile type="Inventor">{}_new.wrl</geometryFile>\n'.format(model_path))
	xml.write('</root>\n')

#hard coded path
copyfile("mesh/{}.xml".format(model_path), '/home/liruiw/Projects/graspit/models/objects/{}.xml'.format(model_path))
copyfile("mesh/{}_new.wrl".format(model_path), '/home/liruiw/Projects/graspit/models/objects/{}_new.wrl'.format(model_path))