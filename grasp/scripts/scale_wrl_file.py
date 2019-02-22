import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import re
import sys
from shutil import copyfile
import os
def mkdir_if_missing(dst_dir):
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

holder = []
scale_flag = False
#obj = [name for name in os.listdir('models') if name.endswith(model_path)][0]
mkdir_if_missing('mesh')
for obj in os.listdir('models'):
    model_path = '_'.join(obj.split('_')[1:])
    print 'model path', obj, model_path
    os.system('meshlabserver -i models/{}/textured_simple.obj -o mesh/{}.wrl'.format(obj, model_path))
    with open("mesh/{}.wrl".format(model_path), "rb") as vrml,\
     open("mesh/{}_new.wrl".format(model_path), "w+") as vrml2:
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

    holder_array = np.array(holder) 
    print holder_array.shape
    with open("mesh/{}.xml".format(model_path), "w+") as xml:
        xml.write('<?xml version="1.0" ?>\n')
        xml.write('<root>\n')
        xml.write('\t<material>plastic</material>\n')
        xml.write('\t<mass>300</mass>\n')
        xml.write('\t<cog>0 0 0</cog>\n')
        xml.write('\t<geometryFile type="Inventor">{}_new.wrl</geometryFile>\n'.format(model_path))
        xml.write('</root>\n')

    #hard coded path
    os.system('cp mesh/{}.xml $GRASPIT/models/objects/{}.xml'.format(model_path, model_path))
    os.system('cp mesh/{}_new.wrl $GRASPIT/models/objects/{}_new.wrl'.format(model_path, model_path))