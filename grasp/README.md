# robotPose Grasp
### Installation

To use graspit for panda hand, please install **https://graspit-simulator.github.io/build/html/installation_linux.html**. Then install third party library **https://github.com/JenniferBuehler/graspit-pkgs**. Place the grasp folder inside the src/ and do catkin_make. You would need to resolve all files and path. (Take a look at autocopy and generate wrl files by scale_wrl.py). 

Online planning or generate new grasps (require MATIO for non-ros) by roslaunch graspit_pose graspit_pose.launch. Take a look at its parameters. 

For visualization after generating the grasps. Create the softlinks for ycb_renderer, YCB models, panda arm models, YCB_Video and output.

Mainly, test the communication between planner and pose detection by 
```Shell
python vis_grasp_pose_ros.py 0040 5
roslaunch graspit_pose graspit_pose_ros.launch
```
Note: make sure you make install graspit again since we modify its source code.