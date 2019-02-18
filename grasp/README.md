# robotPose Grasp
### Installation

To use graspit for panda hand, please install **https://graspit-simulator.github.io/build/html/installation_linux.html**. Then install third party library **https://github.com/JenniferBuehler/graspit-pkgs**. Place the grasp folder inside the src/ and do catkin_make. You would need to resolve all files and path. (models, worlds and panda robots are provided). 

Online planning or generate new grasps (require MATIO for non-ros) by roslaunch graspit_pose graspit_pose.launch. Take a look at its parameters. 

For visualization after generating the grasps. Create the softlinks for ycb_renderer, YCB models, panda arm models, YCB_Video and output.

scale_wrl_file.py can be used to convert meshlab file and scale the mesh (e.g python scale_wrl_file.py cracker_box).  

vis_grasp_pose.py can visualize one grasp (e.g python vis_grasp_pose.py cracker_box).

vis_grasp_pose_all.py can visualize a grasp in a YCB configuration. (e.g python vis_grasp_pose.py 0040 5)

Mainly, test the communication between planner and pose detection by 
```Shell
python vis_grasp_pose_ros.py 0040 5
roslaunch graspit_pose graspit_pose_ros.launch
```