# robotPose
### Installation
Install vtk, transforms3d
```Shell
pip install vtk (conda install -c conda-forge vtk)
pip install transforms3d
```
Install urdf_parser_py from https://github.com/ros/urdf_parser_py
```Shell
cd $urdf_parser_py
mkdir build
cd build
cmake ..
make
```
install pykdl from https://github.com/orocos/orocos_kinematics_dynamics
```Shell
cd $orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
cmake ..
make
cd $orocos_kinematics_dynamics/python_rocos_kdl
mkdir build
cd build
cmake ..
make
```
To generate a usable urdf seperate from ros, run (panda arm for instance):
```Shell
rosrun xacro xacro --inorder /home/liruiw/Projects/franka_ros/franka_description/robots/panda_arm.xacro > /home/liruiw/Projects/robotPose/panda_arm.urdf
```
To test the pose-joint conversions for baxter, run
```Shell
python robot_pykdl.py --robot=baxter
```
To view panda arm synthesized interative UI, run
```Shell
python robot_pykdl.py --robot=panda_arm
```