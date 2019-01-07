# robotPose
### Installation
Install vtk, transforms3d, lxml 
```Shell
pip install vtk (conda install -c conda-forge vtk)
pip install transforms3d
pip install lxml
```
urdf_parser_py is from https://github.com/ros/urdf_parser_py
Install pykdl
```Shell
git clone https://github.com/orocos/orocos_kinematics_dynamics
cd $orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
cmake ..
make
make install
cd $orocos_kinematics_dynamics/python_rocos_kdl
mkdir build
cd build
cmake ..
make
make install
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

To use Baxter, follow instructions to install baxter workspace from **http://sdk.rethinkrobotics.com/wiki/Workstation_Setup**

To run simulation, follow instructions to install baxter_simulator from  **http://sdk.rethinkrobotics.com/wiki/Simulator_Installation**

To control simulation, follow instructions to install Moveit from **http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial**  

After you start the simulation, test moving the robot by
```Shell
python move_arm.py --robot=baxter
```
To run moveit with franka, follow the instructions here:
**http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html**

After you start the simulation, test moving the robot by
```Shell
python move_arm.py --robot=panda_arm
```
