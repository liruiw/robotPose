# robotPose
### Installation
Install transforms3d, lxml 
Install pykdl (either in ros or from source).
```Shell
git clone https://github.com/orocos/orocos_kinematics_dynamics
```
make install $orocos_kinematics_dynamics/orocos_kdl and python_orocos_kdl

To generate a usable urdf seperate from ros, run (panda arm for instance):
```Shell
rosrun xacro xacro --inorder panda_arm.urdf.xacro > panda_arm.urdf
```
To test the pose-joint conversions for baxter or panda_arm, run
```Shell
python robot_pykdl.py --robot=baxter
```

To use Baxter, follow instructions to install baxter workspace from **http://sdk.rethinkrobotics.com/wiki/Workstation_Setup**

To run simulation, follow instructions to install baxter_simulator from  **http://sdk.rethinkrobotics.com/wiki/Simulator_Installation**

To control simulation, follow instructions to install Moveit from **http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial**  

After you start the simulation, you can control the robot arm using moveit in misc folder. run moveit with franka, follow the instructions here:
**http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html**
