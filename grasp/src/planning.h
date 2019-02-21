#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/GraspItHelpers.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <random>
#include <math.h>
#include <graspit/world.h>
#include <graspit/robot.h>
#include <graspit/body.h>
#include <graspit/grasp.h>
#include <graspit/graspitCore.h>
#include <matio.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
using GraspIt::GraspItSceneManager;
float dgauss(float mean, float stdDev);
Eigen::Quaterniond euler2quat(float roll, float pitch, float yaw);
void rotationCase(Eigen::Quaterniond quat, int target);
int getRotationCase(Eigen::Quaterniond quat, int target);
const std::vector<std::string> YCB_classes = { "background", "master_chef_can", "cracker_box", "sugar_box", "tomato_soup_can", "mustard_bottle", 
                         "tuna_fish_can", "pudding_box", "gelatin_box", "potted_meat_can", "banana", "pitcher_base", 
                         "bleach_cleanser", "bowl", "mug", "power_drill", "wood_block", "scissors", "large_marker", 
                         "large_clamp", "extra_large_clamp", "foam_brick"};
int object;
double *poseData;
std::vector<int> ros_clsData;
bool poseReady;
bool classReady;
bool objectReady;
bool gripperPoseReady;
bool repeated = false;
std::vector<Eigen::Vector3d> posePosition;
std::vector<Eigen::Quaterniond> poseOrientation;

Eigen::Vector3d gripperPosition;
Eigen::Quaterniond gripperOrientation;
Eigen::Vector3d basePosition;
Eigen::Quaterniond baseOrientation;
GraspIt::EigenTransform tableCaseTf = GraspIt::EigenTransform();
const std::vector<Eigen::Vector3d> YCB_extents = {
Eigen::Vector3d(0.105098, 0.103336, 0.147140),
Eigen::Vector3d(0.072948, 0.167432, 0.223122),
Eigen::Vector3d(0.051228, 0.097062, 0.184740),
Eigen::Vector3d(0.068346, 0.070898, 0.118506),
Eigen::Vector3d(0.099712, 0.071530, 0.215002),
Eigen::Vector3d(0.085656, 0.085848, 0.041788),
Eigen::Vector3d(0.140458, 0.136312, 0.044982),
Eigen::Vector3d(0.092226, 0.102030, 0.037278),
Eigen::Vector3d(0.106770, 0.061462, 0.099400),
Eigen::Vector3d(0.146328, 0.202874, 0.039542),
Eigen::Vector3d(0.159810, 0.157306, 0.293620),
Eigen::Vector3d(0.112422, 0.072590, 0.277178),
Eigen::Vector3d(0.161696, 0.163252, 0.060978),
Eigen::Vector3d(0.133400, 0.094318, 0.084588),
Eigen::Vector3d(0.202122, 0.229442, 0.061552),
Eigen::Vector3d(0.106668, 0.108480, 0.240242),
Eigen::Vector3d(0.110210, 0.257878, 0.015808),
Eigen::Vector3d(0.021110, 0.125212, 0.019532),
Eigen::Vector3d(0.140818, 0.174792, 0.040068),
Eigen::Vector3d(0.210450, 0.185262, 0.036514),
Eigen::Vector3d(0.052900, 0.077960, 0.067918)
};