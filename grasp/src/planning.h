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
GraspIt::EigenTransform transf(Eigen::Quaterniond quat, Eigen::Vector3d translation);
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
std::vector<GraspIt::EigenTransform> objectPoses;
// table offset [-894.329 -509.39 -820.374]
GraspIt::EigenTransform gripperInitialPose;
GraspIt::EigenTransform tablePose;
