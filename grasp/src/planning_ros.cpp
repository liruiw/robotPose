#include "planning.h"

float dgauss(float mean, float stdDev) {
  std::random_device randomness_device{};
  std::mt19937 rand_gen{randomness_device()};
  std::normal_distribution<> normal(mean, stdDev);
  return normal(rand_gen);
}


GraspIt::EigenTransform transf(Eigen::Quaterniond quat, Eigen::Vector3d translation) {
  GraspIt::EigenTransform Pose;
  Pose.setIdentity();
  Pose.translate(translation);
  Pose.rotate(quat);
  return Pose;
}

void objectPoseCallback(const geometry_msgs::PoseArray &poses)
{
  // load poses
  for(int i =0; i < poses.poses.size(); ++i)
  {
    objectPoses.push_back(transf(Eigen::Quaterniond(poses.poses[i].orientation.w, poses.poses[i].orientation.x, 
    poses.poses[i].orientation.y, poses.poses[i].orientation.z), Eigen::Vector3d(poses.poses[i].position.x * 1000, 
    poses.poses[i].position.y * 1000, poses.poses[i].position.z * 1000)));
  }
  poseReady = true;
}

void gripperPoseCallback(const geometry_msgs::Pose &robotPose)
{
  // load gripper poses
  gripperInitialPose = transf(Eigen::Quaterniond(robotPose.orientation.w, robotPose.orientation.x, 
    robotPose.orientation.y, robotPose.orientation.z), Eigen::Vector3d(robotPose.position.x * 1000, 
    robotPose.position.y * 1000, robotPose.position.z * 1000));
  gripperPoseReady = true;
}

void tablePoseCallback(const geometry_msgs::Pose &tPose)
{
  // load table poses
  tablePose = transf(Eigen::Quaterniond(tPose.orientation.w, tPose.orientation.x, 
    tPose.orientation.y, tPose.orientation.z), Eigen::Vector3d(tPose.position.x * 1000, 
    tPose.position.y * 1000, tPose.position.z * 1000));
}

void classIndexCallback(const std_msgs::Int32MultiArray &cls_indexes)
{
  // load classes
  for(int i =0; i < cls_indexes.data.size(); ++i)
  {
  ros_clsData.push_back(cls_indexes.data[i]);
  }
  classReady = true;
}

void targetIndexCallback(const std_msgs::Int32 &target_index)
{
  // load target
  object = target_index.data;
  objectReady = true;
}
int main(int argc, char **argv) {

  // The world file name will be the first argument
  std::string output_dir, default_dir, mat_file;
  std::string world, robot;
  int max_plan_iter, iter_count, max_plan_result;
  //load all ros param
  ros::init(argc, argv, "grasp_planning_node");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("object_poses", 1, objectPoseCallback);
  ros::Subscriber target_sub = nh.subscribe("grasp_target", 1, targetIndexCallback); //same order
  ros::Subscriber classes_sub = nh.subscribe("grasp_classes", 1, classIndexCallback); 
  ros::Subscriber table_sub = nh.subscribe("table_pose", 1, tablePoseCallback); 
  ros::Subscriber gripper_sub = nh.subscribe("robot_pose", 1, gripperPoseCallback); //base and gripper
  ros::Publisher  grasp_pub = nh.advertise<geometry_msgs::PoseArray>("grasp_pose", 1);
  nh.getParam("default_dir", default_dir);
  nh.getParam("world", world);
  nh.getParam("output_dir", output_dir);
  nh.getParam("max_plan_iter", max_plan_iter);
  nh.getParam("robot", robot);
  nh.getParam("iter_count", iter_count);
  nh.getParam("max_plan_result", max_plan_result);
  nh.getParam("mat_file", mat_file);
  std::cout << robot << std::endl;
  std::string worldFilename(default_dir + "/worlds/empty.xml");
  std::string robotFilename(default_dir + "/models/robots/" + robot + "/" + robot + ".xml");
  std::string tableFilename(default_dir + "/models/obstacles/dinner_table.xml");

  bool createDir = true;

  std::string outputDirectory(output_dir);
  ros::Rate loop_rate(30);
  geometry_msgs::PoseArray output_msg;
  SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

  while (ros::ok()) { 
    if (objectReady && poseReady && gripperPoseReady && classReady) {
      for (int i = 0; i < ros_clsData.size(); i++)
      {
        std::cout << YCB_classes[ros_clsData[i]] << " detected" << std::endl;
        std::cout << "TF: " << objectPoses[i] << " "  << std::endl;
      }
      std::cout << "Table TF: " << tablePose << " "  << std::endl;
      std::cout << "gripper TF: " << gripperInitialPose << " "  << std::endl;
      std::string initial_world_iv, initial_world_xml;
      if(!repeated) { 
        //so that we can run multiple times for one configuration
        graspitMgr->loadWorld(worldFilename); //load the world with anchor  
      }  

      //load gripper
      if(!repeated) {          
        graspitMgr->loadRobot(robotFilename, robot, gripperInitialPose); 
      }
      else {
        graspitMgr->moveRobot(robot, gripperInitialPose); //ignore collsion, add switch?
      } 

      // load objects
      for(int i = 0; i < objectPoses.size(); ++i)
      {
        std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[ros_clsData[i]] + ".xml");
        bool graspable = true;
        if(!graspitMgr->isObjectLoaded(YCB_classes[ros_clsData[i]])) {          
          graspitMgr->loadObject(objectFilename, YCB_classes[ros_clsData[i]], graspable, objectPoses[i]); //place as occluder
        }
        else {
          graspitMgr->moveObject(YCB_classes[ros_clsData[i]], objectPoses[i]); //ignore collsion, add switch?
        }    
        if((int)ros_clsData[i] == object) { 
          graspitMgr->setCurrentGraspableObject(YCB_classes[ros_clsData[i]]);
        }
      }

      //load table
      if(!repeated) {          
        graspitMgr->loadObject(tableFilename, "dinner_table", false, tablePose); //place as occluder
      }
      else {
        graspitMgr->moveObject("dinner_table", tablePose); //ignore collsion, add switch?
      } 

      GraspIt::EigenTransform gripperPose;
      for (int i = 0; i < iter_count; i++) {
        graspitMgr->moveRobot(robot, gripperInitialPose);         
        graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld_"  + std::to_string(i + 1) +  ".xml", createDir);
        graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld_" + std::to_string(i + 1) +  ".iv", createDir);
        SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner("YCB_Grasp", graspitMgr)); //create
        int maxPlanningSteps = max_plan_iter;
        int repeatPlanning = 1;
        int keepMaxPlanningResults = max_plan_result; 
        bool finishWithAutograsp = true;

        // might need to tune up the simulated annealing parameter
        if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults,  finishWithAutograsp))
        {
          std::cerr << "Error doing the planning." << std::endl;

        }
        std::string resultsWorldDirectory = outputDirectory + "/worlds/";
        std::string filenamePrefix =  YCB_classes[object] + "_result_world";
        bool saveInventor = true; 
        bool saveGraspit = true;

        std::vector<GraspIt::EigenGraspResult> allGrasps;
        planner->getResults(allGrasps);

        planner->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, saveGraspit, saveInventor, createDir);
        // Iterate through all results and print information about the grasps:
        std::cout << "Grasp results:" << std::endl;
        std::vector<GraspIt::EigenGraspResult>::iterator it;
        std::cout << outputDirectory << filenamePrefix << std::endl;

        for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
        {
          std::cout << *it << std::endl;
          double energy = it->getEnergy();
          std::vector<double> jointVal = it->getEigenGraspValues();
          gripperPose = it->getObjectToHandTransform();
          Eigen::Quaterniond orientation =  Eigen::Quaterniond(gripperPose.rotation());
          Eigen::Vector3d translation = gripperPose.translation();
          geometry_msgs::Pose pose;
          pose.position.x = translation[0] / 1000;
          pose.position.y = translation[1] / 1000;
          pose.position.z = translation[2] / 1000; //scale translation
          pose.orientation.x = orientation.x();
          pose.orientation.y = orientation.y();
          pose.orientation.z = orientation.z();
          pose.orientation.w = orientation.w();
          output_msg.poses.push_back(pose);
        }
      }
      grasp_pub.publish(output_msg); 
      objectReady = false; poseReady = false;
      classReady = false; gripperPoseReady = false;
      objectPoses.clear();
      ros_clsData.clear();
      repeated = true;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}