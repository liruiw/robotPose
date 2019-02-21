#include "planning.h"

float dgauss(float mean, float stdDev) {
  std::random_device randomness_device{};
  std::mt19937 rand_gen{randomness_device()};
  std::normal_distribution<> normal(mean, stdDev);
  return normal(rand_gen);
}

int getRotationCase(Eigen::Quaterniond quat, int target) {
  std::vector<Eigen::Vector3d> bb;
  std::vector<Eigen::Vector3d> normals;
  std::vector<Eigen::Vector3d> rotNormals;

  std::vector<double> normalsZ;
  
  // create bounding box
  normals.push_back(Eigen::Vector3d(0, 0, 1));
  normals.push_back(Eigen::Vector3d(0, 0, -1));
  normals.push_back(Eigen::Vector3d(1, 0, 0));
  normals.push_back(Eigen::Vector3d(-1, 0, 0));
  normals.push_back(Eigen::Vector3d(0, -1, 0));
  normals.push_back(Eigen::Vector3d(0, 1, 0));
  
  //compute normal vector
  //
  for (int i = 0; i < 6; i++)
  {
    rotNormals.push_back(quat *(normals[i]).normalized()); // 
  }
  for (int i = 0; i < 6; i++)
  {
    normalsZ.push_back((quat *(normals[i])).normalized()[2]); // 
  }
  for (int i = 0; i < 6; i++)
  {
    std::cout << "Normal vector: " << normals[i][0] << " " 
               << normals[i][1] << " " 
               << normals[i][2] << " "
               << std::endl;
    std::cout << "Rotate Normal vector: " << rotNormals[i][0] << " " 
               << rotNormals[i][1] << " " 
               << normalsZ[i] << " "
               << std::endl;          
  }
  //most negative z in normal vector
  int min_idx = std::min_element(normalsZ.begin(), normalsZ.end()) - normalsZ.begin(); 
  std::cout << "min_idx: " << min_idx << " min element " << normalsZ[min_idx] << std::endl;
  return min_idx + 1;
}

Eigen::Quaterniond euler2quat(float roll, float pitch, float yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()); 
  return q;
}


void rotationCase(Eigen::Quaterniond quat, int target){ //assume camera z up
  tableCaseTf.setIdentity();
  double ztrans = YCB_extents[target][2] * 1000;
  int faceDown = getRotationCase(quat, target);
  std::cout << "Case Num" << faceDown << std::endl; 
  switch(faceDown){
  case 2: 
  tableCaseTf.prerotate(euler2quat(M_PI, 0, 0));
  break;
  
  case 3: 
  tableCaseTf.prerotate(euler2quat(0, M_PI/2, 0));
  tableCaseTf.pretranslate(Eigen::Vector3d(0, 0, (YCB_extents[target][0] * 1000 - ztrans)/2));
  break;
  
  case 4:
  tableCaseTf.prerotate(euler2quat(0, -M_PI/2, 0));
  tableCaseTf.pretranslate(Eigen::Vector3d(0, 0, (YCB_extents[target][0] * 1000 - ztrans)/2));
  break;

  case 5: 
  tableCaseTf.prerotate(euler2quat(M_PI/2, 0, 0));
  tableCaseTf.pretranslate(Eigen::Vector3d(0, 0, (YCB_extents[target][1] * 1000 - ztrans)/2));
  break;

  case 6: 
  tableCaseTf.prerotate(euler2quat(-M_PI/2, 0, 0));
  tableCaseTf.pretranslate(Eigen::Vector3d(0, 0, (YCB_extents[target][1] * 1000 - ztrans)/2));
  break;
  }
}   


void objectPoseCallback(const geometry_msgs::PoseArray &poses)
{
  // load poses
  for(int i =0; i < poses.poses.size(); ++i)
  {
  poseOrientation.push_back(Eigen::Quaterniond(poses.poses[i].orientation.w, poses.poses[i].orientation.x, 
    poses.poses[i].orientation.y, poses.poses[i].orientation.z));
  posePosition.push_back(Eigen::Vector3d(poses.poses[i].position.x * 1000, 
    poses.poses[i].position.y * 1000, poses.poses[i].position.z * 1000));
  }
  poseReady = true;
}

void robotPoseCallback(const geometry_msgs::PoseArray &robotPose)
{
  // load robot poses
  gripperOrientation = Eigen::Quaterniond(robotPose.poses[0].orientation.w, robotPose.poses[0].orientation.x, 
  robotPose.poses[0].orientation.y, robotPose.poses[0].orientation.z);
  gripperPosition = Eigen::Vector3d(robotPose.poses[0].position.x * 1000, 
  robotPose.poses[0].position.y * 1000, robotPose.poses[0].position.z * 1000);
  baseOrientation = Eigen::Quaterniond(robotPose.poses[1].orientation.w, robotPose.poses[1].orientation.x, 
  robotPose.poses[1].orientation.y, robotPose.poses[1].orientation.z);
  basePosition = Eigen::Vector3d(robotPose.poses[1].position.x * 1000, 
  robotPose.poses[1].position.y * 1000, robotPose.poses[1].position.z * 1000);
  gripperPoseReady = true;
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
  ros::Subscriber gripper_sub = nh.subscribe("robot_pose", 1, robotPoseCallback); //base and gripper
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
  std::string worldFilename(default_dir + "/worlds/" + world + ".xml");
  std::string robotFilename(default_dir + "/models/robots/" + robot + "/" + robot + ".xml");

  bool createDir = true;

  std::string outputDirectory(output_dir);
  GraspIt::EigenTransform targetPose;
  GraspIt::EigenTransform relativePose;
  ros::Rate loop_rate(30);
  geometry_msgs::PoseArray output_msg;
  SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

  while (ros::ok()) { 

  if (objectReady && poseReady && gripperPoseReady && classReady) {

    for (int i = 0; i < ros_clsData.size(); i++)
    {
    std::cout << YCB_classes[ros_clsData[i]] << " detected" << std::endl;
    std::cout << "Rotation: " << poseOrientation[i].w() << " " 
                  << poseOrientation[i].x() << " " 
                  << poseOrientation[i].y() << " "
                  << poseOrientation[i].z() << " " << std::endl;
    std::cout << "Translation: " << posePosition[i][0] << " " << posePosition[i][1] << " " << posePosition[i][2] << std::endl;
    }

    std::string tableworldFilename(default_dir + "/worlds/panda_table_" + YCB_classes[object] + ".xml");  // Load the graspit world
    std::string initial_world_iv, initial_world_xml;
    if(!repeated) { 
    //so that we can run multiple times for one configuration
    graspitMgr->loadWorld(tableworldFilename); //load the world with anchor  
    }  
    
    for(int i = 0; i < posePosition.size(); ++i)
    {
    if((int)ros_clsData[i] == object){ //anchor pose
      targetPose.setIdentity();
      targetPose.prerotate(poseOrientation[i]);
      targetPose.pretranslate(posePosition[i]);
      
      rotationCase(baseOrientation.inverse() * poseOrientation[i], object);
    }
    }

    for(int i = 0; i < posePosition.size(); ++i)
    {
    std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[ros_clsData[i]] + ".xml");
    
    if((int)ros_clsData[i] != object){ // wrt the anchor
      relativePose.setIdentity();      
      relativePose.prerotate(poseOrientation[i]);
      relativePose.pretranslate(posePosition[i]);
      relativePose = tableCaseTf.inverse() * targetPose.inverse() * relativePose ;
      
      // relativePose.translate(tableCaseTf.translation());
      // relativePose.rotate(tableCaseTf.rotation());

      if(!repeated) {
      graspitMgr->loadObject(objectFilename, YCB_classes[ros_clsData[i]], false, relativePose); //place as occluder
      }
      else{
      graspitMgr->moveObject(YCB_classes[ros_clsData[i]], relativePose);
      }      
    }
    else {
       graspitMgr->moveObject(YCB_classes[ros_clsData[i]], tableCaseTf);
    }  
    }
    
    GraspIt::EigenTransform gripperPose;
    for (int i = 0; i < iter_count; i++) {
    GraspIt::EigenTransform gripperInitialPose = targetPose.inverse();
    gripperInitialPose.prerotate(gripperOrientation);
    gripperInitialPose.pretranslate(gripperPosition);
    graspitMgr->moveRobot(robot, gripperInitialPose); 
    
    graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld_"  + std::to_string(i + 1) +  ".xml", createDir);
    graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld_" + std::to_string(i + 1) +  ".iv", createDir);
    SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner("YCB_Grasp", graspitMgr)); //create
    int maxPlanningSteps = max_plan_iter;
    int repeatPlanning = 1;
    int keepMaxPlanningResults = max_plan_result; 
    bool finishWithAutograsp = true;

    // might need to tune up the simulated annealing parameter
    if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, 
          finishWithAutograsp))
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
    posePosition.clear();
    poseOrientation.clear();
    ros_clsData.clear();
    repeated = true;
  }
  ros::spinOnce();
  loop_rate.sleep();
  }
}