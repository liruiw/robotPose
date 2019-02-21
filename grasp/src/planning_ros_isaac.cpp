#include "planning.h"

float dgauss(float mean, float stdDev) 
{
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

int main(int argc, char **argv) 
{

  /* initialize random seed: */
  srand (time(NULL));
	
  // The world file name will be the first argument
  std::string output_dir, default_dir, mat_file;
  std::string world, robot, frame;
  int max_plan_iter, iter_count, max_plan_result;

  //load all ros param
  ros::init(argc, argv, "grasp_listener");
  ros::NodeHandle nh;
  tf::TransformListener listener;
  ros::Publisher grasp_pub = nh.advertise<geometry_msgs::PoseArray>("grasp_pose", 1);

  nh.getParam("default_dir", default_dir);
  nh.getParam("world", world);
  nh.getParam("frame", frame);
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

  bool obstacle = true;
  bool createDir = true;

  // the output directory will be the second argument
  std::string outputDirectory(output_dir);
  GraspIt::EigenTransform robotTransform;
  GraspIt::EigenTransform objectTransform;
  GraspIt::EigenTransform obstacleTransform;
  GraspIt::EigenTransform targetPose;
  robotTransform.setIdentity();
  objectTransform.setIdentity();
  obstacleTransform.setIdentity();
  std::ofstream grasp_pose_file;
  ros::Rate loop_rate(30);
  geometry_msgs::PoseArray output_msg;
  SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

  while (ros::ok()) 
  {

    // look for object poses
    tf::StampedTransform transform;
    std::string target_frame = frame;
    ros_clsData.clear();
    objectPoses.clear();
    output_msg.poses.clear();

    for (int i = 1; i < YCB_classes.size(); i++)
    {
      std::string source_frame = "00_" + YCB_classes[i] + "_world";
      try
      {
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        ros_clsData.push_back(i);

        // rotation
        tf::Quaternion tfQuat = transform.getRotation();

        // translation
        tf::Vector3 tfVec = transform.getOrigin();
        objectPoses.push_back(transf(Eigen::Quaterniond(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()),
          Eigen::Vector3d(tfVec.getX() * 1000, tfVec.getY() * 1000, tfVec.getZ() * 1000)));
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }

    // look for gripper pose
    std::string source_frame = "panda_hand_world";
    try
    {
      listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

      // rotation
      tf::Quaternion tfQuat = transform.getRotation();

      // translation
      tf::Vector3 tfVec = transform.getOrigin();

      gripperInitialPose = transf(Eigen::Quaterniond(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()),
        Eigen::Vector3d(tfVec.getX() * 1000, tfVec.getY() * 1000, tfVec.getZ() * 1000));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // look for table pose
    source_frame = "table_world";
    try
    {
      listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

      // rotation
      tf::Quaternion tfQuat = transform.getRotation();

      // translation
      tf::Vector3 tfVec = transform.getOrigin();

      tablePose = transf(Eigen::Quaterniond(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()),
        Eigen::Vector3d(tfVec.getX() * 1000, tfVec.getY() * 1000, tfVec.getZ() * 1000));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // if some object is detected
    if (ros_clsData.size() > 0) 
    {
      // print object info
      for (int i = 0; i < ros_clsData.size(); i++)
      {
        std::cout << YCB_classes[ros_clsData[i]] << " detected" << std::endl;
        std::cout << "TF: " << objectPoses[i] << " "  << std::endl;
      }

      std::cout << "Hand TF: " << gripperInitialPose << std::endl;
      std::cout << "Table TF: " << tablePose << std::endl;

      // std::cout << "press enter to exit...";
      // while(getchar() == '\n')
      //  break;

      // sample one object as the target
      if(!repeated)
      {
        int index = rand() % ros_clsData.size();
        object = ros_clsData[index];
      }
      object = ros_clsData[0];

      // Load the graspit world
      std::string file_name = outputDirectory + "/" + YCB_classes[object] + "_grasp_pose.txt";
      std::cout << file_name << std::endl;
      std::string initial_world_iv, initial_world_xml;

      // so that we can run multiple times for one configuration
      if(!repeated) 
        graspitMgr->loadWorld(worldFilename); //load the world with anchor

      std::vector<GraspIt::EigenTransform> objectTransformList;
      std::vector<std::string> objectFileList;
      Eigen::Matrix4d poseMatrix; 	

      // load objects
      for(int i = 0; i < objectPoses.size(); ++i)
      {
        std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[ros_clsData[i]] + ".xml");
        bool graspable = false;
        if((int)ros_clsData[i] == object) { 
          graspable = true;
        }
        if(!repeated) {          
          graspitMgr->loadObject(objectFilename, YCB_classes[ros_clsData[i]], graspable, objectPoses[i]); //place as occluder
        }
        else {
          graspitMgr->moveObject(YCB_classes[ros_clsData[i]], objectPoses[i]); //ignore collsion, add switch?
        }     
      }

      //load gripper
      if(!repeated) {          
        graspitMgr->loadRobot(robotFilename, robot, gripperInitialPose); //place as occluder
      }
      else {
        graspitMgr->moveRobot(robot, gripperInitialPose); //ignore collsion, add switch?
      } 

      //load table
      if(!repeated) {          
        graspitMgr->loadObject(tableFilename, "dinner_table", false, tablePose); //place as occluder
      }
      else {
        graspitMgr->moveObject("dinner_table", tablePose); //ignore collsion, add switch?
      } 

      GraspIt::EigenTransform gripperPose;

      for (int i = 0; i < iter_count; i++) 
      {
				
        graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld_"  + std::to_string(i + 1) +  ".xml", createDir);
        graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld_" + std::to_string(i + 1) +  ".iv", createDir);
        std::cout << outputDirectory + "/worlds/startWorld_" << std::endl;

        SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner("YCB_Grasp", graspitMgr)); //create
        int maxPlanningSteps = max_plan_iter;
        int repeatPlanning = 1;
        int keepMaxPlanningResults = max_plan_result; // only save the best result
        bool finishWithAutograsp = true;

        // might need to tune up the simulated annealing parameter
        if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, finishWithAutograsp))
          std::cerr << "Error doing the planning." << std::endl;

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
        std::cout << outputDirectory << " " << filenamePrefix << std::endl;

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
          pose.position.z = translation[2] / 1000;
          pose.orientation.x = orientation.x();
          pose.orientation.y = orientation.y();
          pose.orientation.z = orientation.z();
          pose.orientation.w = orientation.w();
          output_msg.poses.push_back(pose);
        }
      }

      grasp_pub.publish(output_msg);
      repeated = true;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
