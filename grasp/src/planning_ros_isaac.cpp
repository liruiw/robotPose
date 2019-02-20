#include "planning.h"

float dgauss(float mean, float stdDev) 
{
  std::random_device randomness_device{};
  std::mt19937 rand_gen{randomness_device()};
  std::normal_distribution<> normal(mean, stdDev);
  return normal(rand_gen);
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
  std::string worldFilename(default_dir + "/worlds/" + world + ".xml");
  std::string robotFilename(default_dir + "/models/robots/" + robot + "/" + robot + ".xml");

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
    posePosition.clear();
    poseOrientation.clear();
    output_msg.poses.clear();

    for (int i = 1; i < YCB_classes.size(); i++)
    {
      std::string source_frame = "00_" + YCB_classes[i];

      try
      {
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        ros_clsData.push_back(i);

        // rotation
        tf::Quaternion tfQuat = transform.getRotation();
        poseOrientation.push_back(Eigen::Quaterniond(tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z()));

        // translation
        tf::Vector3 tfVec = transform.getOrigin();
        posePosition.push_back(Eigen::Vector3d(tfVec.getX() * 1000, tfVec.getY() * 1000, tfVec.getZ() * 1000));
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }

    // if some object is detected
    if (ros_clsData.size() > 0) 
    {
      // print object info
      for (int i = 0; i < ros_clsData.size(); i++)
      {
        std::cout << YCB_classes[ros_clsData[i]] << " detected" << std::endl;
        std::cout << "Rotation: " << poseOrientation[i].w() << " " 
                                  << poseOrientation[i].x() << " " 
                                  << poseOrientation[i].y() << " "
                                  << poseOrientation[i].z() << " " << std::endl;
        std::cout << "Translation: " << posePosition[i][0] << " " << posePosition[i][1] << " " << posePosition[i][2] << std::endl;
      }

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
      std::string tableworldFilename(default_dir + "/worlds/panda_table_" + YCB_classes[object] + ".xml");
      std::cout << "start " << tableworldFilename << std::endl;
      std::string file_name = outputDirectory + "/" + YCB_classes[object] + "_grasp_pose.txt";
      std::cout << file_name << std::endl;
      std::string initial_world_iv, initial_world_xml;

      // so that we can run multiple times for one configuration
      if(!repeated) 
        graspitMgr->loadWorld(tableworldFilename); //load the world with anchor

      std::vector<GraspIt::EigenTransform> objectTransformList;
      std::vector<std::string> objectFileList;
      Eigen::Matrix4d poseMatrix; 	
			
      for(int i = 0; i < posePosition.size(); ++i)
      {
        poseMatrix = Eigen::Matrix4d::Identity();
        // assume only one, find its pose
        if((int)ros_clsData[i] == object)
        { 
          targetPose.setIdentity();
          targetPose.translate(posePosition[i]);
          targetPose.rotate(poseOrientation[i]);
        }
      }

      for(int i = 0; i < posePosition.size(); ++i)
      {
        Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Identity();	    
        std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[ros_clsData[i]] + ".xml");
        objectFileList.push_back(objectFilename);

        if((int)ros_clsData[i] != object)
        {
          GraspIt::EigenTransform relativePose = targetPose.inverse();
          relativePose.translate(posePosition[i]);
          relativePose.rotate(poseOrientation[i]);
          objectTransformList.push_back(relativePose);

          if(!repeated) 
            graspitMgr->loadObject(objectFilename, YCB_classes[ros_clsData[i]], false, relativePose); //place as occluder
          else
            graspitMgr->moveObject(YCB_classes[ros_clsData[i]], relativePose);					
        }
      }

      Eigen::Vector3d gripperPos;
      std::vector<Eigen::Matrix4d> graspPose;
      std::vector<double> jointVals;
      std::vector<double> energies;
      graspPose.push_back(Eigen::Matrix4d::Identity());
      energies.push_back(100.0);
      jointVals.push_back(0.0); //dummies
      Eigen::Matrix4d relativeTf;
      GraspIt::EigenTransform gripperPose;

      for (int i = 0; i < iter_count; i++) 
      {
				
        float scale = dgauss(500, 200); //sphere radius
        float x = dgauss(0, 1);
        float y = dgauss(0, 1);
        float z = dgauss(0, 1);
        float r = sqrt(x*x + y*y + z*z);
        x = x / r * scale;
        y = y / r * scale;
        z = z / r * scale;

        gripperPos = Eigen::Vector3d(x, y, z);
        robotTransform.setIdentity();
        Eigen::Quaterniond quaternion(dgauss(0, 1), dgauss(0, 1), dgauss(0, 1), dgauss(0, 1));

        // random initial rotation or might take in as input
        robotTransform.rotate(quaternion);
        robotTransform.translate(gripperPos); 
        graspitMgr->moveRobot(robot, robotTransform); 
        graspitMgr->moveObject(YCB_classes[object], objectTransform); 
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
          relativeTf = it->getObjectToHandTransform().matrix();
          gripperPose = it->getObjectToHandTransform();
          Eigen::Vector4d pos = relativeTf.col(3);
          bool far = true;
	        Eigen::Quaterniond orientation =  Eigen::Quaterniond(gripperPose.rotation());
          Eigen::Vector3d translation = gripperPose.translation();

          geometry_msgs::Pose pose;
          pose.position.x = gripperPose.translation()[0] / 1000;
          pose.position.y = gripperPose.translation()[1] / 1000;
          pose.position.z = gripperPose.translation()[2] / 1000;
          pose.orientation.x = orientation.x();
          pose.orientation.y = orientation.y();
          pose.orientation.z = orientation.z();
          pose.orientation.w = orientation.w();
          output_msg.poses.push_back(pose);
          graspPose.push_back(relativeTf);
        }
      }

      grasp_pub.publish(output_msg);
      repeated = true;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
