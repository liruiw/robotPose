#include "planning.h"

float dgauss(float mean, float stdDev) {
	std::random_device randomness_device{};
	std::mt19937 rand_gen{randomness_device()};
	std::normal_distribution<> normal(mean, stdDev);
	return normal(rand_gen);
}

void objectPoseCallback(const geometry_msgs::PoseArray &poses)
{
  // load poses
	for(int i =0; i < poses.poses.size(); ++i)
	{
		poseOrientation.push_back(Eigen::Quaterniond(poses.poses[i].orientation.x, 
			poses.poses[i].orientation.y, poses.poses[i].orientation.z, poses.poses[i].orientation.w));
		posePosition.push_back(Eigen::Vector3d(poses.poses[i].position.x * 1000, 
			poses.poses[i].position.y * 1000, poses.poses[i].position.z * 1000));
	}
	poseReady = true;
}

void classIndexCallback(const std_msgs::Int32MultiArray &cls_indexes)
{
	//load classes
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
	ros::init(argc, argv, "grasp_listener");
	ros::NodeHandle nh;
	ros::Subscriber grasp_sub = nh.subscribe("object_poses", 1000, objectPoseCallback);
	ros::Subscriber target_sub = nh.subscribe("grasp_target", 1000, targetIndexCallback); //same order
	ros::Subscriber classes_sub = nh.subscribe("grasp_classes", 1000, classIndexCallback); //same order
	ros::Publisher  grasp_pub = nh.advertise<geometry_msgs::Pose>("grasp_pose", 1000);
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
	
	SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());
	while (ros::ok()) { 
		if (objectReady && classReady && poseReady) {
			std::string tableworldFilename(default_dir + "/worlds/panda_table_" + YCB_classes[object] + ".xml");  // Load the graspit world
			std::cout<<  "start" << std::endl;
			std::string file_name = outputDirectory + "/" + YCB_classes[object] + "_grasp_pose.txt";
			std::string initial_world_iv, initial_world_xml;
			if(!repeated) { //so that we can run multiple times for one configuration
				graspitMgr->loadWorld(tableworldFilename); //load the world with anchor	
			}	
			std::vector<GraspIt::EigenTransform> objectTransformList;
			std::vector<std::string> objectFileList;
			Eigen::Matrix4d poseMatrix; 	
			
			for(int i = 0; i<posePosition.size(); ++i)
			{
				poseMatrix = Eigen::Matrix4d::Identity();
			    if((int)ros_clsData[i] == object){ //assume only one, find its pose
			    	targetPose = GraspIt::EigenTransform();
			    	targetPose.translate(posePosition[i]);
			    	targetPose.rotate(poseOrientation[i]);
			    }
			}
			for(int i = 0; i < posePosition.size(); ++i)
			{
				Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Identity();	    
				std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[ros_clsData[i]] + ".xml");
				objectFileList.push_back(objectFilename);
				if((int)ros_clsData[i] != object){
					GraspIt::EigenTransform relativePose = targetPose.inverse();
					relativePose.translate(posePosition[i]);
					relativePose.rotate(poseOrientation[i]);
					objectTransformList.push_back(relativePose);
					if(ros_clsData[i] < 10) {
						if(!repeated) {
				    		graspitMgr->loadObject(objectFilename, YCB_classes[ros_clsData[i]], false, relativePose); //place as occluder
				    	}
				    	else{
				    		graspitMgr->moveObject(YCB_classes[ros_clsData[i]], relativePose);
				    	}						
					}
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
			for (int i = 0; i < iter_count; i++) {
				
			    float scale = dgauss(500, 200);; //sphere radius
			    float x = dgauss(0, 1);  float y = dgauss(0, 1); float z = dgauss(0, 1);
			    float r = sqrt(x*x + y*y + z*z);
			    x = x / r * scale; y = y / r * scale; z = z / r * scale;
			    
			    gripperPos = Eigen::Vector3d(x,y,z);
			    robotTransform.setIdentity();
			    Eigen::Quaterniond quaternion(dgauss(0, 1), dgauss(0, 1), dgauss(0, 1), dgauss(0, 1));
			    
			    robotTransform.rotate(quaternion);
		  		robotTransform.translate(gripperPos); //random initial rotation or might take in as input
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
			    	relativeTf = it->getObjectToHandTransform().matrix();
			    	gripperPose	= it->getObjectToHandTransform();
			    	Eigen::Vector4d pos = relativeTf.col(3);
			    	bool far = true;
			    	for (unsigned index = 0; index < graspPose.size(); index++) {
			    		if ( (graspPose[index].col(3) - pos).norm() < 80) { //by distance of hand center
			    			far = false;
			    			if (energy < energies[index]){ // choose best
			    				graspPose[index] = relativeTf;
			    				energies[index] = energy;
			    				jointVals[index] = jointVal[0];
			    				break;
			    			}  
			    		}
			    	}
			    	if(far){ 
			    		graspPose.push_back(relativeTf);
			    		energies.push_back(energy); // discreticize the pose
			    		jointVals.push_back(jointVal[0]);	    		
			    	}
			    }
			}
			grasp_pose_file.open(file_name.c_str(), std::ofstream::app); 
			for (unsigned index = 0; index < graspPose.size(); index++) {
		    	if(energies[index] < 45.0)  { //skip the first one and threshold
		    		grasp_pose_file << graspPose[index] << "," << jointVals[index] << 
						 "," <<  energies[index] << "\n"; //for recovering finger joint and energy
				}
			}
			geometry_msgs::Pose output_msg;
			Eigen::Quaterniond orientation =  Eigen::Quaterniond(gripperPose.rotation());
			Eigen::Vector3d translation = gripperPose.translation();
			output_msg.position.x = gripperPose.translation()[0] / 1000;
			output_msg.position.y = gripperPose.translation()[1] / 1000;
			output_msg.position.z = gripperPose.translation()[2] / 1000;
			output_msg.orientation.x = orientation.x();
			output_msg.orientation.y = orientation.y();
			output_msg.orientation.z = orientation.z();
			output_msg.orientation.w = orientation.w();
			grasp_pub.publish(output_msg); 
			objectReady = false;
			poseReady = false;
			classReady = false;
			posePosition.clear();
			poseOrientation.clear();
			ros_clsData.clear();
			grasp_pose_file.close();
			repeated = true;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}