#include "planning.h"

float dgauss(float mean, float stdDev) {
	std::random_device randomness_device{};
	std::mt19937 rand_gen{randomness_device()};
	std::normal_distribution<> normal(mean, stdDev);
	return normal(rand_gen);
}

void graspPoseCallback(const geometry_msgs::PoseArray &poses)
{
  // load poses
}

void classIndexCallback(const std_msgs::Int32MultiArray &cls_indexes)
{
  // load classes
}

void targetIndexCallback(const std_msgs::Int32 &target_index)
{
  // load classes
}
int main(int argc, char **argv) {
	
    // The world file name will be the first argument
	std::string output_dir, default_dir, mat_file;
	std::string world, robot;
	int max_plan_iter, iter_count, max_plan_result, object;
	mat_t *mat;
	matvar_t *poses, *cls_indexes;
    //load all ros param
	ros::init(argc, argv, "grasp_listener");
	ros::NodeHandle nh("~");
	ros::Subscriber grasp_sub = nh.subscribe("poses", 1000, graspPoseCallback);
	ros::Subscriber classes_sub = nh.subscribe("target", 1000, targetIndexCallback); //same order
	nh.getParam("default_dir", default_dir);
	nh.getParam("world", world);
	nh.getParam("output_dir", output_dir);
	nh.getParam("max_plan_iter", max_plan_iter);
	nh.getParam("robot", robot);
	//nh.getParam("object", object);
	nh.getParam("iter_count", iter_count);
	nh.getParam("max_plan_result", max_plan_result);
	nh.getParam("mat_file", mat_file);

	std::string worldFilename(default_dir + "/worlds/" + world + ".xml");
	std::string robotFilename(default_dir + "/models/robots/" + robot + "/" + robot + ".xml");
	std::string tableworldFilename(default_dir + "/worlds/panda_table_" + YCB_classes[object] + ".xml");  // Load the graspit world

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
	std::string file_name = outputDirectory + "/" + YCB_classes[object] + "_grasp_pose.txt";
	std::string initial_world_iv, initial_world_xml;
	//apend to results
	SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());
	if (obstacle){
		graspitMgr->loadWorld(tableworldFilename); //load the world with anchor		
		// mat = Mat_Open(mat_file.c_str(), MAT_ACC_RDWR);
		// poses = Mat_VarRead(mat, (char*)"poses");
		// cls_indexes = Mat_VarRead(mat, (char*)"cls_indexes");
		std::vector<GraspIt::EigenTransform> objectTransformList;
		std::vector<std::string> objectFileList;
		unsigned poseSize = poses->nbytes/poses->data_size;
		unsigned clsSize = cls_indexes->nbytes/cls_indexes->data_size ;
		const double *poseData = static_cast<const double*>(poses->data); //load this from ros	
		const double *clsData = static_cast<const double*>(cls_indexes->data) ;
		Eigen::Matrix4d poseMatrix; 	
		
		for(int i = 0; i<poseSize / 12; ++i)
		{
			poseMatrix = Eigen::Matrix4d::Identity();
			for(int j=0; j<9; ++j) poseMatrix(j % 3, j / 3) = poseData[i * 12 + j];
			for(int j=9; j<12; ++j) poseMatrix(j % 3, j / 3) = poseData[i * 12 + j] * 1000; //scale		    
		    if((int)clsData[i] == object){ //assume only one, find its pose
		    	targetPose = GraspIt::EigenTransform(poseMatrix);
		    }
		}
		for(int i = 0; i < poseSize / 12; ++i)
		{
			Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Identity();	    
			std::string objectFilename(default_dir + "/models/objects/" + YCB_classes[clsData[i]] + ".xml");
			objectFileList.push_back(objectFilename);
			if((int)clsData[i] != object){
				for(int j=0; j<9; ++j) poseMatrix(j % 3, j / 3) = poseData[i * 12 + j];
				for(int j=9; j<12; ++j) poseMatrix(j % 3, j / 3) = poseData[i * 12 + j] * 1000; //scale
				GraspIt::EigenTransform relativePose = targetPose.inverse();
				GraspIt::EigenTransform Pose(poseMatrix);
				relativePose.translate(Pose.translation());
				relativePose.rotate(Pose.rotation());
				objectTransformList.push_back(relativePose);
		    	graspitMgr->loadObject(objectFilename, YCB_classes[clsData[i]], false, relativePose); //place as occluder
		    }
		}
	}
	else{
		graspitMgr->loadWorld(worldFilename);
		graspitMgr->loadRobot(robotFilename, robot, robotTransform);
	}
	
	Eigen::Vector3d gripperPos;
	std::vector<Eigen::Matrix4d> graspPose;
	std::vector<double> jointVals;
	std::vector<double> energies;
	graspPose.push_back(Eigen::Matrix4d::Identity());
	energies.push_back(100.0);
	jointVals.push_back(0.0); //dummies

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
	    	Eigen::Matrix4d relative_tf = it->getObjectToHandTransform().matrix();
	    	Eigen::Vector4d pos = relative_tf.col(3);
	    	bool far = true;
	    	for (unsigned index = 0; index < graspPose.size(); index++) {
	    		if ( (graspPose[index].col(3) - pos).norm() < 80) { //by distance of hand center
	    			far = false;
	    			if (energy < energies[index]){ // choose best
	    				graspPose[index] = relative_tf;
	    				energies[index] = energy;
	    				jointVals[index] = jointVal[0];
	    				break;
	    			}  
	    		}
	    	}
	    	if(far){ 
	    		graspPose.push_back(relative_tf);
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
	grasp_pose_file.close();
}