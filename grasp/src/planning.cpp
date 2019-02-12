#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/GraspItHelpers.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <random>
#include <math.h>
#include <graspit/world.h>
#include <graspit/robot.h>
#include <graspit/body.h>
#include <graspit/grasp.h>
#include <graspit/graspitCore.h>

using GraspIt::GraspItSceneManager;
float dgauss(float mean, float stdDev) {
	std::random_device randomness_device{};
	std::mt19937 rand_gen{randomness_device()};
	std::normal_distribution<> normal(mean, stdDev);
	return normal(rand_gen);
}
int main(int argc, char **argv) {
	
    // The world file name will be the first argument
	std::string output_dir, default_dir;
	std::string world, robot, object;
	int max_plan_iter, iter_count, max_plan_result;
	ros::init(argc, argv, "node");
	ros::NodeHandle nh("~");
	nh.getParam("default_dir", default_dir);
	nh.getParam("world", world);
	nh.getParam("output_dir", output_dir);
	nh.getParam("max_plan_iter", max_plan_iter);
	nh.getParam("robot", robot);
	nh.getParam("object", object);
	nh.getParam("iter_count", iter_count);
	nh.getParam("max_plan_result", max_plan_result);
	std::string worldFilename(default_dir + "/worlds/" + world + ".xml");
	std::string robotFilename(default_dir + "/models/robots/" + robot + "/" + robot + ".xml");
	std::string objectFilename(default_dir + "/models/objects/" + object + ".xml");
	std::string tableworldFilename(default_dir + "/worlds/panda_table_" + object + ".xml");
    // Load the graspit world
	bool obstacle = false;
	bool createDir = true;
    // the output directory will be the second argument
	std::string outputDirectory(output_dir);
	std::cout << output_dir << std::endl;
    // in case one wants to view the initial world before planning, save it.

	GraspIt::EigenTransform robotTransform;
	GraspIt::EigenTransform objectTransform;
	GraspIt::EigenTransform obstacleTransform;
	robotTransform.setIdentity();
	objectTransform.setIdentity();
	Eigen::Vector3d objectPos = Eigen::Vector3d(700.0, 700., 920.0); //on table and should enough for avoiding grasp from bottom
	//objectTransform.translate(objectPos);
	obstacleTransform.setIdentity();
	std::ofstream grasp_pose_file;
	std::string file_name = outputDirectory + "/" + object + "_grasp_pose.txt";
	std::string initial_world_iv, initial_world_xml;
	//apend to results
	SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());
	if (obstacle){
		graspitMgr->loadWorld(tableworldFilename);
	}
	else{
		graspitMgr->loadWorld(worldFilename);
		graspitMgr->loadRobot(robotFilename, robot, robotTransform);
	//graspitMgr->loadObject(default_dir + "/models/obstacles/dinner_table.xml", "dinner_table", false, obstacleTransform); //obstacle to simulate real world
		graspitMgr->loadObject(objectFilename, object, true, objectTransform);		
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
  		robotTransform.translate(gripperPos); //random initial rotation
  		//objectTransform.rotate(Eigen::AngleAxisd(1.5707, Eigen::Vector3d::UnitX())); //invert z 
  		graspitMgr->moveRobot(robot, robotTransform); 
  		graspitMgr->moveObject(object, objectTransform); 
  		graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld_"  + std::to_string(i) +  ".xml", createDir);
  		graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld_" + std::to_string(i) +  ".iv", createDir);
	    // Create the planner which accesses the graspit world.
  		SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner("YCB_Grasp", graspitMgr));

	    // load different initial poses
	    // Number of iterations for the planning algorithm
  		int maxPlanningSteps = max_plan_iter;

	    // Number of times to repeat the planning process    
  		int repeatPlanning = 1;

	    // Maximum number of planning results to keep (of each planning repeat)
	    int keepMaxPlanningResults = max_plan_result; // only save the best result
	    // Finalize each planning result with an "auto-grasp" to ensure there really are
	    // contacts between fingers and objects (sometimes, the grasp result is just very
	    // close to the object, but not really touching it).
	    bool finishWithAutograsp = true;
	    // might need to tune up the simulated annealing parameter
	    if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, finishWithAutograsp))
	    {
	    	std::cerr << "Error doing the planning." << std::endl;

	    }
	    std::string resultsWorldDirectory = output_dir;
	    
	    // each result file will start with this prefix:
	    std::string filenamePrefix = "world";
	    
	    // specify to save as inventor files:
	    bool saveInventor = true;
	    
	    // specify to save as graspit world files:
	    bool saveGraspit = true;

	    // Now, save the results.
	    //planner->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, saveGraspit, saveInventor, createDir);
	    // get the planning results
	    std::vector<GraspIt::EigenGraspResult> allGrasps;
	    planner->getResults(allGrasps);

	    // Iterate through all results and print information about the grasps:
	    std::cout << "Grasp results:" << std::endl;
	    std::vector<GraspIt::EigenGraspResult>::iterator it;

	    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
	    {
	    	std::cout << *it << std::endl;
	    	double energy = it->getEnergy();
	    	std::vector<double> jointVal = it->getEigenGraspValues();
	    	Eigen::Matrix4d relative_tf = it->getObjectToHandTransform().matrix();
	    	Eigen::Vector4d pos = relative_tf.col(3);
	    	bool far = true;
	    	for (unsigned index = 0; index < graspPose.size(); index++) {
	    		std::cout << (graspPose[index].col(3) - pos).norm() << std::endl;
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
	    	if(far){ //or as discretize
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