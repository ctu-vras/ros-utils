#include "mapping.h"
#include "quadMapSupport.h"

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// message synchronization:
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace sensor_msgs;
using namespace message_filters;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){
  std::cout << "Mapping node has started\n";

  if(argc >= 2){

  	ros::init(argc, argv, "robot_mapping25D_node");
  	ros::NodeHandle nh;

	// define ros params that are going to be used
	std::string topic_in_PointCloud;
	std::string topic_in_Localization;
	
	std::string topic_in_uberCommands;
	std::string topic_in_goals;
	
	std::string topic_out_objectives;
	std::string topic_out_path;
	
	std::string topic_out_debug_elev_local;
	float elevLocalMapSize;
	std::string topic_out_debug_elev_far;
	int lowResMultiplier;
	std::string topic_out_debug_occ;
	bool enableDebugOcc;
	bool enableDebugTime;
	
	bool enableTransCloudToRobotFrame;
	int pcdSkipStep;
	int pointSkipStep;
	float senseXMin;
	float senseXMax;
	float senseYMin;
	float senseYMax;
	float senseZMin;
	float senseZMax;
	
	// frontiers
	bool enableFrontiers;
	float frontierNeighSize;
	int frontierMinCandidates;
	
	// shadows
	bool shadowsEnable;
	float shadowsDist;
	
	// planning
	std::string textPlannMode;
	PlannMode plannMode;
	
	// rest of the mapping
	float mapExpOpAreaDistance;
	float mapResolution;
	float maxTravMargin;
	
	// dropping beacons
	bool dropBeaconEnable;
	float dropBeaconDist;
	float dropBeaconMinDist;	
	
	// load topic parameters using ros params
	nh.param<std::string>("/robot_mapping25D_node/inPointCloud", topic_in_PointCloud, "");
	nh.param<std::string>("/robot_mapping25D_node/inPose", topic_in_Localization, "");
	nh.param<std::string>("/robot_mapping25D_node/inUberCommands", topic_in_uberCommands, "");
	nh.param<std::string>("/robot_mapping25D_node/goals", topic_in_goals, "");

	nh.param<std::string>("/robot_mapping25D_node/outObjectives", topic_out_objectives, "");
	nh.param<std::string>("/robot_mapping25D_node/outPath", topic_out_path, "");

	nh.param<std::string>("/robot_mapping25D_node/outDebugLocalElevation", topic_out_debug_elev_local, "");
	nh.param<float>("/robot_mapping25D_node/outDebugLocalElevationSize", elevLocalMapSize, 4.0);
	nh.param<std::string>("/robot_mapping25D_node/outDebugFarElevation", topic_out_debug_elev_far, "");
	nh.param<int>("/robot_mapping25D_node/lowResMultip", lowResMultiplier, 4);
	
	nh.param<std::string>("/robot_mapping25D_node/outDebugOCC", topic_out_debug_occ, "");
	nh.param<bool>("/robot_mapping25D_node/enableDebugOCC", enableDebugOcc, false);
	nh.param<bool>("/robot_mapping25D_node/enableDebugTimes", enableDebugTime, false);
	
	// load aditional parameters
	nh.param<bool>("/robot_mapping25D_node/enableTransCloudToRobotFrame", enableTransCloudToRobotFrame, false);
	nh.param<int>("/robot_mapping25D_node/pcdSkipStep", pcdSkipStep, 1);
	nh.param<int>("/robot_mapping25D_node/pointSkipStep", pointSkipStep, 1);
	nh.param<float>("/robot_mapping25D_node/senseXmin", senseXMin, -100);
	nh.param<float>("/robot_mapping25D_node/senseXmax", senseXMax,  100);
	nh.param<float>("/robot_mapping25D_node/senseYmin", senseYMin, -100);
	nh.param<float>("/robot_mapping25D_node/senseYmax", senseYMax,  100);
	nh.param<float>("/robot_mapping25D_node/senseZmin", senseZMin, -100);
	nh.param<float>("/robot_mapping25D_node/senseZmax", senseZMax,  100);

	// frontier
	nh.param<bool>("/robot_mapping25D_node/enableFrontiers", enableFrontiers, false);
	nh.param<float>("/robot_mapping25D_node/frontierNeighSize", frontierNeighSize, 1.0);
	nh.param<int>("/robot_mapping25D_node/frontierMinCandidates", frontierMinCandidates, 5);
	
	// shadows
	nh.param<bool>("/robot_mapping25D_node/shadowsEnable", shadowsEnable, false);
	nh.param<float>("/robot_mapping25D_node/shadowsDist", shadowsDist, 0.5);
	
	// planning
	nh.param<std::string>("/robot_mapping25D_node/plannMode", textPlannMode, "");
	if(textPlannMode==std::string("followFrontiers")){
		plannMode = PlannMode::FOLLOW_FRONTIERS;
	}else if(textPlannMode==std::string("followGiven")){
		plannMode = PlannMode::FOLLOW_GIVEN;
	}else{
		plannMode = PlannMode::NONE;
	}
	
	// rest of the mapping
	nh.param<float>("/robot_mapping25D_node/mapExpOpAreaDistance", mapExpOpAreaDistance, 2);
	nh.param<float>("/robot_mapping25D_node/mapResolution", mapResolution, 0.05);
	nh.param<float>("/robot_mapping25D_node/maxTravMargin", maxTravMargin, 0.1);
	
	// drop becons based on the map
	nh.param<bool>("/robot_mapping25D_node/dropBeaconEnable", dropBeaconEnable, false);
	nh.param<float>("/robot_mapping25D_node/dropBeaconDist", dropBeaconDist, 12);
	nh.param<float>("/robot_mapping25D_node/dropBeaconMinDist", dropBeaconMinDist, 10);


	// print loaded topics
	printf("\nLoaded parameters (topics):\n");
	printf("In point cloud: %s\n", topic_in_PointCloud.c_str());
	printf("In pose:        %s\n", topic_in_Localization.c_str());
	printf("In commands:    %s\n", topic_in_uberCommands.c_str());
	printf("In goals:       %s\n\n", topic_in_goals.c_str());
	
	printf("Out objectives: %s\n", topic_out_objectives.c_str());
	printf("Out path:       %s\n", topic_out_path.c_str());
	
	printf("Debug elev. lo: %s\n", topic_out_debug_elev_local.c_str());
	printf("Debug elev. fa: %s\n", topic_out_debug_elev_far.c_str());
	printf("Debug occ grid:  %s", topic_out_debug_occ.c_str());
	if(enableDebugOcc){
		printf(" (enabled)\n");
	}else{
		printf(" (disabled)\n");
	}
	
   	// create the map based on the parameters
   	MapCreator mapCR;
   	mapCR.map.initMap( -32*mapResolution, -32*mapResolution, 64*mapResolution, 6 );
   	mapCR.map.setMaxTraversableMargin(maxTravMargin);
   	mapCR.setPCDSkipStep(pcdSkipStep);   	   	   
   	   	   
   	SensorPreprocModel* sensePar = mapCR.getSenseParamsPtr();   
   	sensePar->setPointSkipStep( pointSkipStep);
   	sensePar->setXcuts( senseXMin, senseXMax);
   	sensePar->setYcuts( senseYMin, senseYMax);
   	sensePar->setZcuts( senseZMin, senseZMax);
   	
   	sensePar->setTransToRobCoord( enableTransCloudToRobotFrame);
   	mapCR.setMapExpOpAreaDistance( mapExpOpAreaDistance);
   	mapCR.setFrontiersParameters( enableFrontiers, frontierNeighSize, frontierMinCandidates );   
   	mapCR.setShadowsParameters( shadowsEnable, shadowsDist );
   	mapCR.setPlannParams( plannMode );
   	mapCR.setEnableDebugTime( enableDebugTime);
   	mapCR.setBeaconDropParams( dropBeaconEnable, dropBeaconDist, dropBeaconMinDist );
  	
   	   
   	// create publishers
   	ros::Publisher pathOut = nh.advertise<nav_msgs::Path> ( topic_out_path, 10);
   	mapCR.setPathPublisher( &pathOut );
   	
   	ros::Publisher pcd3out = nh.advertise<sensor_msgs::PointCloud2> ( topic_out_objectives, 2);
   	mapCR.setObjectivePublisher( &pcd3out);
   	
   	ros::Publisher pcdLocOut = nh.advertise<sensor_msgs::PointCloud2> ( topic_out_debug_elev_local,2);
   	ros::Publisher pcdGlobOut= nh.advertise<sensor_msgs::PointCloud2> ( topic_out_debug_elev_far,2);
    mapCR.setElevDebugPublishers( &pcdLocOut, &pcdGlobOut, elevLocalMapSize, lowResMultiplier );
    
    ros::Publisher occOut = nh.advertise<nav_msgs::OccupancyGrid> ( topic_out_debug_occ,2);
    mapCR.setDebugOccPublisher( &occOut);
    mapCR.setEnableOccDebug( enableDebugOcc );
    
    // create subscribers
    ros::Subscriber uber_commander  = nh.subscribe( topic_in_uberCommands, 10, &MapCreator::uberCallback, &mapCR);
	ros::Subscriber goal_subscriber = nh.subscribe( topic_in_goals, 2, &MapCreator::goalCallback, &mapCR);

	// synced pcd and loc. topics: pointcloud, odometry
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcd_sub(nh, topic_in_PointCloud, 1);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, topic_in_Localization, 1);

	typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SelectedSyncPolicy;
	Synchronizer<SelectedSyncPolicy> syncer(SelectedSyncPolicy(10), pcd_sub, odom_sub);
	syncer.registerCallback(boost::bind(&MapCreator::pcdPoseCallback, &mapCR, _1, _2));

   	ros::spin(); 
  }else{
    ROS_ERROR("Not enough input arguments!");
  }
  return 0;
}



