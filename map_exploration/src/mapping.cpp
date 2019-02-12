#include "mapping.h"
#include <pcl/filters/shadowpoints.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
// ----------------- main callbacks --------------------


MapCreator::MapCreator(){ 
	pcdCounter=0; 
	pcdSkipStep=1; 
	
	robCommander.init(pathPub, &map); 
	
	enableOccDebug=false;
	mapExpOpAreaDistance=2;
	
	// frontiers
	neighSize = 1;
	minCandidates = 5;
	
	// planning
	plannMode = PlannMode::NONE;
	modeFollowGiven.setCommander(&robCommander);
	modeFollowGiven.goal = Pose6DOF( 0, 0);
	modeFollowFrontiers.setCommander(&robCommander);
	
	
}


// receive commands
void MapCreator::uberCallback(const visualization_msgs::MenuEntry::ConstPtr& msg){
	// title - name of the target node, command - the command for the current node  
	if(true){ //TODO check title  msg->title
		std::string follow("follow_frontiers");
		if( follow==msg->command ){
			// followFrontier = true;
			std::cout << "TODO Command: follow frontiers received.\n";
		}else{
			std::string stopFollow("stop_follow_frontiers");
			if( stopFollow==msg->command ){
				// followFrontier = false;
				std::cout << "TODO Command: stop follow frontiers received.\n";
			}
		}
	}
}

// this enables the user to set a goal on the fly
void MapCreator::goalCallback(const nav_msgs::Path::ConstPtr& msg){ 
	if(msg->poses.size() > 0){
		Eigen::Quaternionf tmpQuat;
  	  	tmpQuat.x() = msg->poses[0].pose.orientation.x;
  	  	tmpQuat.y() = msg->poses[0].pose.orientation.y;
  	  	tmpQuat.z() = msg->poses[0].pose.orientation.z;
  	  	tmpQuat.w() = msg->poses[0].pose.orientation.w;
      	this->modeFollowGiven.goal = Pose6DOF( Eigen::Vector3f(msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z), tmpQuat );
		modeFollowGiven.hasNewGoal = true; 			//TODO put together with a setter in a class
		printf("New goal has been received.\n");	//TODO put together with a setter in a class
	}
}


void MapCreator::pcdPoseCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const nav_msgs::Odometry::ConstPtr& loc_msg){
	// test the covariance
  	bool covarOK = ((loc_msg->pose.covariance[0]+loc_msg->pose.covariance[7]+loc_msg->pose.covariance[14]) < 0.5) ? true : false;
  	
  	if(pcdCounter%pcdSkipStep==0 && covarOK){
  		timer.start();
  
		// create pose from the message
		Pose6DOF syncedPose;
  		syncedPose.fillPose(loc_msg->header.stamp.toSec(), 
  			loc_msg->pose.pose.position.x,
  			loc_msg->pose.pose.position.y,
  			loc_msg->pose.pose.position.z,
  			loc_msg->pose.pose.orientation.x,
  			loc_msg->pose.pose.orientation.y,
  			loc_msg->pose.pose.orientation.z,
  			loc_msg->pose.pose.orientation.w);
  		
		// ******************** run the preprocessing *************************
		// convert input message to pcl format
		pcl::PointCloud<pcl::PointXYZ> cloud;
		// pcl::fromROSMsg (*msg, cloud);
				  	
		// assign sensor origin to the cloud
		cloud.sensor_origin_ = syncedPose.getTXZY();
		cloud.sensor_orientation_ = syncedPose.getOrientation();
				
		CloudOperations::preprocessCloud( &cloud, msg, &senseParams);
				
		
					//********************************
					//filtering
					/*
					pcl::PointCloud<pcl::PointXYZ>::Ptr input;
					input = cloud.makeShared();

					pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
					ne.setInputCloud (input);

					pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
					ne.setSearchMethod (tree);

					pcl::PointCloud<pcl::PointNormal>::Ptr input_normals (new pcl::PointCloud<pcl::PointNormal>);
					ne.setKSearch (15);
					ne.compute (*input_normals);
		
					pcl::ShadowPoints <pcl::PointXYZ, pcl::PointNormal> spfilter (true); // Extract removed indices
					spfilter.setInputCloud (input);
					spfilter.setThreshold (0.1f);
					spfilter.setNormals (input_normals);

					pcl::PointCloud<pcl::PointXYZ> output;
					spfilter.filter (output);

					cloud = output;
					*/
					//********************************

				  	
			timer.end();
			preprocessingTime = timer.getElapsedTime();  	
			// **** end of the preprocessing **** 
				  	
			// ************************ build the map **************************
			timer.start();
	
			// insert the new points
			map.expandMapIfNearEdge( &syncedPose, mapExpOpAreaDistance ); // check, if we should expand the map
			insertPoints = cloud.width*cloud.height;
			for(int i=0; i<cloud.width*cloud.height; ++i){
				map.insert(cloud.at(i).x, cloud.at(i).y, cloud.at(i).z);
			}
			timer.end();
			mapInsertTime = timer.getElapsedTime();
			
			// geometrical margins
			timer.start();
			map.updatedNodesComputeMargins();
			timer.end();
			mapMarginTime = timer.getElapsedTime();
			
			// frontiers
			timer.start();
			if(enableFrontiers){
				map.updateFrontiers();  
				map.clusterFrontiers( neighSize, minCandidates);
			}
			timer.end();
			mapFrontierTime = timer.getElapsedTime();
			
			// shadowing
			timer.start();
			if(shadowsEnable){
				map.updateShadows(shadowsDist);
			}
			map.resetUpdateNodes();
			timer.end();
			mapShadowTime = timer.getElapsedTime();
			mapNodesCount = map.getNodeCount();
				  	
			// drop beacons
			if( beaconDrop.isEnable()){
				if(beaconDrop.dropBeacon( &syncedPose )){
					printf("Drop the beacon!\n");
				}
			}
				  	
			// ************************ publish debug messages ************************
			timer.start();
			
			// publish double precision elevation map
			publishDoublePrecElevation( &syncedPose );
				  	
			// publish current objectives
			this->publishDebugObjectives( syncedPose );

			if(enableOccDebug){
				CloudOperations::show25DmapAsOcc( occDebugPub, &map, 9, "camera_depth_optical_frame");
			}
					  
			timer.end();
			debugPublishTime = timer.getElapsedTime();
			
					  
			// ************************ planning ************************
			timer.start();
			
			if(plannMode==PlannMode::FOLLOW_FRONTIERS){
				modeFollowFrontiers.doOneStep(syncedPose, &(map.frontiers));
			}else if(plannMode==PlannMode::FOLLOW_GIVEN){
				modeFollowGiven.doOneStep( syncedPose );
			}
			
			timer.end();
			mapPlanningTime = timer.getElapsedTime();
			if(enableDebugTime){
				printStatistics();
			}
  					
	}
  	pcdCounter++;  	
}

// ------------------------- rest of the functionality ----------------------------
void MapCreator::printStatistics(){
	printf("\nMapping statistics for cloud: %d\n", pcdCounter);
	printf("preproc. time:     %0.3f s, valid points: %d\n", preprocessingTime, insertPoints);
	printf("map insert time:   %0.3f s, total nodes:  %d\n", mapInsertTime, mapNodesCount);
	printf("map margin time:   %0.3f s\n", mapMarginTime);
	printf("map frontier time: %0.3f s\n", mapFrontierTime);
	printf("map shadow time:   %0.3f s\n", mapShadowTime);
	printf("map planning time: %0.3f s\n", mapPlanningTime);
	printf("debug pub time:    %0.3f s\n\n", debugPublishTime);
}

void MapCreator::publishDebugObjectives( Pose6DOF robotPose ){
	pcl::PointCloud<pcl::PointXYZRGB> cloudObjectives;
	std::vector<Pose6DOF> objectives;
	map.frontiers.fillPosesByFrontiers(&objectives);
	// put all objectives markers over the map layer
	for(int i=0; i<objectives.size(); ++i){
		float heightOver = map.getHeight( objectives.at(i).getX(), objectives.at(i).getY() ) + 0.2;
		if(heightOver < DRAW_MIN_HEIGHT){
			heightOver = 0;
		}
		objectives.at(i).setZ(heightOver);
	}
	
	//map.interpolatePath( &objectives, &validPath );
	std::vector<Pose6DOF> goals;
	goals.push_back(modeFollowGiven.goal);
	// put all goal markers over the map layer
	for(int i=0; i<goals.size(); ++i){
		float heightOver = map.getHeight( goals.at(i).getX(), goals.at(i).getY() ) + 0.2;
		if(heightOver < DRAW_MIN_HEIGHT){
			heightOver = 0;
		}
		goals.at(i).setZ(heightOver);
	}
	CloudOperations::showObjectives( &cloudObjectives, robotPose, goals, &objectives, "camera_depth_optical_frame");
	sensor_msgs::PointCloud2 objMSG;
	CloudOperations::coloredCloudToROSmsg( &cloudObjectives, &objMSG);
	cloudObjectivePub->publish( objMSG );
}

void MapCreator::publishDoublePrecElevation( Pose6DOF* robotPose ){
	pcl::PointCloud<pcl::PointXYZRGB> cloudCost;
	pcl::PointCloud<pcl::PointXYZRGB> cloudCostGlob;
	elevLocalDebugArea.moveArea( robotPose );
	CloudOperations::fillColoredCloudByLocalElevMap( &map, &cloudCost, "camera_depth_optical_frame", &elevLocalDebugArea );		
	CloudOperations::fillColoredCloudByGlobElevMap( &map, &cloudCostGlob, "camera_depth_optical_frame", &elevLocalDebugArea, globAccMultip);
			
	// publish debug cloud - local
	sensor_msgs::PointCloud2 debMSG;
	CloudOperations::coloredCloudToROSmsg( &cloudCost, &debMSG );
	cloudDebugPubLoc->publish( debMSG );
		  	
	// publish debug cloud - global
	sensor_msgs::PointCloud2 debMSGglob;
	CloudOperations::coloredCloudToROSmsg( &cloudCostGlob, &debMSGglob );
	cloudDebugPubGlob->publish( debMSGglob );

}


