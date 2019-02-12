#ifndef MAPPING_PCD_H
#define MAPPING_PCD_H

#include "pose6DOF.h"
#include "cloudOperations.h"
#include "timeMeasure.h"
#include "quadMap.h"
#include "map25Dplanner.h"
#include "commander.h"

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MenuEntry.h>

#include <iostream>
#include <string>
#include <vector>

// pcl libs
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// instal : sudo apt-get install ros-kinetic-pcl-conversions 


// These are different modes, that can be used to plann in the map
enum PlannMode{ NONE, FOLLOW_FRONTIERS, FOLLOW_GIVEN };

// TODO: implement frontier sorting
class PlannModeFollowFrontiers{
	private:
		Commander* rCommander;
		int frontierExistenceSkipStep;  // avoid replanning each step
		int checkDowncounter;
	public:
		PlannModeFollowFrontiers(){ checkDowncounter=0; frontierExistenceSkipStep=6; };
		
		void setCommander(Commander* _comander){ rCommander=_comander; };
		
		void doOneStep(Pose6DOF robotPose, Quad25Dmap::FrontierContainer* frontiers){
			// check path feasibility
			bool feasiblePath = rCommander->isActualPathFeasible();
			if(!feasiblePath){
				printf("Current path is unfeasible!\n");	
			}
			
			// replann if path is unfeasible or check if frontier disappered after some time
			if( !feasiblePath || checkDowncounter==0 ){ // replann
				// now find the first feasible frontier - we solve both problems
				bool someNewPath = false;
				rCommander->stopRobot();
				rCommander->setLocation(robotPose);
				
				std::vector<Quad25Dmap::FloatPair>* filPtr = frontiers->getfrontiersFiltered();
				for(int i=0; i<filPtr->size(); ++i){
					rCommander->setGoal( Pose6DOF(filPtr->at(i).x, filPtr->at(i).y) );  
					// try to plann to that
					if(rCommander->planNewPath()){
						someNewPath = true;
						break;
					}	
			
				}
				
				if(!someNewPath){
					printf("No feasible frontiers found.\n");
				}
			
				checkDowncounter = frontierExistenceSkipStep; // reset downcounter
			}else{
				rCommander->republishActualPath();
				checkDowncounter--;
			}
		
			
		}
				/* former version:
			if(followFrontier){
				robCommander.setLocation(syncedPose);
				
				// select frontier to follow - just take the first one in the row	
				bool someNewPath = false;
				std::vector<Quad25Dmap::FloatPair>* filPtr = map.frontiers.getfrontiersFiltered();
				for(int i=0; i<filPtr->size(); ++i){
					robCommander.setGoal( Pose6DOF(filPtr->at(i).x, filPtr->at(i).y) );  
					// try to plann to that
					bool feasiblePath = robCommander.isActualPathFeasible();
					if(robCommander.planNewPath()){
						someNewPath = true;
						break;
					}	
			
				}
			
			}
			*/
		
		
};

class PlannModeFollowGiven{
	private:
		Commander* rCommander;
		int replanSkipStep;
		int stepCallCounter;
	public:
		Pose6DOF goal;
		bool hasNewGoal;
		
		
		PlannModeFollowGiven(){
			hasNewGoal=false;
			replanSkipStep = 16;
			stepCallCounter = 0;
		};
		
		void doOneStep(Pose6DOF robotPose){
			// printf("Following given\n");
			rCommander->setLocation(robotPose);
			rCommander->setGoal( goal );  	  
			bool feasiblePath = rCommander->isActualPathFeasible();
			if(!feasiblePath){
				printf("Current path is unfeasible!\n");	
			}
				  
			if( rCommander->pathLength()==0 || stepCallCounter%replanSkipStep==0 || hasNewGoal ){ // no path or unfeasible path -> plan it !feasiblePath
				rCommander->stopRobot();  		// stop the robot
				if(rCommander->planNewPath()){
					hasNewGoal = false;
				}	
			}else{
				rCommander->republishActualPath();	
			}
			
			stepCallCounter++;
		};
		
		// getters and setters
		void setCommander(Commander* _comander){ rCommander=_comander; };
		
};

class DropBeaconParams{
	private:
		bool enable;
		float dropDist;	// standard beacon drop distance
		float minDist; 	// not used yet beacons cannot be dropped closer than this distance
		std::vector<Pose6DOF> beaconPoses;
		
	public:
		DropBeaconParams(){};
		void setParams( bool _enable, float _dropDist, float _minDist){
			enable = _enable;
			dropDist = _dropDist;
			minDist = _minDist;
		};
		
		bool isEnable(){ return enable; };
		float getDropDist(){ return dropDist; };
		float getMinDist(){ return minDist; };
		
		// try if we can drop the beacon
		// true if succesfull
		bool dropBeacon( Pose6DOF *location){
			bool closeBeacon = false;
			for(int i=beaconPoses.size()-1; i>=0; i--){ //search from the last -> faster			
				if( location->distanceTo( beaconPoses.at(i) ) < (double) dropDist ){ //something too close
					closeBeacon = true;
					break;
				}
			}
			if(!closeBeacon){ // remember beacon position
				beaconPoses.push_back( Pose6DOF(location) );
			}
			return !closeBeacon;
		};		
};


class MapCreator{
  	private:
  		// cloud skip
	  	int pcdCounter;
		int pcdSkipStep;
		SensorPreprocModel senseParams;
		
		float mapExpOpAreaDistance;
		bool enableFrontiers;
		float neighSize;
		int minCandidates;
			
		bool shadowsEnable;
		float shadowsDist;	
			
		// debug pcd composition
		ros::Publisher* cloudDebugPubLoc;
		ros::Publisher* cloudDebugPubGlob;
		Quad25Dmap::MapArea elevLocalDebugArea;
		ros::Publisher* cloudObjectivePub;
		int globAccMultip;
		
		bool enableOccDebug;
		ros::Publisher* occDebugPub;
	
		// time measuring, statistics
		TimeMeasurer timer;
		int insertPoints; 				//points after downsampling and preprocessing
		double preprocessingTime;
		double mapInsertTime;
		double mapMarginTime;
		double mapFrontierTime;
		double mapShadowTime;
		double mapPlanningTime;
		int mapNodesCount;
		double debugPublishTime;
		bool enableDebugTime;
		
		// planning
		Commander robCommander;
		PlannMode plannMode;
		PlannModeFollowFrontiers modeFollowFrontiers;
		PlannModeFollowGiven modeFollowGiven;

		ros::Publisher* pathPub;	//TODO refractor to mode class
		
		DropBeaconParams beaconDrop;
  	public:
  		// 2.5 D map
  		Quad25Dmap::QuadMap map;
  		MapCreator();
		
		// callbacks
		void uberCallback(const visualization_msgs::MenuEntry::ConstPtr& msg); // receive commands
		void goalCallback(const nav_msgs::Path::ConstPtr& msg); // this feeds the mapping by the new goals to visit 
		void pcdPoseCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const nav_msgs::Odometry::ConstPtr& loc_msg);
	
		
		// main functionality
		void printStatistics();
		void publishDebugObjectives( Pose6DOF robotPose );
		void publishDoublePrecElevation( Pose6DOF* robotPose );

		// getters, setters
		void setElevDebugPublishers( ros::Publisher* pubElevLoc,  ros::Publisher* pubElevGlob, float elevLocalMapSize, int _globAccMultip){ 
			cloudDebugPubLoc=pubElevLoc; 
			cloudDebugPubGlob=pubElevGlob; 
			globAccMultip = _globAccMultip;
			elevLocalDebugArea = Quad25Dmap::MapArea( elevLocalMapSize, elevLocalMapSize, elevLocalMapSize/2, elevLocalMapSize/2);
		};
		
		void setObjectivePublisher( ros::Publisher* cloudObjectivePubIn){ cloudObjectivePub = cloudObjectivePubIn; };
		void setDebugOccPublisher(ros::Publisher* occDebugPubIn){ occDebugPub = occDebugPubIn; };
		void setPathPublisher( ros::Publisher* _pathPub){ pathPub = _pathPub; robCommander.init(_pathPub, &map); };
		
		void setEnableOccDebug(bool enableVal){ enableOccDebug=enableVal; };
		void setPCDSkipStep( int _skipStep ){ pcdSkipStep=_skipStep; };

		SensorPreprocModel* getSenseParamsPtr(){ return &senseParams; };
		
		void setMapExpOpAreaDistance( float _mapExpOpAreaDistance){ mapExpOpAreaDistance=_mapExpOpAreaDistance; };
		void setEnableFrontiers(bool _enableFrontiers){ enableFrontiers=_enableFrontiers; };
		void setFrontiersParameters( bool _enableFrontiers, float _neighSize, int _minCandidates ){   
			enableFrontiers=_enableFrontiers;
			neighSize=_neighSize;
			minCandidates=_minCandidates;
		};   
		void setShadowsParameters( bool _shadowsEnable, float _shadowsDist ){
			shadowsEnable = _shadowsEnable;
			shadowsDist = _shadowsDist;
		};
		void setPlannParams( PlannMode _mode){ plannMode=_mode; };
		void setBeaconDropParams( bool dropBeaconEnable, float dropBeaconDist, float dropBeaconMinDist ){ 
			beaconDrop.setParams( dropBeaconEnable, dropBeaconDist, dropBeaconMinDist);
		};
		
		void setEnableDebugTime(bool _enableDebugTime){ enableDebugTime=_enableDebugTime; };
 };


 #endif

