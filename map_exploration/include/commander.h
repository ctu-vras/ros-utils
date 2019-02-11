#ifndef _COMMANDER_H
#define _COMMANDER_H

#include "pose6DOF.h"
#include "map25Dplanner.h"
#include "quadMapSupport.h"
#include "ros/ros.h"   // only because of the path publisher
#include <nav_msgs/Path.h>

class Commander{
  private:
  	std::vector<Pose6DOF> actualPath;
	map25Dplanner planner;
	ros::Publisher* pathPub;
	Pose6DOF goal; 
	Pose6DOF location;
	Quad25Dmap::QuadMap* map;
	
	
	int pathNewCounter;  // this ensures that the path is taken by the tracker as the new one - increment before by 1!
	
	// robot params
	float robotRadius; 	// half of the robot diameter
	// half of the robot diameter  bool haveValidPath;	// half of the robot diameter
	
	// maneuvers
	bool doingManeuver;
	
  public:
  	
  	Commander(){};
  	void init( ros::Publisher* _pathPub, Quad25Dmap::QuadMap* _map );

  	void stopRobot();
  	
  	int pathLength(){ return actualPath.size(); };
  	bool isActualPathFeasible();
  	
  	// returns true if the planning was succesfull
  	bool planNewPath();
  	void republishActualPath();
  	
  	// maneuvers
  	void lookAround( double totalYaw); // [rad]
  	 
  	// getters and setters
	void setGoal( Pose6DOF _goal){ goal=_goal; };
  	void setLocation( Pose6DOF loc){ location=loc; };
  	
  	// put path over the map - it changes the z coordinates
  	void putPathOverMap(std::vector<Pose6DOF>* path);
  	
 };
 
 void publishPath(ros::Publisher* pub, std::vector<Pose6DOF>* pathOut, int MessageMark );

 #endif

