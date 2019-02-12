#include "commander.h"


void Commander::init( ros::Publisher* _pathPub, Quad25Dmap::QuadMap* _map ){
	pathPub = _pathPub;
	map = _map;
	robotRadius = 0.165;
	pathNewCounter = 0;
	planner.setRobotRadius(robotRadius);
	planner.setMaxPlanningTime(0.4);
	doingManeuver = false;
}


bool Commander::isActualPathFeasible(){
	return map->isPathTraversable( &actualPath, robotRadius*0.8 );
}


void Commander::stopRobot(){
	pathNewCounter++;
	std::vector<Pose6DOF> stopPath;
	publishPath( pathPub, &stopPath, pathNewCounter );
}


void Commander::republishActualPath(){
	publishPath( pathPub, &actualPath, pathNewCounter );
}


bool Commander::planNewPath(){
	// create problem description
	std::vector<Pose6DOF> startsP;
 	startsP.push_back( location );
	std::vector<Pose6DOF> goalsP;
	goalsP.push_back( goal );
	
	// test if goal or robot position are on an unknown place -> than run planinng over unknowns
	if((map->getCellType( location.getX(), location.getY() )==Quad25Dmap::CellType::UNKNOWN) || (map->getCellType( goal.getX(), goal.getY() )==Quad25Dmap::CellType::UNKNOWN)){
		planner.setUnknownsAreTraversable(true);
	}else{
		planner.setUnknownsAreTraversable(false);
	}
	planner.assignProblemDescription( 1, map, &startsP, &goalsP );

	// run the planner
	actualPath.clear();
	std::vector<std::vector<Pose6DOF>*> pathToFill;
	pathToFill.push_back(&actualPath);
	bool ret = planner.plan();
	
	if(ret){
  		planner.fillPathBySolution( &pathToFill);
  		// check if the end point is really in the path
  		Pose6DOF lastPos;
  		if(actualPath.size() > 0){
  			lastPos = actualPath.at( actualPath.size()-1 );
  		}
		if( lastPos.getX()==goal.getX() && lastPos.getY()==goal.getY() ){  // TODO change this to close enough ?
  			pathNewCounter++;
  			putPathOverMap( &actualPath );
  			publishPath( pathPub, &actualPath, pathNewCounter );
  		}else{
  			ret = false;
  		}
  	}else{
  		printf("planner not found the solution!\n");
  	}
	return ret;
}

void Commander::putPathOverMap(std::vector<Pose6DOF>* path){
  	for(int i=0; i<path->size(); ++i){
  		float heightOver = map->getHeight( path->at(i).getX(), path->at(i).getY() ) + 0.2;
		if(heightOver < DRAW_MIN_HEIGHT){
			heightOver = 0;
		}
  		path->at(i).setZ( heightOver );
  	}  	
}


//  ----------------- maneuvers -------------------
  	
void Commander::lookAround( double totalYaw){
	// TODO implement command for the tracker

}



//  -----------------  publish path -------------------

void publishPath(ros::Publisher* pub, std::vector<Pose6DOF>* pathOut, int MessageMark ){
  nav_msgs::Path msg;
  //add time of the path creation
  msg.header.stamp.nsec = MessageMark;

  //add poses to the path
  geometry_msgs::PoseStamped pose;
  pose.header.stamp.nsec = MessageMark;
  std::vector<geometry_msgs::PoseStamped> plan;

  // firts node contains length of the path
  int pathLength = pathOut->size();
  msg.poses.resize(pathLength);
  for (int i=0; i<pathLength; i++){
    pose.pose.position.x = pathOut->at(i).getX();
    pose.pose.position.y = pathOut->at(i).getY();
    pose.pose.position.z = pathOut->at(i).getZ();
    pose.pose.orientation.x = pathOut->at(i).getQX();
    pose.pose.orientation.y = pathOut->at(i).getQY();
    pose.pose.orientation.z = pathOut->at(i).getQZ();
    pose.pose.orientation.w = pathOut->at(i).getQW();
    msg.poses[i] = pose;
  }

  if(pathLength > 0){
    msg.header.frame_id = "camera_depth_optical_frame";
  //  msg.header.stamp = plan[0].header.stamp;
  }
  
  pub->publish(msg);
}
