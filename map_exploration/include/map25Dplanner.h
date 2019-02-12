#ifndef PLANNER_WRAPPER_H
#define PLANNER_WRAPPER_H

#include "pose6DOF.h"
#include "quadMap.h"

// basic stuff
#include <iostream>
#include <math.h>
#include <vector>

// OMPL
#include <ompl/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/util/PPM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class map25Dplanner{

  private:
     og::SimpleSetupPtr ss_;	// OMPL
     Quad25Dmap::QuadMap *mapForPlanning; // map for the planning
          
     int robotCount;
     bool unknownsAreTraversable;
     
     // this check validity of the step
     bool isStateValid(const ob::State *state) const;  // universal, prepared for n-robots
     bool isStateValidOneRobot(const ob::State *state) const;  		// enfastered for 1 robot - including unknown space
     bool isStateValidOneRobotOnMap(const ob::State *state) const;  // enfastered for 1 robot
     // max time limit
     double maxPlanningTime; 

	 // robot parameter used during the checking the state validity
	 float robotRadius;

  public:
 
     map25Dplanner();
          
     void assignProblemDescription(int robotCountIn, Quad25Dmap::QuadMap* map, std::vector<Pose6DOF>* startsIn, std::vector<Pose6DOF>* goalsIn);

     bool plan();

     // path pub and other tricks
     bool fillPathBySolution(std::vector<std::vector<Pose6DOF>*>* pathToFill);
     
     // getters and setters
     void setMaxPlanningTime(double _time){ maxPlanningTime = _time; };
     void setRobotRadius(float _robRad){ robotRadius = _robRad; };
     void setUnknownsAreTraversable(bool val){ unknownsAreTraversable = val; };
};

#endif
