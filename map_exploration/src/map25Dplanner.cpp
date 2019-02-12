#include "map25Dplanner.h"

map25Dplanner::map25Dplanner(){
  this->mapForPlanning = nullptr;
  this->maxPlanningTime = 5;
  this->robotRadius = 0.001;
  this->unknownsAreTraversable = false;
}

void map25Dplanner::assignProblemDescription( int robotCountIn, Quad25Dmap::QuadMap* map, std::vector<Pose6DOF>* startsIn, std::vector<Pose6DOF>* goalsIn){
  // assign basic parameters
  mapForPlanning = map;
  robotCount = robotCountIn;

  // start to with the description for OMPL library
  // printf("Planner: map params; width: %f, height: %f\n", mapForPlanning->getFieldSizes()->getMapSize(), mapForPlanning->getFieldSizes()->getMapSize());

  auto space(std::make_shared<ob::RealVectorStateSpace>());
  
  // space for each robot
  double spaceXmin = mapForPlanning->getFieldSizes()->getXmin();
  double spaceYmin = mapForPlanning->getFieldSizes()->getYmin();
  double spaceSquareSize = mapForPlanning->getFieldSizes()->getMapSize();
  
  for(int i=0; i<robotCount; i++){ // min and max coords
    space->addDimension( spaceXmin, spaceXmin+spaceSquareSize ); 
    space->addDimension( spaceYmin, spaceYmin+spaceSquareSize );
  }
  ss_ = std::make_shared<og::SimpleSetup>(space);
 
  // set state validity checking for this space
  if(robotCount==1){
  	if(unknownsAreTraversable){
  		ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValidOneRobot(state); });
  	}else{
  		ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValidOneRobotOnMap(state); });
  	}  	
  }else{
  	ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
  }
  space->setup();
  ss_->getSpaceInformation()->setStateValidityCheckingResolution( (double)0.0001 );
  ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));

  // obtain start and goal states
  ob::ScopedState<> start(ss_->getStateSpace());
  ob::ScopedState<> goal(ss_->getStateSpace());

  for (int i=0; i<robotCount; i++){
    // fill the start and goal space
    start[2*i+0] = (float) startsIn->at(i).getX();
    start[2*i+1] = (float) startsIn->at(i).getY();
    goal[2*i+0] = (float) goalsIn->at(i).getX();
    goal[2*i+1] = (float) goalsIn->at(i).getY();

  }
  ss_->setStartAndGoalStates(start, goal);
}
 
 
bool map25Dplanner::plan(){
  bool ret = false;
  if (!ss_){
    return false;
  }
         
  // generate a few solutions; all will be added to the goal;
  for (int i = 0; i < 1; ++i){ // try it one times
    if (ss_->getPlanner()){
      ss_->getPlanner()->clear();
    }
    ss_->solve(maxPlanningTime); // max time for planner [s]
  }
  const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
  // OMPL_INFORM("Found %d solutions", (int)ns);
  if (ss_->haveSolutionPath()){
    ss_->simplifySolution();
    og::PathGeometric &p = ss_->getSolutionPath();
    ss_->getPathSimplifier()->simplifyMax(p);
   // ss_->getPathSimplifier()->smoothBSpline(p);
    
    ret = true;
  }
  return ret;
}
 
bool map25Dplanner::isStateValid(const ob::State *state) const{
  // read all coordinates to vectors
  bool ret = true;
  std::vector<Pose6DOF> robotPoses;

  for(int i=0; i<robotCount; i++){
    const double xTest = state->as<ob::RealVectorStateSpace::StateType>()->values[2*i];
    const double yTest = state->as<ob::RealVectorStateSpace::StateType>()->values[2*i+1];   
    ret = ret && mapForPlanning->isTraversable( xTest, yTest ); 
    if(!ret){
      return false;
    }
    robotPoses.push_back(Pose6DOF(xTest, yTest));
       
  }

  // collision checking with other robots
  // TODO: repair this
  /*
  while(!robotPoses.empty()){
    CellPoint tmpPoint = robotPoses.back();
    robotPoses.pop_back();
    for(int i=0; i<robotPoses.size(); i++){ // check with others
      if(tmpPoint.distToPoint(&(robotPoses.at(i)))<robotMaxCellDist){
        return false;
      }
    }
  }*/
  return ret;
}

bool map25Dplanner::isStateValidOneRobotOnMap(const ob::State *state) const{
	const double xTest = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    const double yTest = state->as<ob::RealVectorStateSpace::StateType>()->values[1];  
    //printf("xt: %f yt: %f\n", xTest, yTest); 
    return mapForPlanning->isTraversable( xTest, yTest ); 
}


bool map25Dplanner::isStateValidOneRobot(const ob::State *state) const{
	const double xTest = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    const double yTest = state->as<ob::RealVectorStateSpace::StateType>()->values[1];   
    return mapForPlanning->isNotOccupied( xTest, yTest ); 
}


bool map25Dplanner::fillPathBySolution(std::vector<std::vector<Pose6DOF>*>* pathToFill){
  bool ret = true;
  // fill path by the solution
  if (!ss_ || !ss_->haveSolutionPath()){
    return false;
  }
  og::PathGeometric &p = ss_->getSolutionPath();
  
  // check if there is enough paths in the vector, and clear them 
  // printf("Planner: path count: %ld, robot count: %d\n", pathToFill->size(), robotCount);
  if(pathToFill->size()==robotCount){
    //clear all the input vectors
    for(int i=0; i<robotCount; i++){
      pathToFill->at(i)->clear();
    }

    // fill all the vectors by the solution
    for (std::size_t i = 0; i < p.getStateCount(); ++i){// for poses
      for(int j=0; j<robotCount; j++){ // for robots  // TODO: there is mistake with the minimum
        const double xCoor = p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2*j];
        const double yCoor = p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2*j+1];
        pathToFill->at(j)->push_back( Pose6DOF( xCoor, yCoor, (float)0.3) );
      }
    }
  }else{
    ret = false;
    printf("ERROR->Plane planner: Robot count and vector sizes are different.\n");
  }
  return ret;
}


