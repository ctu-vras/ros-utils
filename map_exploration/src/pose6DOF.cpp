#include "pose6DOF.h"


void Pose6DOF::fillPose(float ti, float xi, float yi, float zi, float qxi, float qyi, float qzi, float qwi){
  t =ti;
  pose(0) = xi;
  pose(1) = yi;
  pose(2) = zi;
  orient.x() = qxi;
  orient.y() = qyi;
  orient.z() = qzi;
  orient.w() = qwi;
}

std::string Pose6DOF::toString(){
  std::ostringstream ret;
  ret << t << " " << pose(0)  << " " << pose(1)  << " " << pose(2)  << " " << 
  orient.x() << " " << orient.y()  << " " << orient.z()  << " " << orient.w();
  return ret.str();
}


// return distance to given pose*distance to given pose
double Pose6DOF::distanceTo( Eigen::Vector3f poseTo ) {
  Eigen::Vector3f delta = this->pose - poseTo;
  return sqrt(delta.transpose()*delta);
}

//returns orientation needed to rotate the robot to the certain goal
double Pose6DOF::angleTo( Pose6DOF goal) { 
  double angleOfGoal = atan2( goal.pose(1) - pose(1), goal.pose(0) - pose(0));
  return angleOfGoal - this->getYaw();
}

double Pose6DOF::getYaw(){
	Eigen::Matrix3f YawRot = orient.toRotationMatrix();
	Eigen::Vector3f pointY(1,0,0);
	pointY = YawRot*pointY;
	return atan2(pointY(1), pointY(0));
}

// ------------ getters and setters ------------
Eigen::Vector4f Pose6DOF::getTXZY(){
	Eigen::Vector4f ret(t, pose(0), pose(1), pose(2));
	return ret;
}

Eigen::Quaternionf Pose6DOF::getOrientation(){
	return orient;
}



