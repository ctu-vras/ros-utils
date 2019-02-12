#ifndef POSE_6DOF_H
#define POSE_6DOF_H

#include <string>
#include <sstream>
#include <eigen3/Eigen/Dense>

class Pose6DOF{
  private:
    float t;   	// time
    Eigen::Vector3f pose; 		// [x, y, z]
    Eigen::Quaternionf orient;	// [ quat ]
    
  public:
  	Pose6DOF(){ t=0; pose = Eigen::Vector3f::Zero(); orient.x()=0; orient.y()=0; orient.z()=0; orient.w()=1; };
  	Pose6DOF( Eigen::Vector3f poseIn, Eigen::Quaternionf orientIn ){ pose=poseIn; orient=orientIn; };
  	Pose6DOF( float x, float y){ pose(0)=x; pose(1)=y; pose(2)=0; }; // 2D point init
  	Pose6DOF( float x, float y, float z){ pose(0)=x; pose(1)=y; pose(2)=z; };
  	Pose6DOF( Pose6DOF *posePtr){ this->t=posePtr->t; this->pose=posePtr->pose; this->orient=posePtr->orient; };
    void fillPose(float ti, float xi, float yi, float zi, float qxi, float qyi, float qzi, float qwi);
    
    // basic functions
    double distanceTo( Eigen::Vector3f poseTo );
    double distanceTo( Pose6DOF poseTo ){ return distanceTo( poseTo.pose); };
    double angleTo( Pose6DOF goal);
    
    std::string toString();
    
    //getters and setters
    float getX(){return pose(0); };
    float getY(){return pose(1); };
    float getZ(){return pose(2); };
    
    void setX(float _x){ pose(0)=_x; };
    void setY(float _y){ pose(1)=_y; };
    void setZ(float _z){ pose(2)=_z; };
    
    float getQX(){return orient.x(); };
    float getQY(){return orient.y(); };
    float getQZ(){return orient.z(); };
    float getQW(){return orient.w(); };
    
    double getYaw();
    
    float getTime(){return t; };
    
    Eigen::Vector4f getTXZY();
    Eigen::Quaternionf getOrientation();
    
    void setPose(Eigen::Vector3f _pose){ pose=_pose; };
    void setOrientation(Eigen::Quaternionf _orient){ orient=_orient; };
    
};


#endif
