#ifndef CLOUD_OPERATIONS_H
#define CLOUD_OPERATIONS_H

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense> // special eigen transforms
#include "timeMeasure.h"
#include "quadMap.h"
#include "quadMapSupport.h"
#include <random> // random C11
#include <cmath>

// occ map for debug
#include <nav_msgs/OccupancyGrid.h>

#define MAX_TRAVERSABLE_MARGIN_REPLACE (float) 0.1

class SensorPreprocModel{
	public:
		// mapping parameters
		int pointSkipStep;
		bool transToRobCoord;
		float scanXMin; 
		float scanXMax;
		float scanYMin; 
		float scanYMax;
		float scanZMin; 
		float scanZMax; 
		
		SensorPreprocModel(){ pointSkipStep=1; transToRobCoord=false; scanZMin=-100; scanZMax=100; scanXMin=-100; scanXMax=100; scanYMin=-100; scanYMax=100; };
		void setPointSkipStep( int _pointSkipStep){ pointSkipStep=_pointSkipStep; };
		void setXcuts( float _scanXMin, float _scanXMax){ scanXMin=_scanXMin, scanXMax=_scanXMax; };
		void setYcuts( float _scanYMin, float _scanYMax){ scanYMin=_scanYMin, scanYMax=_scanYMax; };
		void setZcuts( float _scanZMin, float _scanZMax){ scanZMin=_scanZMin, scanZMax=_scanZMax; };
		
		void setTransToRobCoord( bool _transToRobCoord){ transToRobCoord=_transToRobCoord; };
		
		int getPointSkipStep(){ return pointSkipStep; };
};


class CloudOperations{

  public:
    // preprocessing
    int static downsampleCloud(Eigen::MatrixXf* matOut, pcl::PointCloud<pcl::PointXYZ>* cloudIn, int skipStep);
    int static downsampleMsg(Eigen::MatrixXf* matOut, const sensor_msgs::PointCloud2::ConstPtr& pcdMsg, SensorPreprocModel* sensParams);
    void static preprocessCloud( pcl::PointCloud<pcl::PointXYZ>* cloudIn, const sensor_msgs::PointCloud2::ConstPtr& pcdMsg, SensorPreprocModel* sensParams);
    
    
  	// pcl - ROS conversions (these are PCL wrappers):
  	void static coloredCloudToROSmsg( pcl::PointCloud<pcl::PointXYZRGB>* cloudIn, sensor_msgs::PointCloud2* rosMSG );
 	void static cloudToROSmsg( pcl::PointCloud<pcl::PointXYZ>* cloudIn, sensor_msgs::PointCloud2* rosMSG );
  	
  	
  	// visualization -> RVIZ
  	void static fillCloudByElevMap( Quad25Dmap::QuadMap* map, pcl::PointCloud<pcl::PointXYZ>* cloud, std::string _frame_id );
    void static fillColoredCloudByElevMap( Quad25Dmap::QuadMap* map, pcl::PointCloud<pcl::PointXYZRGB>* cloud, std::string _frame_id, int maxSampleLevel );
  	void static fillColoredCloudByLocalElevMap( Quad25Dmap::QuadMap* map, pcl::PointCloud<pcl::PointXYZRGB>* cloud, std::string _frame_id, Quad25Dmap::MapArea* area );
  	void static fillColoredCloudByGlobElevMap ( Quad25Dmap::QuadMap* map, pcl::PointCloud<pcl::PointXYZRGB>* cloud, std::string _frame_id, Quad25Dmap::MapArea* area, int accuracyMultip );
  	void static showObjectives( pcl::PointCloud<pcl::PointXYZRGB>* cloudOut, Pose6DOF robotPose, std::vector<Pose6DOF> goals, std::vector<Pose6DOF>* objectives, std::string _frame_id);
  	  
  	// occ visualization
  	void static show25DmapAsOcc( ros::Publisher* pub, Quad25Dmap::QuadMap* map, int sampleLevel, std::string _frame_id);  
  	  
  	// support matrix operations for eigen
	pcl::PointXYZRGB static createColorPoint( float xCoord, float yCoord, float zCoord, Quad25Dmap::CellType type, bool isShadow);
  	void static composeSE3transform( Eigen::Matrix4f* outputSE3matrix, Eigen::Matrix3f* rot, Eigen::Vector3f* trans );
  	
};








#endif
