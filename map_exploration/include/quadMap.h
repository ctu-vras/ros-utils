#ifndef QUAD_MAP_H
#define QUAD_MAP_H

#include "quadNode.h"
#include "quadMapSupport.h"
#include <vector>	// update fields

#define UNKNOWN_HEIGHT -1000 
// frontiers have higher margin than this
#define FRONTIER_MARGIN 800 
#define NO_FRONTIER_MARGIN 750
#define DRAW_MIN_HEIGHT -5

namespace Quad25Dmap{

class QuadMap{
	private:
		// metric measures:
		FieldSizes sizes;
		
		//graph props
		float heightTolerance; 	// this defines, if we have to divide, or if the height is already rep.
				
		Quad25Dmap::QuadNode *root;
		int nodeCount;
				
		// height update constants (for weighted average) - but coefs are divided by 2 at advance (we save division)
		float hiGrowCoef= 1; 		// this should not be higher than 1 -> uninit height might cause some problems
		float loGrowCoef = 0.1;
		float maxTraversableMargin;	
			
		// spec internal functions that affect the graph consistency
		void updateUpperNodeHeights(QuadNode* node);
		QuadNode* getLeaf(QuadNode* node, int* level, float _x, float _y);
				
		// graph update fields
		NodeSimpleList upDateHeap;
		
		// geom margin computation
		void computeGeomMargin(QuadNode* midNode, float spread);
		void computeGeomMarginOpen(QuadNode* midNode, float spread); // frontiers are detected all around the map
		
	public:
		FrontierContainer frontiers;
	
		// constructors
		QuadMap( ){ nodeCount = 0; maxTraversableMargin=0.1; };
		void initMap( float xMin, float yMin, float mapSize, int _levels );
		void setMaxTraversableMargin( float _maxTraversableMargin){ maxTraversableMargin=_maxTraversableMargin; };
				
		// main functionality
		void insert(float x, float y, float z);
		
		bool isNotOccupied(float x, float y);	// true if not occupied or shadowed - for planning with unknown  
		bool isTraversable(float x, float y);	// true if free and not shadowed or frontier - plann only on the detected area
		
		bool isPathTraversable( std::vector<Pose6DOF>* path );
		
		bool isTraversableSamplingBased(float x, float y, float radius); 		// TODO sync this
		bool isPathTraversable( std::vector<Pose6DOF>* path, float radius ); 	// TODO sync this
		
		bool isTraversableMarginBased(float margin){ return (margin<maxTraversableMargin) ? true : false; }; // TODO sync this
		bool isFrontierMarginBased(float margin){ return (margin>FRONTIER_MARGIN) ? true : false; }; // TODO sync this
		bool isFrontier(float x, float y){ return isFrontierMarginBased(getGeomMargin( x, y)); }; // TODO sync this
		
		// shadow methods
		void setShadow(float x, float y, bool shadow);
		void createShadowCircle(float x, float y, float radius, bool shadow );
		void searchCircleForShadowThrow( float x, float y, float areaRadius, float radius);
		void updateShadows( float radius );
		bool isShadow(float x, float y);
		
		// main getters
		float getHeight( float x, float y );		// get map properties
		float getGeomMargin( float x, float y );
		float getHeightLevelBack( float x, float y, int level ); // here it is possible to cut the level
		
		CellType getCellTypeParamBased( float height, float geomMargin );
		CellType getCellType( float x, float y );
		
		// This function enlarges the map in the specific direction - do not use this during the insert phase.
		// (It is designed to be used, when the robot is close to the map edge.)
		// Before using this function consider use of expandMapIfNearEdge function.
		void expandMap(ExpandDirection direction);
		
		// This function enlarges the map, if the pose is closer to the edge, than the edgeLimit defines.
		// returns true, if the map has been expanded. 
		// (Only one expansion can be dane by the call of this func.)
		bool expandMapIfNearEdge( Pose6DOF* pose, float edgeLimit );
		
		// sampling based methods
		void sampleComputeGeomMargin();
		void sampleComputeGeomMarginTactical(); // area pose must be updated separatelly
		
		// update based functions
		void resetUpdateNodes(){ upDateHeap.clear(); };
		void updatedNodesComputeMargins();		
		void updateFrontiers();
		void clusterFrontiers( float clustRadius, int minCellsInCluster);
		
		
		// line drawing functions and interpolation -> for path checking
		void generateLine2D( Pose6DOF* start, Pose6DOF* end, std::vector<Pose6DOF>* pathOut );
		void interpolatePath( std::vector<Pose6DOF>* outPath, std::vector<Pose6DOF>* sourcePath );
		
		
		// simple getters and setters
		FieldSizes* getFieldSizes(){ return &sizes; };
		int getNodeCount(){ return nodeCount; };
		
		// simple support functions - //TODO remove these by cmat or add them into map support
		float absVal(float num);
		int absVal(int num);
		void swapVals( int* val1, int* val2);
		
};

}
#endif
