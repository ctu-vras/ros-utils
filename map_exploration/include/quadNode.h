#ifndef QUAD_NODE_H
#define QUAD_NODE_H

#define CHILD_COUNT 4
#define BASE_GEOM_MARGIN 10

// child-schema
//  
//  ^
//  | y
//	 _______
//  | 0 | 1 |
//  |-------|
//  | 2 | 3 |
//  ------------> x

namespace Quad25Dmap{
class QuadNode{
	private:
		
		float x;
		float y;
		QuadNode *child[CHILD_COUNT];
		bool shadow; // this contains information, if it is too close to the obstackle
		
	public:
		QuadNode *parent;
		float height;
		float geomMargin;
		
		// some functions:
		QuadNode(QuadNode* _parent, float _height, float _x, float _y);
		bool hasChildren();
		
		void expandNode( float nextLevelResolution );
		
		// Search the whole subtree and delete all the nodes, than replace them the one replacer;
		void replaceChild( QuadNode* replacer, int childIndex ); //TODO: add error throw, if index is not valid
		
		void deleteNodeRecursive(QuadNode* node);
		
		QuadNode* getCloserChild(float _x, float _y); 
		
		float absVal(float num);
		
		bool mergeHeightFromChildren(float heightTolerance);
		// getters and setters
		float getX(){ return x; };
		float getY(){ return y; };
		float getGeomMargin(){ return geomMargin; };
		void setShadow(bool sh){ shadow=sh; };
		bool getShadow(){ return shadow; };
};
}
#endif
