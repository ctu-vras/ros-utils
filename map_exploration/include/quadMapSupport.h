#ifndef QUAD_MAP_SUPPORT_H
#define QUAD_MAP_SUPPORT_H

#include "pose6DOF.h"
#include <algorithm>
#include <list>
#include <vector>
#include <iostream>

namespace Quad25Dmap{

// definition of different cell types
enum CellType { FRONTIER, FREE, OCCUPIED, UNKNOWN };

// This enum is mainly used during the map expansion.
enum ExpandDirection { UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT, NONE };

class FieldSizes{
	private:
		float xMin;
		float yMin;
		float* resolArray;
		int maxLevel;
	public:
		// constructors
		FieldSizes( float _xMin, float _yMin, float _mapSize, int _maxLevel){
			xMin = _xMin;
			yMin = _yMin;
			maxLevel = _maxLevel;

			// compute resolution for all the layers
			resolArray = (float*) malloc( (maxLevel+1)*sizeof(float));
			int ipow = 1;
			for(int i=0; i<maxLevel+1; ++i){
				resolArray[i] = (float)_mapSize/ipow;
				ipow *= 2;
			}
			if(maxLevel<5){
				std::cout << "Warning! Map max level has been set to too low value.\n";
			}
		};
		FieldSizes(){ }; // TODO make this consistent
		
		// getters and setters
		float getXmin(){ return xMin; };
		float getYmin(){ return yMin; };
		float getMapSize(){ return resolArray[0]; };
		float getHighResol(){ return resolArray[maxLevel]; };
		float getResolution( int level ){ return resolArray[level]; };
		int getMaxLevel(){ return maxLevel; };
};

struct FloatPair{
	float x;
	float y;
	FloatPair(float _x, float _y){ x=_x; y=_y; };
};

class FrontierContainer{
	private:
		std::vector<FloatPair> frontiersRaw;
		std::vector<FloatPair> frontiersFiltered;
	public:
		FrontierContainer(){};
		
		// this registers the new frontier
		void addFrontierPoint(QuadNode* node){
			frontiersRaw.push_back( FloatPair(node->getX(), node->getY()) );
		};
		
		void removeFrontier(int index){
			if(frontiersRaw.size() > 0){
				FloatPair last = frontiersRaw.at(frontiersRaw.size()-1);
				frontiersRaw.pop_back();
				if( index < frontiersRaw.size() ){
					frontiersRaw.at(index) = last;
				}
			}
		};		
				
		std::vector<FloatPair>* getFrontiersRaw(){
			return &frontiersRaw;
		};
		
		std::vector<FloatPair>* getfrontiersFiltered(){
			return &frontiersFiltered;
		};
		
		void clear(){frontiersRaw.clear(); };
	
		int frontierCount(){ return frontiersRaw.size(); };
	
		void fillPosesByFrontiers(std::vector<Pose6DOF>* posesOut ){
			for(int i=0; i<frontiersFiltered.size(); ++i){
				posesOut->push_back(Pose6DOF(frontiersFiltered.at(i).x, frontiersFiltered.at(i).y));
			}
		};
};


class MapArea{
// this class can be used to run sampling based methods on a particular area

	private:
		// here are area sizes
		float xSize;
		float ySize;
		float xMidOffset;
		float yMidOffset;
		
		// these poses are updated by the incomming position measurements
		float xMin;	// left down corner
		float yMin;
		
		float xMax;	// right up corner
		float yMax;
	
	public:	
				
		// constructors
		MapArea(){};
		MapArea(float _xSize, float _ySize, float _xMidOffset, float _yMidOffset ){
			xSize = _xSize;
			ySize = _ySize;
			xMidOffset = _xMidOffset;
			yMidOffset = _yMidOffset;
			
			// init limits to something around zero - just to be sure
			xMin = -xMidOffset;
			yMin = -yMidOffset;
			xMax =  xMidOffset;
			yMax =  yMidOffset;
		};
		
		// use this to move the whole area by 
		void moveArea( Pose6DOF* pose ){
			xMin = pose->getX()-xMidOffset;
			yMin = pose->getY()-yMidOffset;
			xMax = xMin+xSize;
			yMax = yMin+ySize;
		};
		
		// getters and setters
		float getXMin(){ return xMin; };
		float getYMin(){ return yMin; };
		float getXMax(){ return xMax; };
		float getYMax(){ return yMax; };
};


class NodeSimpleArray{
	private:
		std::vector<QuadNode*> nodes;
		int validSize;
	public:
		NodeSimpleArray(){nodes.resize(1800); validSize=0; };
		
		bool addNode(QuadNode* newNode){
			bool ret = true;
			// search for the new node
			for(int i=0; i<validSize; i++){
				if(nodes.at(i)==newNode){
					ret = false;
					break;
				}
			}
	
			// node is not in the list -> so add it!
			if(ret){
				this->nodes.at(validSize) = newNode;
				validSize++;
			}

			return ret;
		}
		
		int size(){ return validSize; };
		QuadNode* at(int i){ return nodes.at(i); };
		void clear(){ validSize=0; };
};


struct NodeData{
	QuadNode* ptr;
	float height;
	float geomMargin;
	NodeData(QuadNode* _ptr, float _height, float _geomMargin){ ptr=_ptr; height=_height; geomMargin=_geomMargin; };
};

class NodeSimpleList{
	private:
		std::list<NodeData> nodes;

	public:
		NodeSimpleList(){ };
		
		// return true if the node was allready in the heap
		bool addNode(QuadNode* newNode, float height, float geomMargin){
			// create node data structure
			NodeData newData(newNode, height, geomMargin);
		
			bool inserted = false;
			bool ret = false;
    		for(auto it = nodes.begin(); it!=nodes.end(); it++){
        		if((*it).ptr==newNode){
            		inserted = true;
            		break;
        		}
        		if((*it).ptr>newNode){
            		nodes.insert(it, newData);
            		inserted = true;
            		ret = true;
            		break;
        		}        
    		}
    		if(!inserted){
        		nodes.insert(nodes.end(), newData);
        		ret = true;
    		}
			return ret;
		};
		
		std::list<NodeData>* getListPtr() {return &nodes;};
		int size(){return nodes.size();};
		void clear(){ nodes.clear();};
		
};


// it is unique because of the new node allocation
class NodeUniqueMinHeap{

	private:
		std::vector<QuadNode*> nodes;
		int indexOfNext; // this is just for the further use
		
		std::vector<int> openList;
		int openListValidSize;
			
	public:
		NodeUniqueMinHeap(){ indexOfNext=0; openList.resize(2000); openListValidSize=0;}; // TODO fixed limit
		
		
		bool searchRecursive( QuadNode* newNode, int actIndex){ // DFS - less stack
			bool ret = false;
			if(nodes.at(actIndex)==newNode){
				ret = true;
			}else{
				int preInd = 2*actIndex+1;
				if(preInd<indexOfNext ){ // it is posible to do left search
					if( nodes.at(preInd) <= newNode ){	// it makes sence to do left search
						ret = searchRecursive(newNode, preInd);
					}		
				}
				if(!ret){
					preInd++;
					if(preInd<indexOfNext ){
						if( nodes.at(preInd) <= newNode ){	// it makes sence to do left search
							ret = searchRecursive(newNode, preInd);
						}
					}
				}
			}
			return ret;
		};
		
		// check if the node is allready present
		bool isPresent(QuadNode* newNode){	
		    //printf("start pres search \n");
			bool ret = false;
			if(indexOfNext>0){
				ret = searchRecursive( newNode, 0);
			}
			//printf("end pres search \n");
			return ret;
		};
		
		
		// expand the node to the open list
		void expandNode(int nodeIndex, QuadNode* newNode, std::vector<int>* openList){
			if(nodes.at(nodeIndex) < newNode){ // expand further more
				// left
				int preInd = 2*nodeIndex+1;
				if( preInd<nodes.size() ){
					openList->at(openListValidSize) = preInd;
					openListValidSize++;
				}
				// right
				preInd++;
				if( preInd<nodes.size() ){
					openList->at(openListValidSize) = preInd;
					openListValidSize++;
				}
			}
		
		}
		
		// search heap
		bool isPresentBFS(QuadNode* newNode){
		  	bool ret = false;
		  	if(indexOfNext>0){
				openList.at(openListValidSize) = 0;
				openListValidSize = 1;
				int openInd = 0;
				while(!ret){
					if(openInd>=openListValidSize){ // no valid indexes in open list
						break;
					}
					if(nodes.at(openList.at(openInd)) == newNode){
						ret = true;
					}else{
						expandNode(openList.at(openInd), newNode, &openList);
					}
					openInd++;
				}			
		  	}
			return ret;
		};
		
		// add the new node to the heap
		bool addNode(QuadNode* newNode){
			bool ret = isPresentBFS(newNode);
			if(!ret){
				nodes.push_back(newNode);
				indexOfNext++;
			
				//check heap consistency
				bool heapValid = false;
				int checkIndex = indexOfNext-1;
				
				while(!heapValid){
					int upperNodeInd = (checkIndex%2==0) ? checkIndex-2 : checkIndex-1;
					upperNodeInd = upperNodeInd/2;
				
					if(upperNodeInd>=0){ // new index is valid
						if( nodes.at(checkIndex) > nodes.at(upperNodeInd) ){
							heapValid = true;
					 	}else{
					 		//switch nodes
					 		QuadNode* tmp = nodes.at(checkIndex);
					 		nodes.at(checkIndex) = nodes.at(upperNodeInd);
					 		nodes.at(upperNodeInd) = tmp;
					 		checkIndex = upperNodeInd;
					 	}
					}else{
						heapValid = true;
					}
				}
			}
			return ret;
		};
		int size(){ return nodes.size(); };
		QuadNode* at(int i){ return nodes.at(i); };
		void clear(){ nodes.clear(); indexOfNext=0; openListValidSize=0; };
		
};

}//namespace
#endif



