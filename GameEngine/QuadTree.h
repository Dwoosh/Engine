#pragma once

#include "2DGeometry.h"
#include <vector>

using std::vector;

//structure containing quadtree data
struct QuadTreeData {
	void* object;	//pointer to any object in area
	Rectangle2D bounds;	//bounds of object
	bool flag;	//multipurpose flag

	inline QuadTreeData(void* o, const Rectangle2D& rect) 
		: object(o), bounds(rect), flag(false){}
};

//class representing quadtree
class QuadTreeNode {
protected:
	vector<QuadTreeNode> children;	//children nodes of a tree
	vector<QuadTreeData*> contents;	//objects in node area
	int currentDepth;	//depth of a node
	static int maxDepth;	//max depth of a tree
	static int maxObjPerNode;	//max objects in node
	Rectangle2D nodeBounds;	//bounding area of node
public:
	inline QuadTreeNode(const Rectangle2D& rect) : nodeBounds(rect), currentDepth(0){}
	bool isLeaf();	//checks if node is a leaf
	int getObjNum();	//returns object count
	void insert(QuadTreeData& data);	//inserts object data
	void remove(QuadTreeData& data);	//removes object data
	void update(QuadTreeData& data);	//updates object data
	void shake();	//removes leaf nodes
	void split();	//splits node and its data into children
	void resetFlag();	//resets multipurpose flag in tree
	vector<QuadTreeData*> query(const Rectangle2D& area); //returns all data in given area
};

typedef QuadTreeNode QuadTree;