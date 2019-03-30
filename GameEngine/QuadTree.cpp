#include "stdafx.h"
#include "QuadTree.h"
#include <queue>

//max depth of a tree
int QuadTreeNode::maxDepth = 5;

//max objects in node
int QuadTreeNode::maxObjPerNode = 15;

//checks if node is a leaf
bool QuadTreeNode::isLeaf() {
	return children.size() == 0;
}

//returns object count
int QuadTreeNode::getObjNum() {
	resetFlag();
	int count = contents.size();
	for (int i = 0, size = contents.size(); i < size; ++i) {
		contents[i]->flag = true;
	}
	std::queue<QuadTreeNode*> process;
	process.push(this);
	while (process.size() > 0) {
		QuadTreeNode* processing = process.back();
		if (!processing->isLeaf()) {
			for (int i = 0, size = processing->children.size(); i < size; ++i) {
				process.push(&processing->children[i]);
			}
		}
		else {
			for (int i = 0, size = processing->contents.size(); i < size; ++i) {
				if (!processing->contents[i]->flag) {
					count += 1;
					processing->contents[i]->flag = true;
				}
			}
		}
		process.pop();
	}
	resetFlag();
	return count;
}

//inserts object data
void QuadTreeNode::insert(QuadTreeData& data) {
	if (!rectRectCol(data.bounds, nodeBounds)) { return; }
	if (isLeaf() && contents.size() + 1 > maxObjPerNode) split();
	if (isLeaf()) contents.push_back(&data);
	else {
		for (int i = 0, size = children.size(); i < size; ++i) {
			children[i].insert(data);
		}
	}
}

//removes object data
void QuadTreeNode::remove(QuadTreeData& data) {
	if (isLeaf()) {
		int removeID = -1;
		for (int i = 0, size = contents.size(); i < size; ++i) {
			if (contents[i]->object == data.object) {
				removeID = i;
				break;
			}
		}
		if (removeID != -1) {
			contents.erase(contents.begin() + removeID);
		}
	}
	else {
		for (int i = 0, size = children.size(); i < size; ++i) {
			children[i].remove(data);
		}
	}
	shake();
}

//updates object data
void QuadTreeNode::update(QuadTreeData& data) {
	remove(data);
	insert(data);
}

//removes leaf nodes
void QuadTreeNode::shake() {
	if (!isLeaf()) {
		int objNum = getObjNum();
		if (objNum == 0) children.clear();
		else if (objNum < maxObjPerNode) {
			std::queue<QuadTreeNode*> process;
			process.push(this);
			while (process.size() > 0) {
				QuadTreeNode* processing = process.back();
				if (!processing->isLeaf()) {
					for (int i = 0, size = children.size(); i < size; ++i) {
						process.push(&processing->children[i]);
					}
				}
				else {
					contents.insert(contents.end(), 
						processing->contents.begin(), processing->contents.end());
				}
				process.pop();
			}
			children.clear();
		}
	}
}

//splits node and its data into children
void QuadTreeNode::split() {
	if (currentDepth + 1 >= maxDepth) return;
	vec2 min = getMin(nodeBounds);
	vec2 max = getMax(nodeBounds);
	vec2 center = min + ((max - min) * 0.5f);

	Rectangle2D childAreas[] = {
		Rectangle2D(fromMinMax(vec2(min.x,min.y),vec2(center.x,center.y))),
		Rectangle2D(fromMinMax(vec2(center.x,min.y),vec2(max.x,center.y))),
		Rectangle2D(fromMinMax(vec2(center.x,center.y),vec2(max.x,max.y))),
		Rectangle2D(fromMinMax(vec2(min.x,center.y),vec2(center.x,max.y)))
	};

	for (int i = 0; i < 4; ++i) {
		children.push_back(QuadTreeNode(childAreas[i]));
		children[i].currentDepth = currentDepth + 1;
	}
	for (int i = 0, size = contents.size(); i < size; ++i) {
		children[i].insert(*contents[i]);
	}
	contents.clear();
}

//resets multipurpose flag in tree
void QuadTreeNode::resetFlag() {
	if (isLeaf()) {
		for (int i = 0, size = contents.size(); i < size; ++i) {
			contents[i]->flag = false;
		}
	}
	else {
		for (int i = 0, size = children.size(); i < size; ++i) {
			children[i].resetFlag();
		}
	}
}

//returns all data in given area
vector<QuadTreeData*> QuadTreeNode::query(const Rectangle2D& area) {
	vector<QuadTreeData*> result;
	if (!rectRectCol(area, nodeBounds)) return result;
	if (isLeaf()) {
		for (int i = 0, size = contents.size(); i < size; ++i) {
			if (rectRectCol(contents[i]->bounds, area)) result.push_back(contents[i]);
		}
	}
	else {
		for (int i = 0, size = children.size(); i < size; ++i) {
			vector<QuadTreeData*> recurse = children[i].query(area);
			if (recurse.size() > 0) {
				result.insert(result.end(), recurse.begin(), recurse.end());
			}
		}
	}
	return result;
}