#pragma once

#include "3DGeometry.h"
#include <vector>

//TODO: add other geometry


//octree object
typedef struct OctreeNode {
	AABB bounds; //bounds of this node
	OctreeNode* children; //children nodes
	std::vector<Model*> models; //models to keep track of

	inline OctreeNode() : children(0) {}
	inline ~OctreeNode() {
		if (children != 0) {
			delete[] children;
		}
	}

} OctreeNode;

//scene object
class Scene {
private:
	Scene(const Scene&);
	Scene& operator=(const Scene&);
protected:
	std::vector<Model*> objects; //objects to track
	OctreeNode* octree; //octree accelerator
public:
	inline Scene() : octree(0){}
	inline ~Scene() {
		if (octree != 0) {
			delete octree;
		}
	}

	void addModel(Model* model); //adds model to track
	void removeModel(Model* model); //removes model from vector
	void updateModel(Model* model); //updates model in vector
	std::vector<Model*> findChildren(const Model* model); //returns all children of given model

	Model* raycast(const Ray& ray); //raycast against scene and return closest hit
	std::vector<Model*> query(const Sphere& sphere); //returns all objects that occupy region of sphere
	std::vector<Model*> query(const AABB& aabb); //returns all objects that occupy region of AABB

	bool accelerate(const vec3& position, float size, int depth); //add acceleration octree

	std::vector<Model*> cull(const Frustum& f); //culls objects to only those in frustum
};

//splits tree recursively
void splitTree(OctreeNode* node, int depth);

//insert model into node
void insert(OctreeNode* node, Model* model);

//remove model from node
void remove(OctreeNode* node, Model* model);

//update model in node
void update(OctreeNode* node, Model* model);

//return closest object in given set to the origin of ray
Model* findClosest(const std::vector<Model*>& set, const Ray& ray);

//raycast against octree and return closest hit
Model* raycast(OctreeNode* node, const Ray& ray);

//returns all objects that occupy region of sphere
std::vector<Model*> query(OctreeNode* node, const Sphere& sphere);

//returns all objects that occupy region of AABB
std::vector<Model*> query(OctreeNode* node, const AABB& aabb);