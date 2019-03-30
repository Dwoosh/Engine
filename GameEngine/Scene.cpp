#include "stdafx.h"
#include "Scene.h"
#include <algorithm>
#include <stack>
#include <list>

//adds model to track
void Scene::addModel(Model* model) {
	//if model is in the objects vector, dont add it
	if (std::find(objects.begin(), objects.end(), model) != objects.end()) {
		return;
	}
	objects.push_back(model);
}

//removes model from vector
void Scene::removeModel(Model* model){
	objects.erase(std::remove(objects.begin(), objects.end(), model), objects.end());
}

//updates model in vector
void Scene::updateModel(Model* model) {
	removeModel(model);
	addModel(model);
}

//returns all children of given model
std::vector<Model*> Scene::findChildren(const Model* model) {
	std::vector<Model*> result;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		if (objects[i] == 0 || objects[i] == model) {
			continue;	//avoid null, root model and cycles
		}
		//walk up the tree, if there is a model then 
		//this object is child of argument model
		Model* iterator = objects[i]->parent;
		if (iterator != 0) {
			if (iterator == model) {
				result.push_back(objects[i]);
				continue;
			}
			iterator = iterator->parent;
		}
	}
	return result;
}

//raycast against scene and return closest hit
Model* Scene::raycast(const Ray& ray) {
	//if accelerator is present, raycast against it
	if (octree != 0) {
		return ::raycast(octree, ray);
	}
	Model* result = 0;
	float res_t = -1;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		float t = modelRayIntxn(*objects[i], ray);
		if (result == 0 && t >= 0) {
			result = objects[i];
			res_t = t;
		}
		else if (result != 0 && t < res_t) {
			result = objects[i];
			res_t = t;
		}
	}
	return result;
}

//returns all objects that occupy region of sphere
std::vector<Model*> Scene::query(const Sphere& sphere) {
	//if accelerator is present, query it
	if (octree != 0) {
		return ::query(octree, sphere);
	}
	std::vector<Model*> result;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		OBB bounds = getOBB(*objects[i]);
		if (sphereOBBIntxn(sphere, bounds)) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

//returns all objects that occupy region of AABB
std::vector<Model*> Scene::query(const AABB& aabb) {
	//if accelerator is present, query it
	if (octree != 0) {
		return ::query(octree, aabb);
	}
	std::vector<Model*> result;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		OBB bounds = getOBB(*objects[i]);
		if (AABBOBBIntxn(aabb, bounds)) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

//splits tree recursively
void splitTree(OctreeNode* node, int depth) {
	if (depth-- <= 0) return;
	//split octree
	if (node->children == 0) {
		node->children = new OctreeNode[8];

		vec3 cent = node->bounds.center;
		vec3 ext = node->bounds.size * 0.5f;

		node->children[0].bounds = AABB(cent + vec3(-ext.x, +ext.y, -ext.z), ext);
		node->children[1].bounds = AABB(cent + vec3(+ext.x, +ext.y, -ext.z), ext);
		node->children[2].bounds = AABB(cent + vec3(-ext.x, +ext.y, +ext.z), ext);
		node->children[3].bounds = AABB(cent + vec3(+ext.x, +ext.y, +ext.z), ext);
		node->children[4].bounds = AABB(cent + vec3(-ext.x, -ext.y, -ext.z), ext);
		node->children[5].bounds = AABB(cent + vec3(+ext.x, -ext.y, -ext.z), ext);
		node->children[6].bounds = AABB(cent + vec3(-ext.x, -ext.y, +ext.z), ext);
		node->children[7].bounds = AABB(cent + vec3(+ext.x, -ext.y, +ext.z), ext);
	}
	//assign models to each child
	if (node->children != 0 && node->models.size() > 0) {
		for (int i = 0; i < 8; ++i) { //iterate through children
			for (int j = 0, size = node->models.size(); j < size; ++j) {
				//check each model in node if it is in child node
				OBB bounds = getOBB(*node->models[j]);
				if (AABBOBBIntxn(node->children[i].bounds, bounds)) {
					node->children[i].models.push_back(node->models[j]);
				}
			}
		}
		node->models.clear();
		for (int i = 0; i < 8; ++i) {
			splitTree(&(node->children[i]), depth); //split recursively
		}
	}
}

//insert model into node
void insert(OctreeNode* node, Model* model){
	OBB bounds = getOBB(*model);
	//if it intersects at all
	if (AABBOBBIntxn(node->bounds, bounds)) {
		//if no children then add to this node
		if (node->children == 0) {
			node->models.push_back(model);
		}
		else {
			//else add to children
			for (int i = 0; i < 8; ++i) {
				insert(&(node->children[i]), model);
			}
		}
	}
}

//remove model from node
void remove(OctreeNode* node, Model* model) {
	//if no children then find model in this node and erase
	if (node->children == 0) {
		std::vector<Model*>::iterator it = 
			std::find(node->models.begin(), node->models.end(), model);
		if (it != node->models.end()) {
			node->models.erase(it);
		}
	}
	else {
		//else remove recursively from children
		for (int i = 0; i < 8; ++i) {
			remove(&(node->children[i]), model);
		}
	}
}

//update model in node
void update(OctreeNode* node, Model* model) {
	remove(node, model);
	insert(node, model);
}

//return closest object in given set to the origin of ray
Model* findClosest(const std::vector<Model*>& set, const Ray& ray) {
	if (set.size() == 0) return 0;
	Model* closest = 0;
	float closest_t = -1;
	for (int i = 0, size = set.size(); i < size; ++i) {
		float t = modelRayIntxn(*set[i], ray);
		if (t < 0) continue;
		if (closest_t < 0 || t < closest_t) {
			closest_t = t;
			closest = set[i];
		}
	}
	return closest;
}

//raycast against octree and return closest hit
Model* raycast(OctreeNode* node, const Ray& ray) {
	float t = raycast(node->bounds, ray);
	//if ray hit bounds of node
	if (t >= 0) {
		//if there are no children, find closest hit model of this node
		if (node->children == 0) {
			return findClosest(node->models, ray);
		}
		//else find closest hit model of children
		else {
			std::vector<Model*> results;
			for (int i = 0; i < 8; ++i) {
				Model* res = raycast(&(node->children[i]), ray);
				if (res != 0) {
					results.push_back(res);
				}
			}
			return findClosest(results, ray);
		}
	}
	return 0;
}

//returns all objects that occupy region of sphere
std::vector<Model*> query(OctreeNode* node, const Sphere& sphere) {
	std::vector<Model*> result;
	//if sphere intersects bounds at all
	if (sphereAABBIntxn(sphere, node->bounds)) {
		//if there are no children
		if (node->children == 0) {
			for (int i = 0, size = node->models.size(); i < size; ++i) {
				//add objects which bounds intersect sphere
				OBB bounds = getOBB(*(node->models[i]));
				if (sphereOBBIntxn(sphere, bounds)) {
					result.push_back(node->models[i]);
				}
			}
		}
		//else query each child
		else {
			for (int i = 0; i < 8; ++i) {
				std::vector<Model*> child = query(&(node->children[i]), sphere);
				if (child.size() > 0) {
					result.insert(result.end(), child.begin(), child.end());
				}
			}
		}
	}
	return result;
}

//returns all objects that occupy region of AABB
std::vector<Model*> query(OctreeNode* node, const AABB& aabb) {
	std::vector<Model*> result;
	if (AABBAABBIntxn(aabb, node->bounds)) {
		//if there are no children
		if (node->children == 0) {
			for (int i = 0, size = node->models.size(); i < size; ++i) {
				//add objects which bounds intersect sphere
				OBB bounds = getOBB(*(node->models[i]));
				if (AABBOBBIntxn(aabb, bounds)) {
					result.push_back(node->models[i]);
				}
			}
		}
		//else query each child
		else {
			for (int i = 0; i < 8; ++i) {
				std::vector<Model*> child = query(&(node->children[i]), aabb);
				if (child.size() > 0) {
					result.insert(result.end(), child.begin(), child.end());
				}
			}
		}
	}
	return result;
}

//add acceleration octree
bool Scene::accelerate(const vec3& position, float size, int depth) {
	if (octree != 0) {
		return false;
	}
	vec3 min(position.x - size, position.y - size, position.z - size);
	vec3 max(position.x + size, position.y + size, position.z + size);
	octree = new OctreeNode();
	octree->bounds = fromMinMax(min, max);
	octree->children = 0;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		octree->models.push_back(objects[i]);
	}
	splitTree(octree, depth);
	return true;
}

Scene& Scene::operator=(const Scene& scene) {
	if (this != &scene) {
		this->octree = scene.octree;
		for (int i = 0, size = scene.objects.size(); i < size; ++i) {
			this->objects.push_back(scene.objects[i]);
		}
	}
	return *this;
}

Scene::Scene(const Scene& scene) {
	octree = scene.octree;
	for (int i = 0, size = scene.objects.size(); i < size; ++i) {
		objects.push_back(scene.objects[i]);
	}
}

//culls objects to only those in frustum
std::vector<Model*> Scene::cull(const Frustum& f) {
	std::vector<Model*> result;
	//if no accelerator loop through every object and check
	if (octree == 0) {
		for (int i = 0; i < objects.size(); ++i) {
			OBB bounds = getOBB(*(objects[i]));
			if (frustumOBBIntxn(f, bounds)) {
				result.push_back(objects[i]);
			}
		}
	}
	else {
		std::list<OctreeNode*> nodes;
		nodes.push_back(octree);
		while (nodes.size() > 0){
			OctreeNode* active = *nodes.begin();
			nodes.pop_front();
			//if it has children check them for culling
			if (active->children != 0) {
				for (int i = 0; i < 8; ++i) {
					AABB bounds = active->children[i].bounds;
					if (frustumAABBIntxn(f, bounds)) {
						nodes.push_back(&active->children[i]);
					}
				}
			}
			//if leaf check contained objects for culling
			else {
				for (int i = 0; i < active->models.size(); ++i) {
					OBB bounds = getOBB(*(active->models[i]));
					if (frustumOBBIntxn(f, bounds)) {
						result.push_back(active->models[i]);
					}
				}
			}
		}
	}
	return result;
}