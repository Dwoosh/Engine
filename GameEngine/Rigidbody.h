#pragma once

#include <vector>
#include "3DGeometry.h"

#define RIGIDBODY_TYPE_BASE     0
#define RIGIDBODY_TYPE_PARTICLE 1
#define RIGIDBODY_TYPE_SPHERE   2
#define RIGIDBODY_TYPE_BOX      3

class Rigidbody {
public:
	int type;

	inline Rigidbody() { type = RIGIDBODY_TYPE_BASE; }
	virtual ~Rigidbody(){}

	inline bool hasVolume() { return type == RIGIDBODY_TYPE_SPHERE || type == RIGIDBODY_TYPE_BOX; }

	virtual void update(float deltaTime){}
	virtual void render() {}
	virtual void applyForces(){}
	virtual void solveConstraints(const std::vector<OBB>& constraints){}
};