#pragma once

#include "Rigidbody.h"
#include "Spring.h"
#include "Cloth.h"

class PhysicsSystem {
protected:
	std::vector<Rigidbody*> bodies;
	std::vector<OBB> constraints;

	std::vector<Rigidbody*> colliders1;
	std::vector<Rigidbody*> colliders2;
	std::vector<CollisionManifold> results;

	std::vector<Spring> springs;
	std::vector<Cloth*> cloths;
public:
	//how much positional correction to apply
	float linearProjectionPercent; //keep between 0.2 - 0.8
	//how much allow objects to penetrate before collision takes effect
	float penetrationSlack; //keep between 0.01 - 0.1
	//iterations of physics calc
	int impulseIteration; //keep between 1 - 20

	//for debug
	bool DebugRender;
	bool DoLinearProjection;
	bool RenderRandomColors;

	PhysicsSystem();

	void update(float deltaTime);
	void render();

	void addRigidbody(Rigidbody* body);
	void addConstraint(const OBB& constraint);
	void addSpring(const Spring& spring);
	void addCloth(Cloth* cloth);
	void clearRigidbodies();
	void clearConstraints();
	void clearSprings();
	void clearCloths();
};