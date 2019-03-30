#pragma once

#include "Rigidbody.h"

class Particle : public Rigidbody {
	vec3 position, oldPosition;
	vec3 forces;
	float mass, bounce;

	vec3 gravity;
	float friction;

	vec3 velocity;
public:
	Particle();
	void update(float deltaTime);
	void render();
	void applyForces();
	void solveConstraints(const std::vector<OBB>& constraints);

	void setPosition(const vec3& pos);
	vec3 getPosition();
	void setBounce(float b);
	float getBounce();
	void addImpulse(const vec3& impulse);
	float invMass();
	void setMass(float m);
	vec3 getVelocity();
	void setFriction(float f);
};