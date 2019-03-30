#pragma once

#include "Rigidbody.h"

#define GRAVITY_CONST vec3(0.0f, -9.82f, 0.0f)

class RigidbodyVolume : public Rigidbody{
public:
	vec3 position;
	vec3 velocity;
	vec3 forces;
	float mass;
	float cor; //coefficient of restitution (ratio of v after collision / before)
	float friction;
	
	vec3 orientation;
	vec3 angVelocity;
	vec3 torques;

	OBB box;
	Sphere sphere;
	
	inline RigidbodyVolume() : cor(0.5f), mass(1.0f), friction(0.6f) { type = RIGIDBODY_TYPE_BASE; }
	inline RigidbodyVolume(int bodyType) : cor(0.5f), mass(1.0f), friction(0.6f) { type = bodyType; }

	~RigidbodyVolume() {}

	void render();
	void update(float dt);
	void applyForces();
	void synchCollisionVolumes();
	float invMass();
	void addLinearImpulse(const vec3& impulse);

	mat4 invTensor();
	virtual void addRotationalImpulse(const vec3& point, const vec3& impulse);
};

CollisionManifold findCollisionFeatures(RigidbodyVolume& ra, RigidbodyVolume& rb);
void applyImpulse(RigidbodyVolume& A, RigidbodyVolume& B, const CollisionManifold& M, int c);