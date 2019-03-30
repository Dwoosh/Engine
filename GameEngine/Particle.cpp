#include "stdafx.h"
#include "Particle.h"
#include "3DGeometry.h"
#include "FixedFunctionPrimitives.h"

Particle::Particle() {
	type = RIGIDBODY_TYPE_PARTICLE;
	mass = 1.0f;
	bounce = 0.7f;
	friction = 0.95f;
	gravity = vec3(0.0f, -9.82f, 0.0f);
}

void Particle::update(float deltaTime) {
	//vec3 acceleration = forces * invMass();

	velocity = position - oldPosition;
	oldPosition = position;
	float deltaSq = deltaTime * deltaTime;

	//vec3 oldVelocity = velocity;

	//velocity = velocity * friction + acceleration * deltaTime;
	//verlet integration
	position = position + (velocity * friction + forces * deltaSq);
}

void Particle::render() {
	Sphere visual(position, 0.1f);
	::Render(visual);
}

void Particle::applyForces() {
	forces = gravity * mass;
}

void Particle::solveConstraints(const std::vector<OBB>& constraints) {
	int size = constraints.size();
	for (int i = 0; i < size; ++i) {
		Line travelled(oldPosition, position);
		//if collision
		if (linetest(constraints[i], travelled)) {
			vec3 velocity = position - oldPosition;
			vec3 direction = normalized(velocity);
			Ray ray(oldPosition, direction);
			RaycastResult result;
			if (raycast(constraints[i], ray, &result)) {
				//move a little bit above collision point
				position = result.point + result.normal * 0.003f;
				//parallel and perpendicular components of velocity
				vec3 vn = result.normal * dot(result.normal, velocity);
				vec3 vt = velocity - vn;
				oldPosition = position - (vt - vn * bounce);
				//velocity = vt - vn * bounce;
				//break;
			}
		}
	}
}

void Particle::setPosition(const vec3& pos) {
	position = oldPosition = pos;
}

vec3 Particle::getPosition() {
	return position;
}

void Particle::setBounce(float b) {
	bounce = b;
}

float Particle::getBounce() {
	return bounce;
}


void Particle::addImpulse(const vec3& impulse){
	velocity = velocity + impulse;
}

float Particle::invMass(){
	if(mass == 0.0f){ return 0.0f;}
	return 1.0f / mass;
}

void Particle::setMass(float m){
	if(m < 0){m = 0;}
	mass = m;
}

vec3 Particle::getVelocity(){
	return velocity;
}

void Particle::setFriction(float f){
	if(f < 0){f = 0;}
	friction = f;
}

