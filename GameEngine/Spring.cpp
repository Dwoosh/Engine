#include "Spring.h"

Particle* Spring::getP1() {
	return p1;
}

Particle* Spring::getP2() {
	return p2;
}

void Spring::setParticles(Particle* pa, Particle* pb) {
	p1 = pa;
	p2 = pb;
}

void Spring::setConstants(float _k, float _b) {
	k = _k;
	b = _b;
}

void Spring::applyForce(float dt) {
	vec3 relativePos = p2->getPosition() - p1->getPosition();
	vec3 relativeVel = p2->getVelocity() - p1->getVelocity();

	float x = magnitude(relativePos) - restingLength;
	float v = magnitude(relativeVel);

	float F = (-k * x) + (-b * v);
	vec3 impulse = normalized(relativePos) * F;
	p1->addImpulse(impulse * p1->invMass());
	p2->addImpulse(impulse * -1.0f * p2->invMass());
}
