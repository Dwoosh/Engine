#pragma once

#include "Particle.h"

class Spring {
public:
	Particle* p1;
	Particle* p2;

	float k; //[-n,0] lower k, more loose spring
	float b; //[0,1]
	float restingLength;

	inline Spring(float _k, float _b, float len) : k(_k), b(_b), restingLength(len){}
	Particle* getP1();
	Particle* getP2();
	void setParticles(Particle* pa, Particle* pb);
	void setConstants(float _k, float _b);
	void applyForce(float dt);
};