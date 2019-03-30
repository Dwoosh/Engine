#pragma once

#include "Spring.h"
#include <vector>

class Cloth {
protected:
	std::vector<Particle> verts;
	std::vector<Spring> structural; //horizontal and vertical between each particle
	std::vector<Spring> shear; //diagonal between particles
	std::vector<Spring> bend; //horizontal and vertical between not neighbouring particles
	float clothSize;
	
public:
	void initialize(int gridSize, float distance, const vec3& position);
	void setStructuralSprings(float k, float b);
	void setShearSprings(float k, float b);
	void setBendSprings(float k, float b);
	void setParticleMass(float mass);

	void applyForces();
	void update(float dt);
	void applySpringForces(float dt);
	void solveConstraints(const std::vector<OBB>& constraints);

	void render();
};
