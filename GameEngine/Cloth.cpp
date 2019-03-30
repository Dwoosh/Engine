#include "Cloth.h"
#include "glad\glad.h"
#include "FixedFunctionPrimitives.h"

void Cloth::initialize(int gridSize, float distance, const vec3& position) {
	float k = -1.0f;
	float b = 0.0f;
	clothSize = gridSize;
	verts.clear();
	structural.clear();
	shear.clear();
	bend.clear();
	verts.resize(gridSize * gridSize);
	float half = (float)(gridSize - 1)*0.5f;
	if (gridSize < 3) {
		gridSize = 3;
	}
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;
			//find world space position of this particle and set it
			float x_pos = ((float)x + position.x - half) * distance;
			float z_pos = ((float)z + position.z - half) * distance;
			verts[i].setPosition(vec3(x_pos, position.y, z_pos));
			verts[i].setMass(1.0f);
			verts[i].setBounce(0.0f);
			verts[i].setFriction(0.9f);
		}
	}
	//create horizontal structural springs
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize-1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + x;
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			structural.push_back(spring);
		}
	}
	//create vertical structural springs
	for (int x = 0; x < gridSize - 1; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;
			int j = z * gridSize + (x + 1);
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			structural.push_back(spring);
		}
	}
	//create shear springs
	for (int x = 0; x < gridSize - 1; ++x) {
		for (int z = 0; z < gridSize - 1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + (x + 1);
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			shear.push_back(spring);
		}
	}
	for (int x = 1; x < gridSize; ++x) {
		for (int z = 0; z < gridSize - 1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + (x - 1);
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			shear.push_back(spring);
		}
	}
	//create bend springs
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize - 2; ++z) {
			int i = z * gridSize + x;
			int j = (z + 2) * gridSize + x;
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			bend.push_back(spring);
		}
	}
	for (int x = 0; x < gridSize - 2; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;
			int j = z * gridSize + (x + 2);
			vec3 iPos = verts[i].getPosition();
			vec3 jPos = verts[j].getPosition();
			float rest = magnitude(iPos - jPos);
			Spring spring(k, b, rest);
			spring.setParticles(&verts[i], &verts[j]);
			bend.push_back(spring);
		}
	}
}

void Cloth::setStructuralSprings(float k, float b) {
	for (int i = 0, size = structural.size(); i < size; ++i) {
		structural[i].setConstants(k, b);
	}
}

void Cloth::setShearSprings(float k, float b) {
	for (int i = 0, size = shear.size(); i < size; ++i) {
		shear[i].setConstants(k, b);
	}
}

void Cloth::setBendSprings(float k, float b) {
	for (int i = 0, size = bend.size(); i < size; ++i) {
		bend[i].setConstants(k, b);
	}
}

void Cloth::setParticleMass(float mass) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].setMass(mass);
	}
}

void Cloth::applyForces() {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].applyForces();
	}
}

void Cloth::update(float dt) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].update(dt);
	}
}

void Cloth::applySpringForces(float dt) {
	for (int i = 0, size = structural.size(); i < size; ++i) {
		structural[i].applyForce(dt);
	}
	for (int i = 0, size = shear.size(); i < size; ++i) {
		shear[i].applyForce(dt);
	}
	for (int i = 0, size = bend.size(); i < size; ++i) {
		bend[i].applyForce(dt);
	}
}

void Cloth::solveConstraints(const std::vector<OBB>& constraints) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].solveConstraints(constraints);
	}
}

void Cloth::render() {
	for (int x = 0; x < clothSize - 1; ++x) {
		for (int z = 0; z < clothSize - 1; ++z) {
			int tl = z * clothSize + x; //top left
			int bl = (z + 1) * clothSize + x; //bottom left
			int tr = z * clothSize + (x + 1); //top right
			int br = (z + 1) * clothSize + (x + 1); //bottom right
			Triangle t1(verts[tl].getPosition(),
						verts[br].getPosition(),
						verts[bl].getPosition());
			Triangle t2(verts[tl].getPosition(),
						verts[tr].getPosition(),
						verts[br].getPosition());
			::Render(t1,true);
			::Render(t2,true);
		}
	}
}
