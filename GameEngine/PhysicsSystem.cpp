#include "stdafx.h"
#include "PhysicsSystem.h"
#include "FixedFunctionPrimitives.h"
#include "glad\glad.h"
#include "RigidbodyVolume.h"

PhysicsSystem::PhysicsSystem() {
	linearProjectionPercent = 0.45f;
	penetrationSlack = 0.01f;
	impulseIteration = 5;
	colliders1.reserve(100);
	colliders2.reserve(100);
	results.reserve(100);
}


void PhysicsSystem::update(float deltaTime) {
	//clear from previous frame
	colliders1.clear();
	colliders2.clear();
	results.clear();
	//find pairs of colliding bodies
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		for (int j = i; j < size; ++j) {
			if (i == j) { continue; }
			CollisionManifold result;
			resetCollisionManifold(&result);
			if (bodies[i]->hasVolume() && bodies[j]->hasVolume()) {
				RigidbodyVolume* m1 = (RigidbodyVolume*)bodies[i];
				RigidbodyVolume* m2 = (RigidbodyVolume*)bodies[j];
				result = findCollisionFeatures(*m1, *m2);
			}
			if (result.colliding) { //if colliding, store them
				colliders1.push_back(bodies[i]);
				colliders2.push_back(bodies[j]);
				results.push_back(result);
			}
		}
	}
	//apply all forces
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		bodies[i]->applyForces();
	}
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->applyForces();
	}
	//apply impulses to colliding bodies
	for (int k = 0; k < impulseIteration; ++k) {
		for (int i = 0, size = results.size(); i < size; ++i) {
			int jSize = results[i].contacts.size();
			for (int j = 0; j < jSize; ++j) {
				if (colliders1[i]->hasVolume() && colliders2[i]->hasVolume()) {
					RigidbodyVolume* m1 = (RigidbodyVolume*)colliders1[i];
					RigidbodyVolume* m2 = (RigidbodyVolume*)colliders2[i];
					applyImpulse(*m1, *m2, results[i], j);
				}
			}
		}
	}
	//update positions
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		bodies[i]->update(deltaTime);
	}
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->update(deltaTime);
	}
	//linear projection to fix any sinking issues
	for (int i = 0, size = results.size(); i < size; ++i) {
		if (!colliders1[i]->hasVolume() && !colliders2[i]->hasVolume()) {
			continue;
		}
		RigidbodyVolume* m1 = (RigidbodyVolume*)colliders1[i];
		RigidbodyVolume* m2 = (RigidbodyVolume*)colliders2[i];
		float totalMass = m1->invMass() + m2->invMass();
		if (totalMass == 0.0f) { continue; } //neither will move if mass == 0
		//find correction amount and correct
		float depth = fmaxf(results[i].depth - penetrationSlack, 0.0f);
		float scalar = depth / totalMass;
		vec3 correction = results[i].normal * scalar * linearProjectionPercent;
		m1->position = m1->position - correction * m1->invMass();
		m2->position = m2->position + correction * m2->invMass();
		m1->synchCollisionVolumes();
		m2->synchCollisionVolumes();
	}
	for (int i = 0, size = springs.size(); i < size; ++i) {
		springs[i].applyForce(deltaTime);
	}
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->applySpringForces(deltaTime);
	}
	//solve any constraints
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		bodies[i]->solveConstraints(constraints);
	}
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->solveConstraints(constraints);
	}
}

//debug render function
void PhysicsSystem::render() {
	//colors of objects to render
	static const float rigidbodyDiffuse[]{
		200.0f / 255.0f, 0.0f, 0.0f, 0.0f
	};
	static const float rigidbodyAmbient[]{
		200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f
	};
	static const float constraintDiffuse[]{
		0.0f, 200.0f / 255.0f, 0.0f, 0.0f
	};
	static const float constraintAmbient[]{
		50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f
	};
	static const float zero[]{ 0.0f, 0.0f, 0.0f, 0.0f };
	//set colors for rigidbodies
	glColor3f(rigidbodyDiffuse[0], rigidbodyDiffuse[1], rigidbodyDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, rigidbodyAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, rigidbodyDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	//render all bodies
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		bodies[i]->render();
	}
	//same for constraints
	glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	for (int i = 0; i < constraints.size(); ++i) {
		::Render(constraints[i]);
	}
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->render();
	}
}

void PhysicsSystem::addRigidbody(Rigidbody* body) {
	bodies.push_back(body);
}

void PhysicsSystem::addConstraint(const OBB& constraint) {
	constraints.push_back(constraint);
}

void PhysicsSystem::addSpring(const Spring& spring) {
	springs.push_back(spring);
}

void PhysicsSystem::addCloth(Cloth* cloth) {
	cloths.push_back(cloth);
}

void PhysicsSystem::clearRigidbodies() {
	bodies.clear();
}

void PhysicsSystem::clearConstraints() {
	constraints.clear();
}

void PhysicsSystem::clearSprings() {
	springs.clear();
}

void PhysicsSystem::clearCloths() {
	cloths.clear();
}