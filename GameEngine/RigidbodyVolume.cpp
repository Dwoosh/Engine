#include "RigidbodyVolume.h"
#include "FixedFunctionPrimitives.h"
#include "3DGeometry.h"

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))

void RigidbodyVolume::render() {
	synchCollisionVolumes();
	if (type == RIGIDBODY_TYPE_SPHERE) {
		::Render(sphere);
	}
	else if (type == RIGIDBODY_TYPE_BOX) {
		::Render(box);
	}
}

void RigidbodyVolume::update(float dt) {
	const float damping = 0.98f;
	vec3 acceleration = forces * invMass();
	velocity = velocity + acceleration * dt;
	velocity = velocity * damping;
	if (type == RIGIDBODY_TYPE_BOX) {
		vec3 angAcc = multiplyVector(torques, invTensor());
		angVelocity = angVelocity + angAcc * dt;
		angVelocity = angVelocity * damping;
	}
	position = position + velocity * dt;
	if (type == RIGIDBODY_TYPE_BOX) {
		orientation = orientation + angVelocity * dt;
	}
	synchCollisionVolumes();
}

void RigidbodyVolume::applyForces() {
	forces = GRAVITY_CONST * mass;
}

void RigidbodyVolume::synchCollisionVolumes() {
	sphere.center = position;
	box.center = position;
	box.orientation = rotationMat3(
		RAD2DEG(orientation.x),
		RAD2DEG(orientation.y),
		RAD2DEG(orientation.z)
	);
}

float RigidbodyVolume::invMass() {
	if (mass == 0.0f) { return 0.0f; }
	return 1.0f / mass;
}

void RigidbodyVolume::addLinearImpulse(const vec3& impulse) {
	velocity = velocity + impulse;
}

CollisionManifold findCollisionFeatures(RigidbodyVolume& ra, RigidbodyVolume& rb) {
	CollisionManifold result;
	resetCollisionManifold(&result);
	if (ra.type == RIGIDBODY_TYPE_SPHERE) {
		if (rb.type == RIGIDBODY_TYPE_SPHERE) {
			result = findCollisionFeatures(ra.sphere, rb.sphere);
		}
		else if (rb.type == RIGIDBODY_TYPE_BOX) {
			result = findCollisionFeatures(rb.box, ra.sphere);
			result.normal = result.normal * -1.0f;
		}
	}
	else if (ra.type == RIGIDBODY_TYPE_BOX) {
		if (rb.type == RIGIDBODY_TYPE_BOX) {
			result = findCollisionFeatures(ra.box, rb.box);
		}
		else if (rb.type == RIGIDBODY_TYPE_SPHERE) {
			result = findCollisionFeatures(ra.box, rb.sphere);
		}
	}
	return result;
}

void applyImpulse(RigidbodyVolume& A, RigidbodyVolume& B, const CollisionManifold& M, int c) {
	//linear velocity
	float invMass1 = A.invMass();
	float invMass2 = B.invMass();
	float invMassSum = invMass1 + invMass2;
	if (invMassSum == 0.0f) { return; }

	//points of contact relative to center of mass
	vec3 rA = M.contacts[c] - A.position;
	vec3 rB = M.contacts[c] - B.position;
	
	mat4 iA = A.invTensor();
	mat4 iB = B.invTensor();
	//relative velocity
	vec3 relativeVel = (B.velocity + cross(B.angVelocity, rB)) - (A.velocity + cross(A.angVelocity,rA));
	vec3 relativeNorm = M.normal;
	normalize(relativeNorm);
	if (dot(relativeVel, relativeNorm) > 0.0f) { return; } //moving away

	float e = fminf(A.cor, B.cor);
	float numerator = (-(1.0f + e) * dot(relativeVel, relativeNorm)); //e+1 because of two objects

	float d1 = invMassSum;
	vec3 d2 = cross(multiplyVector(cross(rA, relativeNorm), iA), rA);
	vec3 d3 = cross(multiplyVector(cross(rB, relativeNorm), iB), rB);
	float denominator = d1 + dot(relativeNorm, d2 + d3);

	float j = (denominator == 0.0f) ? 0.0f : numerator / denominator; //magnitude of impulse
	//divide by number of contacts
	if (M.contacts.size() > 0.0f && j != 0.0f) {
		j /= (float)M.contacts.size();
	}

	vec3 impulse = relativeNorm * j;
	A.velocity = A.velocity - impulse * invMass1;
	B.velocity = B.velocity + impulse * invMass2;
	A.angVelocity = A.angVelocity - multiplyVector(cross(rA, impulse), iA);
	B.angVelocity = B.angVelocity + multiplyVector(cross(rB, impulse), iB);

	//friction
	//find vector tangential to collision normal
	vec3 t = relativeVel - (relativeNorm * dot(relativeVel, relativeNorm));
	if (CMP(magnitudeSq(t), 0.0f)) { return; }
	normalize(t);

	numerator = -dot(relativeVel, t);
	d1 = invMassSum;
	d2 = cross(multiplyVector(cross(rA, t), iA), rA);
	d3 = cross(multiplyVector(cross(rB, t), iB), rB);
	denominator = d1 + dot(t, d2 + d3);
	if (denominator == 0.0f) { return; }

	float jt = numerator / denominator; //magnitude of friction
	if (M.contacts.size() > 0.0f && jt != 0.0f) {
		jt /= (float)M.contacts.size();
	}
	if (CMP(jt, 0.0f)) { return; }

	//mag of friction cant be greater or smaller than 
	//mag of impulse scaled by coefficient of friction
	float friction = sqrtf(A.friction * B.friction);
	if (jt > j * friction) {
		jt = j * friction;
	}
	else if (jt < -j * friction) {
		jt = -j * friction;
	}
	vec3 tangentImpulse = t * jt;
	A.velocity = A.velocity - tangentImpulse * invMass1;
	B.velocity = B.velocity + tangentImpulse * invMass2;
	A.angVelocity = A.angVelocity - multiplyVector(cross(rA, tangentImpulse), iA);
	B.angVelocity = B.angVelocity + multiplyVector(cross(rB, tangentImpulse), iB);
}

mat4 RigidbodyVolume::invTensor() {
	float ix = 0.0f;
	float iy = 0.0f;
	float iz = 0.0f;
	float iw = 0.0f;
	if (mass != 0 && type == RIGIDBODY_TYPE_SPHERE) {
		float rSq = sphere.radius * sphere.radius;
		float frac = 2.0f / 5.0f;
		ix = rSq * mass * frac;
		iy = rSq * mass * frac;
		iz = rSq * mass * frac;
		iw = 1.0f;
	}
	else if (mass != 0 && type == RIGIDBODY_TYPE_BOX) {
		vec3 size = box.size * 2.0f;
		float frac = 1.0f / 12.0f;
		float xSq = size.x * size.x;
		float ySq = size.y * size.y;
		float zSq = size.z * size.z;
		ix = (ySq + zSq) * mass * frac;
		iy = (xSq + zSq) * mass * frac;
		iz = (xSq + ySq) * mass * frac;
		iw = 1.0f;
	}
	return inverse(mat4(
		ix, 0, 0, 0,
		0, iy, 0, 0,
		0, 0, iz, 0,
		0, 0, 0, iw));
}

void RigidbodyVolume::addRotationalImpulse(const vec3& point, const vec3& impulse) {
	vec3 centerOfMass = position;
	vec3 torque = cross(point - centerOfMass, impulse);
	vec3 angAcc = multiplyVector(torque, invTensor());
	angVelocity = angVelocity + angAcc;
}