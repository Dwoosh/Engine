#include "stdafx.h"
#include "vectors.h"
#include <cmath>
#include <cfloat>

//macro definition of float numbers comparing function

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))

//vector operations definitions

//2-dimensional vector - vector sum
vec2 operator+(const vec2& l, const vec2& r) {
	return { l.x + r.x, l.y + r.y };
}

//3-dimensional vector - vector sum
vec3 operator+(const vec3& l, const vec3& r) {
	return { l.x + r.x, l.y + r.y, l.z + r.z };
}

//2-dimensional vector - vector difference
vec2 operator-(const vec2& l, const vec2& r) {
	return { l.x - r.x, l.y - r.y };
}

//3-dimensional vector - vector difference
vec3 operator-(const vec3& l, const vec3& r) {
	return { l.x - r.x, l.y - r.y, l.z - r.z };
}

//2-dimensional vector - vector multiplication
vec2 operator*(const vec2& l, const vec2& r) {
	return { l.x * r.x, l.y * r.y };
}

//3-dimensional vector - vector multiplication
vec3 operator*(const vec3& l, const vec3& r) {
	return { l.x * r.x, l.y * r.y, l.z * r.z };
}

//2-dimensional vector - scalar multiplication
vec2 operator*(const vec2& l, float r) {
	return { l.x * r, l.y * r };
}

//3-dimensional vector - scalar multiplication
vec3 operator*(const vec3& l, float r) {
	return { l.x * r, l.y * r, l.z * r };
}

//2-dimensional vector - vector comparison
bool operator==(const vec2& l, const vec2& r) {
	return CMP(l.x, r.x) && CMP(l.y, r.y);
}

//3-dimensional vector - vector comparison
bool operator==(const vec3& l, const vec3& r) {
	return CMP(l.x, r.x) && CMP(l.y, r.y) && CMP(l.z, r.z);
}

//2-dimensional vector - vector distinction
bool operator!=(const vec2& l, const vec2& r) {
	return !(l == r);
}

//3-dimensional vector - vector distinction
bool operator!=(const vec3& l, const vec3& r) {
	return !(l == r);
}

//Computes dot product of two 2-dimensional vectors
float dot(const vec2& l, const vec2& r) {
	return l.x * r.x + l.y * r.y;
}

//Computes dot product of two 3-dimensional vectors
float dot(const vec3& l, const vec3& r) {
	return l.x * r.x + l.y * r.y + l.z * r.z;
}

//Computes magnitude of 2-dimensional vector
float magnitude(const vec2& v) {
	return sqrtf(dot(v, v));
}

//Computes magnitude of 3-dimensional vector
float magnitude(const vec3& v) {
	return sqrtf(dot(v, v));
}

//Computes squared magnitude of 2-dimensional vector
float magnitudeSq(const vec2& v) {
	return dot(v, v);
}

//Computes squared magnitude of 2-dimensional vector
float magnitudeSq(const vec3& v) {
	return dot(v, v);
}

//Normalizes 2-dimensional vector
void normalize(vec2& v) {
	v = v * (1.0f / magnitude(v));
}

//Normalizes 3-dimensional vector
void normalize(vec3& v) {
	v = v * (1.0f / magnitude(v));
}

//Normalizes 2-dimensional vector
vec2 normalized(const vec2& v) {
	return  v * (1.0f / magnitude(v));
}

//Normalizes 3-dimensional vector
vec3 normalized(const vec3& v) {
	return  v * (1.0f / magnitude(v));
}

//Computes cross product of two vectors
vec3 cross(const vec3& l, const vec3& r) {
	return { l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z, l.x * r.y - l.y * r.x };
}

//Computes angle between two 2-dimensional vectors
float angle(const vec2& l, const vec2& r) {
	float magnit = sqrtf(magnitudeSq(l) * magnitudeSq(r));
	return acos(dot(l, r) / magnit);
}

//Computes angle between two 3-dimensional vectors
float angle(const vec3& l, const vec3& r) {
	float magnit = sqrtf(magnitudeSq(l) * magnitudeSq(r));
	return acos(dot(l, r) / magnit);
}

//Computes 2-dimensional vector projected onto another
vec2 project(const vec2& length, const vec2& direction) {
	float dt = dot(length, direction);
	float magSq = magnitudeSq(direction);
	return direction * (dt / magSq);
}

//Computes 3-dimensional vector projected onto another
vec3 project(const vec3& length, const vec3& direction) {
	float dt = dot(length, direction);
	float magSq = magnitudeSq(direction);
	return direction * (dt / magSq);
}

//Computes 2-dimensional vector perpendicular from one vector to another
vec2 perpendicular(const vec2& length, const vec2& direction) {
	return length - project(length, direction);
}

//Computes 3-dimensional vector perpendicular from one vector to another
vec3 perpendicular(const vec3& length, const vec3& direction) {
	return length - project(length, direction);
}

//Computes 2-dimensional reflected vector
vec2 reflection(const vec2& vector, const vec2& normal) {
	float dt = dot(vector, normal);
	return vector - normal * (dt * 2.0f);
}

//Computes 3-dimensional reflected vector
vec3 reflection(const vec3& vector, const vec3& normal) {
	float dt = dot(vector, normal);
	return vector - normal * (dt * 2.0f);
}