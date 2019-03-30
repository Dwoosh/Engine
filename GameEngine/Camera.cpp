#include "stdafx.h"
#include "Camera.h"
#include <cmath>
#include <cfloat>

//macro definition of float numbers comparing function

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))

//basic camera

Camera::Camera() {
	projFov = 60.0f;
	projAspect = 1.3f;
	projNear = 0.01f;
	projFar = 1000.0f;
	projWidth = 1.0f;
	projHeight = 1.0f;

	matWorld = mat4();
	matProj = projectionPersp(projFov, projAspect, projNear, projFar);
	projMode = 0;
}

mat4 Camera::getWorldMatrix() {
	return matWorld;
}

mat4 Camera::getViewMatrix() {
	if (!isOrthonormal()) {
		orthonormalize();
	}
	//transpose world to inverse rotation since matrix is orthonormal
	mat4 inv = transpose(matWorld);
	inv._41 = inv._14 = 0.0f;
	inv._42 = inv._24 = 0.0f;
	inv._43 = inv._34 = 0.0f;
	//extract vectors
	vec3 right = vec3(matWorld._11, matWorld._12, matWorld._13);
	vec3 up = vec3(matWorld._21, matWorld._22, matWorld._23);
	vec3 forward = vec3(matWorld._31, matWorld._32, matWorld._33);
	vec3 worldPos = vec3(matWorld._41, matWorld._42, matWorld._43);
	//dot product == rotation * position, negative because of inverse
	inv._41 = -dot(right, worldPos);
	inv._42 = -dot(up, worldPos);
	inv._43 = -dot(forward, worldPos);
	return inv;
}

mat4 Camera::getProjMatrix() {
	return matProj;
}

float Camera::getAspect() {
	return projAspect;
}

void Camera::setWorldMatrix(const mat4& view) {
	matWorld = view;
}

void Camera::setUserProjMatrix(const mat4& proj) {
	matProj = proj;
	projMode = 2;
}

bool Camera::isOrthographic() {
	return projMode == 1;
}

bool Camera::isPerspective() {
	return projMode == 0;
}

bool Camera::isOrthonormal() {
	vec3 right = vec3(matWorld._11, matWorld._12, matWorld._13);
	vec3 up = vec3(matWorld._21, matWorld._22, matWorld._23);
	vec3 forward = vec3(matWorld._31, matWorld._32, matWorld._33);
	//if vectors are not of normal length camera is not orthonormal
	if (!CMP(dot(right, right), 1.0f) || !CMP(dot(up, up), 1.0f) ||
		!CMP(dot(forward, forward), 1.0f)) {
		return false;
	}
	//if vectors are not perpendicular
	if (!CMP(dot(forward, up), 0.0f) || !CMP(dot(forward, right), 0.0f) || 
		!CMP(dot(right, up), 0.0f)) {
		return false;
	}
	return true;
}

void Camera::orthonormalize() {
	vec3 right = vec3(matWorld._11, matWorld._12, matWorld._13);
	vec3 up = vec3(matWorld._21, matWorld._22, matWorld._23);
	vec3 forward = vec3(matWorld._31, matWorld._32, matWorld._33);

	vec3 perpForward = normalized(forward);
	vec3 perpRight = normalized(cross(up, perpForward));
	vec3 perpUp = cross(perpForward, perpRight);
	//rebuild world matrix using perpendicular rotation vectors
	matWorld = mat4(perpRight.x, perpRight.y, perpRight.z, 0.0f,
					perpUp.x, perpUp.y, perpUp.z, 0.0f,
					perpForward.x, perpForward.y, perpForward.z,0.0f,
					matWorld._41, matWorld._42, matWorld._43, 1.0f);
}

void Camera::resize(int width, int height) {
	projAspect = (float)width / (float)height;
	if (isPerspective()) {
		matProj = projectionPersp(projFov, projAspect, projNear, projFar);
	}
	else if (isOrthographic()) {
		projWidth = width;
		projHeight = height;
		float halfWid = projWidth * 0.5f;
		float halfHei = projHeight * 0.5f;
		matProj = projectionOrtho(-halfWid, halfWid, -halfHei, halfHei, projNear, projFar);
	}
	//projMode == 2 <=> user defined projection
}

void Camera::perspective(float fov, float aspect, float nearPlane, float farPlane) {
	projFov = fov;
	projAspect = aspect;
	projNear = nearPlane;
	projFar = farPlane;
	matProj = projectionPersp(projFov, projAspect, projNear, projFar);
	projMode = 0;
}

void Camera::orthographic(float width, float height, float nearPlane, float farPlane) {
	projWidth = width;
	projHeight = height;
	projNear = nearPlane;
	projFar = farPlane;
	float halfWid = projWidth * 0.5f;
	float halfHei = projHeight * 0.5f;
	matProj = projectionOrtho(-halfWid, halfWid, -halfHei, halfHei, projNear, projFar);
	projMode = 1;
}

Frustum Camera::getFrustum() {
	Frustum result;
	mat4 vp = getViewMatrix() * getProjMatrix();
	vec3 col1(vp._11, vp._21, vp._31); //represents normals of frustum planes
	vec3 col2(vp._12, vp._22, vp._32);
	vec3 col3(vp._13, vp._23, vp._33);
	vec3 col4(vp._14, vp._24, vp._34); //represents z-axis of camera
	//calculate normals of planes
	result.left.normal = col4 + col1;
	result.right.normal = col4 - col1;
	result.bottom.normal = col4 + col2;
	result.top.normal = col4 - col2;
	//(0,1) z-axis space dx style. +col4 if using (-1,+1) openGL space
	result.near.normal = col3;	
	result.far.normal = col4 - col3;
	//calculate distances of planes
	result.left.distance = vp._44 + vp._41;
	result.right.distance = vp._44 - vp._41;
	result.bottom.distance = vp._44 + vp._42;
	result.top.distance = vp._44 - vp._42;
	result.near.distance = vp._43;
	result.far.distance = vp._44 - vp._43;
	//normalize normals
	for (int i = 0; i < 6; ++i) {
		float m = 1.0f / magnitude(result.planes[i].normal);
		result.planes[i].normal = result.planes[i].normal * m;
		result.planes[i].distance *= m;
	}
	return result;
}

//orbital camera

OrbitalCamera::OrbitalCamera() {
	target = vec3(0, 0, 0);
	zoomDistance = 10.0f;
	zoomSpeed = 200.0f;
	rotationSpeed = vec2(250.0f, 120.0f);
	yRotationLimit = vec2(-20.0f, 80.0f);
	zoomLimit = vec2(3.0f, 15.0f);
	currentRotation = vec2(0, 0);
	panSpeed = vec2(180.0f, 180.0f);
}

//keep angle in (0,360) range
float OrbitalCamera::clampAngle(float angle, float min, float max) {
	while (angle < -360) {
		angle += 360;
	}
	while (angle > 360) {
		angle -= 360;
	}
	if (angle < min) {
		angle = min;
	}
	if (angle > max) {
		angle = max;
	}
	return angle;
}

void OrbitalCamera::rotate(const vec2& deltaRotation, float deltaTime) {
	currentRotation.x += deltaRotation.x * rotationSpeed.x * zoomDistance * deltaTime;
	currentRotation.y += deltaRotation.y * rotationSpeed.y * zoomDistance * deltaTime;
	currentRotation.x = clampAngle(currentRotation.x, -360, 360);
	currentRotation.y = clampAngle(currentRotation.y, yRotationLimit.x, yRotationLimit.y);
}

void OrbitalCamera::zoom(float deltaZoom, float deltaTime) {
	zoomDistance += deltaZoom * zoomSpeed * deltaTime;
	if (zoomDistance < zoomLimit.x) {
		zoomDistance = zoomLimit.x;
	}
	if (zoomDistance > zoomLimit.y) {
		zoomDistance = zoomLimit.y;
	}
}

void OrbitalCamera::pan(const vec2& deltaPan, float deltaTime) {
	vec3 right = vec3(matWorld._11, matWorld._12, matWorld._13);
	float xPan = deltaPan.x * panSpeed.x * deltaTime;
	target = target - (right * xPan);
	float yPan = deltaPan.y * panSpeed.y * deltaTime;
	target = target + (vec3(0, 1, 0) * yPan);
}

//updates world matrix
void OrbitalCamera::update(float deltaTime) {
	vec3 rotation = vec3(currentRotation.y, currentRotation.x, 0);
	mat3 orientation = rotationMat3(rotation.x, rotation.y, rotation.z);
	vec3 direction = multiplyVector(vec3(0.0f, 0.0f, -zoomDistance), orientation);
	vec3 position = direction + target;
	matWorld = inverse(lookAt(position, target, vec3(0, 1, 0)));
}

void OrbitalCamera::setTarget(const vec3& newTarget) {
	target = newTarget;
}

void OrbitalCamera::setZoom(float zoom) {
	zoomDistance = zoom;
}

void OrbitalCamera::setRotation(const vec2& rot) {
	currentRotation = rot;
}