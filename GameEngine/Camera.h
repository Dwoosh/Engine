#pragma once

#include "matrices.h"
#include "3DGeometry.h"

//basic camera object
class Camera {
protected:
	//projection matrix variables
	float projFov;
	float projAspect;
	float projNear;
	float projFar;
	float projWidth;
	float projHeight;

	mat4 matWorld; //world transform matrix
	mat4 matProj; //projection matrix
	//0 - perspective, 1 - orthographic, 2 - user
	int projMode;
public:
	Camera();
	inline virtual ~Camera(){}
	//getters, setters
	mat4 getWorldMatrix();
	mat4 getViewMatrix();
	mat4 getProjMatrix();
	float getAspect();
	void setWorldMatrix(const mat4& view);
	void setUserProjMatrix(const mat4& proj);
	//helper functions
	bool isOrthographic();
	bool isPerspective();
	bool isOrthonormal();
	void orthonormalize();
	void resize(int width, int height);
	void perspective(float fov, float aspect, float nearPlane, float farPlane);
	void orthographic(float width, float height, float nearPlane, float farPlane);
	Frustum getFrustum();
};

//orbital controllable camera object
class OrbitalCamera : public Camera {
protected:
	vec3 target; //vector pointing to a target for camera to look at
	vec2 panSpeed; //speed of the camera
	float zoomDistance; //how far camera is from target
	vec2 zoomLimit; //x - minimum, y - maximum zoom
	float zoomSpeed;
	vec2 rotationSpeed;
	vec2 yRotationLimit; //x - minimum, y - maximum
	vec2 currentRotation;
	
	float clampAngle(float angle, float min, float max); //keep angle in (0,360) range

public:
	OrbitalCamera();
	inline virtual ~OrbitalCamera(){}

	void setTarget(const vec3& newTarget);
	void setZoom(float zoom);
	void setRotation(const vec2& rotation);

	void rotate(const vec2& deltaRotation, float deltaTime);
	void zoom(float deltaZoom, float deltaTime);
	void pan(const vec2& deltaPan, float deltaTime);
	void update(float deltaTime); //updates world matrix
};