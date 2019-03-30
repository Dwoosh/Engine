#include "stdafx.h"
#include "3DGeometry.h"
#include "vectors.h"
#include <cmath>
#include <cfloat>
#include <list>

//macro definition of float numbers comparing function

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))


//Calculates length of a line
float length(const Line& line) {
	return magnitude(line.start - line.end);
}

//Calculates squared length of a line
float lengthSq(const Line& line) {
	return magnitudeSq(line.start - line.end);
}

//Creates ray from two given points
Ray fromPoints(const Point& from, const Point& to) {
	return Ray(from, normalized(to - from));
}

//Returns minimum point of AABB
vec3 getMin(const AABB& box) {
	vec3 p1 = box.center + box.size;
	vec3 p2 = box.center - box.size;
	return vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}

//Returns maximum point of AABB
vec3 getMax(const AABB& box) {
	vec3 p1 = box.center + box.size;
	vec3 p2 = box.center - box.size;
	return vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}

//Returns AABB constructed from two points
AABB fromMinMax(const vec3& min, const vec3& max) {
	return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

//Returns result of plane equation
float planeEq(const Point& p, const Plane& plane) {
	return dot(p, plane.normal) - plane.distance;
}

//Checks if point is in sphere
bool pointInSphere(const Point& point, const Sphere& sphere) {
	float magSq = magnitudeSq(point - sphere.center);
	float radSq = sphere.radius * sphere.radius;
	return magSq < radSq;
}

//Returns sphere's closest point to given point
Point closestPoint(const Point& point, const Sphere& sphere) {
	vec3 sphereToPoint = point - sphere.center;
	normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;
	return sphereToPoint + sphere.center;
}

//Checks if point is in AABB
bool pointInAABB(const Point& point, const AABB& aabb) {
	Point min = getMin(aabb);
	Point max = getMax(aabb);
	if (point.x < min.x || point.y < min.y || point.z < min.z) return false;
	if (point.x > max.x || point.y > max.y || point.z > max.z) return false;
	return true;
}

//Returns AABB's closest point to given point
Point closestPoint(const Point& point, const AABB& aabb) {
	Point result = point;
	Point min = getMin(aabb);
	Point max = getMax(aabb);
	result.x = (result.x < min.x) ? min.x : result.x;
	result.y = (result.y < min.y) ? min.y : result.y;
	result.z = (result.z < min.z) ? min.z : result.z;
	result.x = (result.x > max.x) ? max.x : result.x;
	result.y = (result.y > max.y) ? max.y : result.y;
	result.z = (result.z > max.z) ? max.z : result.z;
	return result;
}

//Checks if point is in OBB
bool pointInOBB(const Point& point, const OBB& obb) {
	vec3 dir = point - obb.center;
	for (int i = 0; i < 3; ++i) {	//get every axis
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(orientation[0], orientation[1], orientation[2]);
		float distance = dot(dir, axis);
		//check if distance from OBB center is out of bounds
		if (distance > obb.size.asArray[i]) return false;
		if (distance < -obb.size.asArray[i]) return false;
	}
	return true;
}

//Returns OBB's closest point to given point
Point closestPoint(const Point& point, const OBB& obb) {
	Point result = obb.center;
	vec3 dir = point - obb.center;
	for (int i = 0; i < 3; ++i) {	//get every axis
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(orientation[0], orientation[1], orientation[2]);
		float distance = dot(dir, axis);
		if (distance > obb.size.asArray[i]) distance = obb.size.asArray[i];
		if (distance < -obb.size.asArray[i]) distance = -obb.size.asArray[i];
		result = result + (axis * distance);
	}
	return result;
}

//Checks if point is on plane
bool pointOnPlane(const Point& point, const Plane& plane) {
	float d = dot(point, plane.normal);
	//checks point containment using plane equation
	return CMP(d - plane.distance, 0.0f);
}

//Returns plane's closest point to given point
Point closestPoint(const Point& point, const Plane& plane) {
	float d = dot(plane.normal, point);
	float distance = d - plane.distance;
	return point - plane.normal * distance;
}

//Checks if point is on line
bool pointOnLine(const Point& point, const Line& line) {
	Point closest = closestPoint(point, line);
	float distSq = magnitudeSq(closest - point);
	return CMP(distSq, 0.0f);
}

//Returns line's closest point to given point
Point closestPoint(const Point& point, const Line& line) {
	vec3 lineVec = line.end - line.start;
	//project point onto line and normalize
	float t = dot(point - line.start, lineVec) / dot(lineVec, lineVec);
	//clamp t to [0,1] interval
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);
	return line.start + lineVec * t;
}

//Checks if point is on ray
bool pointOnRay(const Point& point, const Ray& ray) {
	if (point == ray.origin) return true;
	vec3 norm = point - ray.origin;
	normalize(norm); //because rays direction vector is normalized
	float diff = dot(norm, ray.direction); //project onto ray
	return CMP(diff, 1.0f);
}

//Returns ray's closest point to given point
Point closestPoint(const Point& point, const Ray& ray) {
	float t = dot(point - ray.origin, ray.direction); //project onto ray
	t = fmaxf(t, 0.0f); //in case point is behind ray
	return Point(ray.origin + ray.direction * t);
}

//Returns interval of AABB projected onto axis
Interval getInterval(const AABB& aabb, const vec3& axis) {
	vec3 min = getMin(aabb);
	vec3 max = getMax(aabb);
	//get each vertex of AABB
	vec3 vertex[8] = {
		vec3(min.x, max.y, max.z), vec3(min.x, max.y, min.z),
		vec3(min.x, min.y, max.z), vec3(min.x, min.y, min.z),
		vec3(max.x, max.y, max.z), vec3(max.x, max.y, min.z),
		vec3(max.x, min.y, max.z), vec3(max.x, min.y, min.z)
	};
	Interval result;
	result.min = result.max = dot(axis, vertex[0]);
	for (int i = 1; i < 8; ++i) {
		//project all vertices on axis and get minimum and maximum
		float projection = dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}
	return result;
}

//Returns interval of OBB projected onto axis
Interval getInterval(const OBB& obb, const vec3& axis) {
	vec3 vertex[8];
	vec3 cent = obb.center;
	vec3 exten = obb.size;
	//get all axes of OBB
	const float* orient = obb.orientation.asArray;
	vec3 axes[] = { vec3(orient[0],orient[1],orient[2]),
					vec3(orient[3],orient[4],orient[5]),
					vec3(orient[6],orient[7],orient[8])
	};
	//get all vertices of OBB
	vertex[0] = cent + axes[0] * exten[0] + axes[1] * exten[1] + axes[2] * exten[2];
	vertex[1] = cent - axes[0] * exten[0] + axes[1] * exten[1] + axes[2] * exten[2];
	vertex[2] = cent + axes[0] * exten[0] - axes[1] * exten[1] + axes[2] * exten[2];
	vertex[3] = cent + axes[0] * exten[0] + axes[1] * exten[1] - axes[2] * exten[2];
	vertex[4] = cent - axes[0] * exten[0] - axes[1] * exten[1] - axes[2] * exten[2];
	vertex[5] = cent + axes[0] * exten[0] - axes[1] * exten[1] - axes[2] * exten[2];
	vertex[6] = cent - axes[0] * exten[0] + axes[1] * exten[1] - axes[2] * exten[2];
	vertex[7] = cent - axes[0] * exten[0] - axes[1] * exten[1] + axes[2] * exten[2];
	
	Interval result;
	result.min = result.max = dot(axis, vertex[0]);
	for (int i = 1; i < 8; ++i) {
		//project all vertices on axis and get minimum and maximum
		float projection = dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}
	return result;
}

//Checks if two objects overlap on axis
bool overlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis) {
	//project both shapes on axis and check if they overlap	
	Interval a = getInterval(aabb, axis);
	Interval b = getInterval(obb, axis);
	return (b.min <= a.max) && (a.min <= b.max);
}

//Checks if two objects overlap on axis
bool overlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis) {
	//project both shapes on axis and check if they overlap	
	Interval a = getInterval(obb1, axis);
	Interval b = getInterval(obb2, axis);
	return (b.min <= a.max) && (a.min <= b.max);
}

//Checks if two spheres intersect
bool sphereSphereIntxn(const Sphere& s1, const Sphere& s2) {
	float sum = s1.radius + s2.radius;
	float distSq = magnitudeSq(s1.center - s2.center);
	return distSq < sum * sum;
}

//Checks if sphere and AABB intersect
bool sphereAABBIntxn(const Sphere& sphere, const AABB& aabb) {
	Point closest = closestPoint(sphere.center, aabb);
	float distSq = magnitudeSq(sphere.center - closest);
	float radSq = sphere.radius * sphere.radius;
	return distSq < radSq;
}

//Checks if sphere and OBB intersect
bool sphereOBBIntxn(const Sphere& sphere, const OBB& obb) {
	Point closest = closestPoint(sphere.center, obb);
	float distSq = magnitudeSq(sphere.center - closest);
	float radSq = sphere.radius * sphere.radius;
	return distSq < radSq;
}

//Checks if sphere and plane intersect
bool spherePlaneIntxn(const Sphere& sphere, const Plane& plane) {
	Point closest = closestPoint(sphere.center, plane);
	float distSq = magnitudeSq(sphere.center - closest);
	float radSq = sphere.radius * sphere.radius;
	return distSq < radSq;
}

//Checks if two AABBs intersect
bool AABBAABBIntxn(const AABB& aabb1, const AABB& aabb2) {
	Point aMin = getMin(aabb1);
	Point aMax = getMax(aabb1);
	Point bMin = getMin(aabb2);
	Point bMax = getMax(aabb2);
	return (aMin.x <= bMax.x && aMax.x >= bMin.x) 
		&& (aMin.y <= bMax.y && aMax.y >= bMin.y) 
		&& (aMin.z <= bMax.z && aMax.z >= bMax.z);
}

//Checks if AABB and OBB intersect using SAT test
bool AABBOBBIntxn(const AABB& aabb, const OBB& obb) {
	//get all possible axes
	const float* orient = obb.orientation.asArray;
	vec3 axesOfSep[15] = {
		vec3(1,0,0),
		vec3(0,1,0),
		vec3(0,0,1),
		vec3(orient[0],orient[1],orient[2]),
		vec3(orient[3],orient[4],orient[5]),
		vec3(orient[6],orient[7],orient[8])
	};
	for (int i = 0; i < 3; ++i) {
		axesOfSep[6 + i * 3 + 0] = cross(axesOfSep[i], axesOfSep[0]);
		axesOfSep[6 + i * 3 + 1] = cross(axesOfSep[i], axesOfSep[1]);
		axesOfSep[6 + i * 3 + 2] = cross(axesOfSep[i], axesOfSep[2]);
	}
	//check for overlap on each axis 
	for (int i = 0; i < 15; ++i) {
		//axis of separation found
		if (!overlapOnAxis(aabb, obb, axesOfSep[i])) return false;
	}
	//no axis of separation
	return true;
}

//Checks if AABB and plane intersect
bool AABBPlaneIntxn(const AABB& aabb, const Plane& plane) {
	//project AABB's half extents onto plane
	float projLen = aabb.size.x * fabsf(plane.normal.x) 
		+ aabb.size.y * fabsf(plane.normal.y) 
		+ aabb.size.z * fabsf(plane.normal.z);
	//project AABB center onto plane
	float d = dot(plane.normal, aabb.center);
	//get distance of AABB center from plane's origin
	float dist = d - plane.distance;
	//intersection occurs if this distance is lesser than extents projection
	return fabsf(dist) <= projLen;
}

//Checks if two OBBs intersect using SAT test
bool OBBOBBIntxn(const OBB& obb1, const OBB& obb2) {
	//get all axes of separation
	const float* o1 = obb1.orientation.asArray;
	const float* o2 = obb2.orientation.asArray;
	vec3 axesOfSep[] = {
		vec3(o1[0],o1[1],o1[2]),
		vec3(o1[3],o1[4],o1[5]),
		vec3(o1[6],o1[7],o1[8]),
		vec3(o2[0],o2[1],o2[2]),
		vec3(o2[3],o2[4],o2[5]),
		vec3(o2[6],o2[7],o2[8])
	};
	for (int i = 0; i < 3; ++i) {
		axesOfSep[6 + i * 3 + 0] = cross(axesOfSep[i], axesOfSep[0]);
		axesOfSep[6 + i * 3 + 1] = cross(axesOfSep[i], axesOfSep[1]);
		axesOfSep[6 + i * 3 + 2] = cross(axesOfSep[i], axesOfSep[2]);
	}
	//check for overlap on each axis
	for (int i = 0; i < 15; ++i) {
		//axis of separation found
		if (!overlapOnAxis(obb1, obb2, axesOfSep[i])) return false;
	}
	//no axis of separation
	return true;
}

//Checks if OBB and plane intersect
bool OBBPlaneIntxn(const OBB& obb, const Plane& plane) {
	//get OBB's axes
	const float* orient = obb.orientation.asArray;
	vec3 rotAx[] = {
		vec3(orient[0],orient[1],orient[2]),
		vec3(orient[3],orient[4],orient[5]),
		vec3(orient[6],orient[7],orient[8])
	};
	//project extents onto plane
	float projLen = obb.size.x * fabsf(dot(plane.normal, rotAx[0]))
		+ obb.size.y * fabsf(dot(plane.normal, rotAx[1]))
		+ obb.size.z * fabsf(dot(plane.normal, rotAx[2]));
	//project OBB's center onto plane
	float d = dot(plane.normal, obb.center);
	//get distance of OBB's center from plane's origin
	float dist = d - plane.distance;
	return fabsf(dist) <= projLen;
}

//Checks if two planes intersect
bool planePlaneIntxn(const Plane& p1, const Plane& p2) {
	//planes intersect only if they are NOT parallel
	vec3 c = cross(p1.normal, p2.normal);
	//if they are parallel cross product is 0
	return !CMP(dot(c, c), 0);
}

//Raycasts given ray on sphere
float raycast(const Sphere& sphere, const Ray& ray) {
	//vector from ray's origin to center of the sphere
	vec3 dist = sphere.center - ray.origin;
	float radSq = sphere.radius * sphere.radius;
	float distSq = magnitudeSq(dist);
	//project dist vec onto ray
	float proj = dot(dist, ray.direction);
	//squared distance from sphere center to projection on ray
	float sideSq = distSq - (proj * proj);
	//distance from sphere boundary to closest point on ray
	float sideToRad = sqrt(radSq - sideSq);
	//no collision - distance from sphere center to 
	//closest point on ray is greater than radius
	if (radSq - sideSq < 0.0f) return -1;
	//ray starts inside
	else if (distSq < radSq) return proj + sideToRad;
	return proj - sideToRad; //normal intersection
}

//Raycasts given ray on AABB using Cyrus-Beck clipping algorithm
float raycast(const AABB& aabb, const Ray& ray) {
	vec3 min = getMin(aabb);
	vec3 max = getMax(aabb);
	//get intersections against each of the AABB's slabs - x,y,z
	//numbers are distances along ray at which intersection occurs
	float t1 = min.x - ray.origin.x;
	float t2 = max.x - ray.origin.x;
	float t3 = min.y - ray.origin.y;
	float t4 = max.y - ray.origin.y;
	float t5 = min.z - ray.origin.z;
	float t6 = max.z - ray.origin.z;
	if (ray.direction.x != 0) {
		t1 = t1 / ray.direction.x;
		t2 = t2 / ray.direction.x;
	}
	if (ray.direction.y != 0) {
		t3 = t3 / ray.direction.y;
		t4 = t4 / ray.direction.y;
	}
	if (ray.direction.z != 0) {
		t5 = t5 / ray.direction.z;
		t6 = t6 / ray.direction.z;
	}
	//find largest minimum value and smallest maximum value
	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
	if (tmax < 0) return -1; //whole aabb is behind ray
	if (tmin > tmax) return -1; //ray is not intersecting
	if (tmin < 0.0f) return tmax; //ray intesects but origin inside
	return tmin;
}

//Raycasts given ray on OBB using Cyrus-Beck clipping algorithm
float raycast(const OBB& obb, const Ray& ray) {
	//get OBB axes
	const float* orient = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	vec3 xAxis(orient[0], orient[1], orient[2]);
	vec3 yAxis(orient[3], orient[4], orient[5]);
	vec3 zAxis(orient[6], orient[7], orient[8]);
	//vector pointing from ray origin to OBB
	vec3 pointing = obb.center - ray.origin;
	//vector containing projection of ray's direction onto each axis
	vec3 projDir(dot(xAxis, ray.direction), 
		dot(yAxis, ray.direction), 
		dot(zAxis, ray.direction));
	//vector containing projection of pointing vector onto each axis
	vec3 projPtr(dot(xAxis, pointing),
		dot(yAxis, pointing),
		dot(zAxis, pointing));
	//get tmin tmax values for axes
	float t[6] = { 0,0,0,0,0,0 };
	for (int i = 0; i < 3; ++i) {
		if (CMP(projDir[i], 0)) {
			//if ray is parallel to tested axis and ray origin is not inside slab there is no hit
			if (-projPtr[i] - size[i] > 0 || -projPtr[i] + size[i] < 0) return -1;
			projDir[i] = 0.00001f; //to avoid division by 0
		}
		t[i * 2 + 0] = (projPtr[i] + size[i]) / projDir[i]; //minimum
		t[i * 2 + 1] = (projPtr[i] - size[i]) / projDir[i]; //maximum
	}
	//ray hit all three slabs, now get largest minimum and smallest maximum
	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
	if (tmax < 0) return -1.0f; //obb is behind ray
	if (tmin > tmax) return -1.0f; //does not intersect
	if (tmin < 0.0f) return tmax; //ray started inside obb
	return tmin;
}

//Raycasts given ray on plane
float raycast(const Plane& plane, const Ray& ray) {
	float projDir = dot(ray.direction, plane.normal);
	float projOrig = dot(ray.origin, plane.normal);
	if (projDir >= 0.0f) return -1; //ray and normal point in the same direction
	//get distance from ray origin to hit
	float t = (plane.distance - projOrig) / projDir;
	if (t >= 0.0f) return t;
	return -1; //ray hits behind plane
}

//Checks if line and sphere intersect
bool linetest(const Sphere& sphere, const Line& line) {
	Point closest = closestPoint(sphere.center, line);
	float distSq = magnitudeSq(sphere.center - closest);
	return distSq <= sphere.radius * sphere.radius;
}

//Checks if line and AABB intersect
bool linetest(const AABB& aabb, const Line& line) {
	//construct ray and raycast against AABB
	Ray ray;
	ray.origin = line.start;
	ray.direction = normalized(line.end - line.start);
	RaycastResult result;
	if (!raycast(aabb, ray, &result)) {
		return false;
	}
	//float t = raycast(aabb, ray);
	float t = result.t;
	//if distance of hit is positive and less or equal 
	//length of a line then we have intersection
	return t >= 0 && t * t <= lengthSq(line);
}

//Checks if line and OBB intersect
bool linetest(const OBB& obb, const Line& line) {
	//construct ray and raycast against OBB
	Ray ray;
	ray.origin = line.start;
	ray.direction = normalized(line.end - line.start);
	RaycastResult result;
	if (!raycast(obb, ray, &result)) {
		return false;
	}
	//float t = raycast(obb, ray);
	float t = result.t;
	//if distance of hit is positive and less or equal 
	//length of a line then we have intersection
	return t >= 0 && t * t <= lengthSq(line);
}

//Checks if line and plane intersect
bool linetest(const Plane& plane, const Line& line) {
	vec3 l = line.end - line.start;
	float projStart = dot(plane.normal, line.start);
	float projLine = dot(plane.normal, l);
	if (projLine == 0) return false; //line and normal are parallel
	//get distance along line where it hits plane
	float t = (plane.distance - projStart)/projLine;
	return t >= 0.0f && t <= 1.0f;
}

//Checks if point is in triangle
bool pointInTriangle(const Point& p, const Triangle& t) {
	//local triangle with point in origin
	vec3 a = t.a - p;
	vec3 b = t.b - p;
	vec3 c = t.c - p;
	//"create" pyramid, if normal vectors of p__ 
	//sides face the same direction point is in triangle
	vec3 normPBC = cross(b, c);
	vec3 normPCA = cross(c, a);
	vec3 normPAB = cross(a, b);
	if (dot(normPBC, normPCA) < 0.0f) return false;
	else if (dot(normPBC, normPAB) < 0.0f) return false;
	return true;
}

//Creates plane from triangle
Plane planeFromTriangle(const Triangle& t) {
	Plane result;
	result.normal = normalized(cross(t.b - t.a, t.c - t.a));
	result.distance = dot(result.normal, t.a);
	return result;
}

//Returns Triangle's closest point to given point
Point closestPoint(const Point& point, const Triangle& t) {
	Plane plane = planeFromTriangle(t);
	Point closest = closestPoint(point, plane);
	if (pointInTriangle(closest, t)) return closest;
	//closest point of each side of triangle
	Point c1 = closestPoint(point, Line(t.a, t.b));
	Point c2 = closestPoint(point, Line(t.b, t.c));
	Point c3 = closestPoint(point, Line(t.c, t.a));
	//distances to point
	float dist1 = magnitudeSq(point - c1);
	float dist2 = magnitudeSq(point - c2);
	float dist3 = magnitudeSq(point - c3);
	if (dist1 < dist2 && dist1 < dist3) return c1;
	else if (dist2 < dist1 && dist2 < dist3) return c2;
	return c3;
}

//Checks if triangle and sphere intersect
bool triangleSphereIntxn(const Triangle& t, const Sphere& sphere) {
	Point closest = closestPoint(sphere.center, t);
	float magSq = magnitudeSq(closest - sphere.center);
	return magSq <= sphere.radius * sphere.radius;
}

//Returns interval of triangle projected onto axis
Interval getInterval(const Triangle& t, const vec3& axis) {
	Interval result;
	result.max = result.min = dot(axis, t.points[0]);
	for (int i = 1; i < 3; ++i) {
		float value = dot(axis, t.points[i]);
		result.min = fminf(result.min, value);
		result.max = fmaxf(result.max, value);
	}
	return result;
}

//Checks if two objects overlap on axis
bool overlapOnAxis(const Triangle& t, const AABB& aabb, const vec3& axis) {
	Interval a = getInterval(t, axis);
	Interval b = getInterval(aabb, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

//Checks if triangle and AABB intersect using SAT test
bool triangleAABBIntxn(const Triangle& t, const AABB& aabb) {
	//edge vectors of triangle
	vec3 edge1 = t.b - t.a;
	vec3 edge2 = t.c - t.b;
	vec3 edge3 = t.a - t.c;
	//face normals of aabb
	vec3 face1(1.0f, 0.0f, 0.0f);
	vec3 face2(0.0f, 1.0f, 0.0f);
	vec3 face3(0.0f, 0.0f, 1.0f);
	//get all axes
	vec3 axes[13] = {
		face1, face2, face3,
		cross(edge1, edge2),
		cross(face1, edge1), cross(face1, edge2), cross(face1, edge3),
		cross(face2, edge1), cross(face2, edge2), cross(face2, edge3),
		cross(face3, edge1), cross(face3, edge2), cross(face3, edge3)
	};
	for (int i = 0; i < 13; ++i) {
		if (!overlapOnAxis(t, aabb, axes[i])) return false;
	}
	return true;
}

//Checks if two objects overlap on axis
bool overlapOnAxis(const Triangle& t, const OBB& obb, const vec3& axis) {
	Interval a = getInterval(t, axis);
	Interval b = getInterval(obb, axis);
	return ((a.min <= b.max) && (b.min <= a.max));
}

//Checks if triangle and OBB intersect using SAT test
bool triangleOBBIntxn(const Triangle& t, const OBB& obb) {
	//edge vectors of triangle
	vec3 edge1 = t.b - t.a;
	vec3 edge2 = t.c - t.b;
	vec3 edge3 = t.a - t.c;
	//face normals of obb
	const float* orient = obb.orientation.asArray;
	vec3 face1(orient[0], orient[1], orient[2]);
	vec3 face2(orient[3], orient[4], orient[5]);
	vec3 face3(orient[6], orient[7], orient[8]);
	//get all axes
	vec3 axes[13] = {
		face1, face2, face3,
		cross(edge1, edge2),
		cross(face1, edge1), cross(face1, edge2), cross(face1, edge3),
		cross(face2, edge1), cross(face2, edge2), cross(face2, edge3),
		cross(face3, edge1), cross(face3, edge2), cross(face3, edge3)
	};
	for (int i = 0; i < 13; ++i) {
		if (!overlapOnAxis(t, obb, axes[i])) return false;
	}
	return true;
}

//Checks if triangle and plane intersect
bool trianglePlaneIntxn(const Triangle& t, const Plane& plane) {
	//get sides of the plane that triangle vertices are on
	float side1 = planeEq(t.a, plane);
	float side2 = planeEq(t.b, plane);
	float side3 = planeEq(t.c, plane);
	//all points on plane
	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0)) return true;
	//all points in front of plane
	if (side1 > 0 && side2 > 0 && side3 > 0) return false;
	//all points in front of plane
	if (side1 < 0 && side2 < 0 && side3 < 0) return false;
	return true;
}

//Checks if two objects overlap on axis
bool overlapOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis) {
	Interval a = getInterval(t1, axis);
	Interval b = getInterval(t2, axis);
	return ((a.min <= b.max) && (b.min <= a.max));
}

//Checks for cross product result of triangle edges equal to zero
vec3 crossEdgeSAT(const vec3& a, const vec3& b, const vec3& c, const vec3& d) {
	vec3 ab = a - b;
	vec3 cd = c - d;
	vec3 result = cross(ab, cd);
	//not parallel
	if (!CMP(magnitudeSq(result), 0)) return result;
	else {
		//get axis perpendicular to ab and try again
		vec3 axis = cross(ab, c - a);
		result = cross(ab, axis);
		//not parallel
		if (!CMP(magnitudeSq(result), 0)) return result;
	}
	return vec3();
}

//Checks if two triangles intersect using SAT test
bool triangleTriangleIntxn(const Triangle& t1, const Triangle& t2) {
	//get all axes
	vec3 axes[] = {
		crossEdgeSAT(t1.a, t1.b, t1.b, t1.c),
		crossEdgeSAT(t2.a, t2.b, t2.b, t2.c),

		crossEdgeSAT(t2.a, t2.b, t1.a, t1.b),
		crossEdgeSAT(t2.a, t2.b, t1.b, t1.c),
		crossEdgeSAT(t2.a, t2.b, t1.c, t1.a),

		crossEdgeSAT(t2.b, t2.c, t1.a, t1.b),
		crossEdgeSAT(t2.b, t2.c, t1.b, t1.c),
		crossEdgeSAT(t2.b, t2.c, t1.c, t1.a),

		crossEdgeSAT(t2.c, t2.a, t1.a, t1.b),
		crossEdgeSAT(t2.c, t2.a, t1.b, t1.c),
		crossEdgeSAT(t2.c, t2.a, t1.c, t1.a)
	};
	for (int i = 0; i < 11; ++i) {
		if (!overlapOnAxis(t1, t2, axes[i])) {
			//check for case if crossEdgeSAT returned zero vector
			if (!CMP(magnitudeSq(axes[i]), 0)) return false;
		}
	}
	return true;
}

//Returns barycentric coordinates of a point with respect to triangle
vec3 getBarycentric(const Point& p, const Triangle& t) {
	//get vectors from triangle vertices to point
	vec3 ap = p - t.a;
	vec3 bp = p - t.b;
	vec3 cp = p - t.c;
	//get edges of triangle
	vec3 ab = t.b - t.a;
	vec3 ac = t.c - t.a;
	vec3 bc = t.c - t.b;
	vec3 cb = t.b - t.c;
	vec3 ca = t.a - t.c;
	//get perpendicular vectors to edges and 
	//values of point projected onto them,
	//values are 0 is point is on line, 1 if point is at triangle vertex
	vec3 v = perpendicular(ab, cb);
	float a = 1.0f - (dot(v, ap) / dot(v, ab));
	v = perpendicular(bc, ac);
	float b = 1.0f - (dot(v, bp) / dot(v, bc));
	v = perpendicular(ca, ab);
	float c = 1.0f - (dot(v, cp) / dot(v, ca));
	return vec3(a, b, c);
}

//Raycasts given ray on triangle
float raycast(const Triangle& t, const Ray& ray) {
	Plane plane = planeFromTriangle(t);
	float tm = raycast(plane, ray);
	if (tm < 0.0f) return tm;
	Point result = ray.origin + ray.direction * tm;
	vec3 barCoords = getBarycentric(result, t);
	if (barCoords.x >= 0.0f && barCoords.x <= 1.0f &&
		barCoords.y >= 0.0f && barCoords.y <= 1.0f &&
		barCoords.z >= 0.0f && barCoords.z <= 1.0f) {
		return tm;
	}
	return -1;
}

//Checks if line and triangle intersect
bool linetest(const Triangle& t, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = normalized(line.end - line.start);
	RaycastResult result;
	if (!raycast(t, ray, &result)) return false;
	float tm = result.t;
	return tm >= 0 && tm * tm <= lengthSq(line);
}

//Creates BVHNode for given mesh
void accelerateMesh(Mesh& mesh) {
	//check for existing accelerator
	if (mesh.accelerator != 0) return;
	//find minimum and maximum of mesh and create bounding AABB
	vec3 min = mesh.vertices[0];
	vec3 max = mesh.vertices[0];
	for (int i = 1; i < mesh.triangleNum * 3; ++i) {
		min.x = fminf(mesh.vertices[i].x, min.x);
		min.y = fminf(mesh.vertices[i].y, min.y);
		min.z = fminf(mesh.vertices[i].z, min.z);
		max.x = fmaxf(mesh.vertices[i].x, max.x);
		max.y = fmaxf(mesh.vertices[i].y, max.y);
		max.z = fmaxf(mesh.vertices[i].z, max.z);
	}
	//create accelerator
	mesh.accelerator = new BVHNode();
	mesh.accelerator->bounds = fromMinMax(min, max);
	mesh.accelerator->triangleNum = mesh.triangleNum;
	mesh.accelerator->triangles = new int[mesh.triangleNum];
	for (int i = 0; i < mesh.triangleNum; ++i) {
		mesh.accelerator->triangles[i] = i;
	}
	splitBVHNode(mesh.accelerator, mesh, 3);
}

//Recursively splits BVHNode until given depth is reached
void splitBVHNode(BVHNode* node, const Mesh& model, int depth) {
	if (depth-- == 0) return;
	if (node->children == 0) {//split if leaf
		if (node->triangleNum > 0) {//split if contains triangles
			node->children = new BVHNode[8];
			vec3 cent = node->bounds.center;
			vec3 ext = node->bounds.size * 0.5f;
			node->children[0].bounds = AABB(cent + vec3(-ext.x, +ext.y, -ext.z), ext);
			node->children[1].bounds = AABB(cent + vec3(+ext.x, +ext.y, -ext.z), ext);
			node->children[2].bounds = AABB(cent + vec3(-ext.x, +ext.y, +ext.z), ext);
			node->children[3].bounds = AABB(cent + vec3(+ext.x, +ext.y, +ext.z), ext);
			node->children[4].bounds = AABB(cent + vec3(-ext.x, -ext.y, -ext.z), ext);
			node->children[5].bounds = AABB(cent + vec3(+ext.x, -ext.y, -ext.z), ext);
			node->children[6].bounds = AABB(cent + vec3(-ext.x, -ext.y, +ext.z), ext);
			node->children[7].bounds = AABB(cent + vec3(+ext.x, -ext.y, +ext.z), ext);
		}
	}
	if (node->children != 0 && node->triangleNum > 0) {
		//if node was just split
		for (int i = 0; i < 8; ++i) {
			node->children[i].triangleNum = 0;
			//fill triangleNum for each child
			for (int j = 0; j < node->triangleNum; ++j) {
				Triangle t = model.triangles[node->triangles[j]];
				if (triangleAABBIntxn(t, node->children[i].bounds)) {
					node->children[i].triangleNum += 1;
				}
			}
			if (node->children[i].triangleNum == 0) {
				continue; //if no triangles do nothing
			}
			node->children[i].triangles = new int[node->children[i].triangleNum];
			int index = 0;
			//if triangle in node is in child add it's index to the list
			for (int j = 0; j < node->triangleNum; ++j) {
				Triangle t = model.triangles[node->triangles[j]];
				if (triangleAABBIntxn(t, node->children[i].bounds)) {
					node->children[i].triangles[index++] = node->triangles[j];
				}
			}
		}
		//cleanup
		node->triangleNum = 0;
		delete[] node->triangles;
		node->triangles = 0;
		//split recursively
		for (int i = 0; i < 8; ++i) {
			splitBVHNode(&node->children[i], model, depth);
		}
	}
}

//Deletes all children of given node recursively
void freeBVHNode(BVHNode* node) {
	if (node->children != 0) {
		//cleanup recursively
		for (int i = 0; i < 8; ++i) {
			freeBVHNode(&node->children[i]);
		}
		delete[] node->children;
		node->children = 0;
	}
	//if triangle indices are present, release them
	if (node->triangleNum != 0 || node->triangles != 0) {
		delete[] node->triangles;
		node->triangles = 0;
		node->triangleNum = 0;
	}
}

//Checks if mesh and ray intersect
float meshRayIntxn(const Mesh& mesh, const Ray& ray) {
	if (mesh.accelerator == 0) {//if no accelerator
		for (int i = 0; i < mesh.triangleNum; ++i) {
			RaycastResult raycastResult;
			raycast(mesh.triangles[i], ray, &raycastResult);
			float result = raycastResult.t;
			if (result >= 0) {
				return result;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		//recursively walk BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					RaycastResult raycastResult;
					raycast(mesh.triangles[iterator->triangles[i]], ray, &raycastResult);
					float r = raycastResult.t;
					if (r >= 0) {
						return r;
					}
				}
			}
			//add any children if they are hit
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					RaycastResult result;
					raycast(iterator->children[i].bounds, ray, &result);
					if (result.t >= 0) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return -1;
}

//Checks if mesh and AABB intersect
bool meshAABBIntxn(const Mesh& mesh, const AABB& aabb) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (triangleAABBIntxn(mesh.triangles[i], aabb)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (triangleAABBIntxn(mesh.triangles[iterator->triangles[i]], aabb)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (AABBAABBIntxn(iterator->children[i].bounds, aabb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//Checks if mesh and OBB intersect
bool meshOBBIntxn(const Mesh& mesh, const OBB& obb) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (triangleOBBIntxn(mesh.triangles[i], obb)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (triangleOBBIntxn(mesh.triangles[iterator->triangles[i]], obb)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (AABBOBBIntxn(iterator->children[i].bounds, obb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//Checks if mesh and plane intersect
bool meshPlaneIntxn(const Mesh& mesh, const Plane& plane) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (trianglePlaneIntxn(mesh.triangles[i], plane)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (trianglePlaneIntxn(mesh.triangles[iterator->triangles[i]], plane)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (AABBPlaneIntxn(iterator->children[i].bounds, plane)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//Checks if mesh and triangle intersect
bool meshTriangleIntxn(const Mesh& mesh, const Triangle& t) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (triangleTriangleIntxn(mesh.triangles[i], t)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (triangleTriangleIntxn(mesh.triangles[iterator->triangles[i]], t)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (triangleAABBIntxn(t, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//Checks if mesh and line intersect
bool meshLineIntxn(const Mesh& mesh, const Line& line) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (linetest(mesh.triangles[i], line)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (linetest(mesh.triangles[iterator->triangles[i]], line)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (linetest(iterator->children[i].bounds, line)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//Checks if mesh and sphere intersect
bool meshSphereIntxn(const Mesh& mesh, const Sphere& sphere) {
	//if no accelerator check normally
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.triangleNum; ++i) {
			if (triangleSphereIntxn(mesh.triangles[i], sphere)) return true;
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			//if it has triangles check them
			if (iterator->triangleNum >= 0) {
				for (int i = 0; i < iterator->triangleNum; ++i) {
					if (triangleSphereIntxn(mesh.triangles[iterator->triangles[i]], sphere)) {
						return true;
					}
				}
			}
			//if it has children check for possible addition to test
			if (iterator->children != 0) {
				for (int i = 7; i >= 0; --i) {
					if (sphereAABBIntxn(sphere, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

//model contents setter
void Model::setContents(Mesh* mesh) {
	contents = mesh;
	//if it is valid, calculate bounds
	if (contents != 0) {
		vec3 min = mesh->vertices[0];
		vec3 max = mesh->vertices[0];
		for (int i = 1; i < mesh->triangleNum * 3; ++i) {
			min.x = fminf(mesh->vertices[i].x, min.x);
			min.y = fminf(mesh->vertices[i].y, min.y);
			min.z = fminf(mesh->vertices[i].z, min.z);
			max.x = fmaxf(mesh->vertices[i].x, max.x);
			max.y = fmaxf(mesh->vertices[i].y, max.y);
			max.z = fmaxf(mesh->vertices[i].z, max.z);
		}
		bounds = fromMinMax(min, max);
	}
}

//Returns world matrix of the given model
mat4 getWorldMatrix(const Model& model) {
	//build local matrix
	mat4 transl = translation(model.position);
	mat4 rotat = rotation(model.rotation.x, model.rotation.y, model.rotation.z);
	mat4 local = rotat * transl;
	mat4 parentMat;
	if (model.parent != 0) {
		parentMat = getWorldMatrix(*model.parent);
	}
	return local * parentMat;
}

//Returns OBB surrounding model
OBB getOBB(const Model& model) {
	mat4 world = getWorldMatrix(model);
	AABB aabb = model.getBounds();
	OBB obb;
	obb.size = aabb.size;
	obb.center = multiplyPoint(aabb.center, world);
	obb.orientation = cut(world, 3, 3);
	return obb;
}

//Checks if model and ray intersect
float modelRayIntxn(const Model& model, const Ray& ray) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform ray into local space
	Ray local;
	local.origin = multiplyPoint(ray.origin, inv);
	local.direction = multiplyVector(ray.direction, inv);
	local.normalizeDirection();
	if (model.getMesh() != 0) { //raycast is there is mesh
		return meshRayIntxn(*(model.getMesh()), local);
	}
	return -1;
}

//Checks if model and AABB intersect
bool modelAABBIntxn(const Model& model, const AABB& aabb) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform AABB into local space
	//this can rotate AABB so change it to OBB
	OBB local;
	local.center = multiplyPoint(aabb.center, inv);
	local.size = aabb.size;
	local.orientation = cut(inv, 3, 3);
	if (model.getMesh() != 0) { //check intersection is there is mesh
		return meshOBBIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Checks if model and OBB intersect
bool modelOBBIntxn(const Model& model, const OBB& obb) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform OBB into local space
	OBB local;
	local.center = multiplyPoint(obb.center, inv);
	local.size = obb.size;
	local.orientation = obb.orientation * cut(inv, 3, 3);
	if (model.getMesh() != 0) { //check intersection is there is mesh
		return meshOBBIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Checks if model and plane intersect
bool modelPlaneIntxn(const Model& model, const Plane& plane) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform OBB into local space
	Plane local;
	local.normal = multiplyVector(plane.normal, inv);
	local.distance = plane.distance;
	if (model.getMesh() != 0) { //check intersection is there is mesh
		return meshPlaneIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Checks if model and triangle intersect
bool modelTriangleIntxn(const Model& model, const Triangle& t) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform OBB into local space
	Triangle local;
	local.a = multiplyPoint(t.a, inv);
	local.b = multiplyPoint(t.b, inv);
	local.c = multiplyPoint(t.c, inv);
	if (model.getMesh() != 0) { //check intersection is there is mesh
		return meshTriangleIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Checks if model and line intersect
bool modelLineIntxn(const Model& model, const Line& line) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform line into local space
	Line local;
	local.start = multiplyPoint(line.start, inv);
	local.end = multiplyPoint(line.end, inv);
	if (model.getMesh() != 0) { //linetest is there is mesh
		return meshLineIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Checks if model and sphere intersect
bool modelSphereIntxn(const Model& model, const Sphere& sphere) {
	mat4 world = getWorldMatrix(model);
	mat4 inv = inverse(world);
	//use inverse world matrix to transform sphere into local space
	Sphere local;
	local.center = multiplyPoint(sphere.center, inv);
	if (model.getMesh() != 0) { //check intersection is there is mesh
		return meshSphereIntxn(*(model.getMesh()), local);
	}
	return false;
}

//Computes intersection point of three planes using Cramers Rule
Point planeIntxn(Plane p1, Plane p2, Plane p3) {
	//create matrix of coefficient
	mat3 coeff(
		p1.normal.x, p2.normal.x, p3.normal.x,
		p1.normal.y, p2.normal.y, p3.normal.y,
		p1.normal.z, p2.normal.z, p3.normal.z
	);
	vec3 dist(-p1.distance, -p2.distance, -p3.distance);
	//matrices with rows replaced with answer/distance vector
	mat3 coefX = coeff;
	mat3 coefY = coeff;
	mat3 coefZ = coeff;
	coefX._11 = dist.x; coefX._12 = dist.y; coefX._13 = dist.z;
	coefY._21 = dist.x; coefY._22 = dist.y; coefY._23 = dist.z;
	coefZ._31 = dist.x; coefZ._32 = dist.y; coefZ._33 = dist.z;
	float coeffDet = determinant(coeff);
	if (CMP(coeffDet, 0)) {
		return Point();
	}
	float coefXDet = determinant(coefX);
	float coefYDet = determinant(coefY);
	float coefZDet = determinant(coefZ);
	//return point of intersection according to Cramers Rule
	return Point(coefXDet / coeffDet, coefYDet / coeffDet, coefZDet / coeffDet);
}

//Computes all corners of a frustum
void getCorners(const Frustum& f, vec3* output) {
	output[0] = planeIntxn(f.near, f.top, f.left);
	output[1] = planeIntxn(f.near, f.top, f.right);
	output[2] = planeIntxn(f.near, f.bottom, f.left);
	output[3] = planeIntxn(f.near, f.bottom, f.right);
	output[4] = planeIntxn(f.far, f.top, f.left);
	output[5] = planeIntxn(f.far, f.top, f.right);
	output[6] = planeIntxn(f.far, f.bottom, f.left);
	output[7] = planeIntxn(f.far, f.bottom, f.right);
}

//Checks if frustum and point intersect
bool frustumPointIntxn(const Frustum& f, const Point& p) {
	for (int i = 0; i < 6; ++i) {
		vec3 normal = f.planes[i].normal;
		float distance = f.planes[i].distance;
		float side = dot(p, normal) + distance;
		if (side < 0.0f) { //point is behind plane
			return false;
		}
	}
	return true;
}

//Checks if frustum and sphere intersect
bool frustumSphereIntxn(const Frustum& f, const Sphere& s) {
	for (int i = 0; i < 6; ++i) {
		vec3 normal = f.planes[i].normal;
		float distance = f.planes[i].distance;
		float side = dot(s.center, normal) + distance;
		if (side < -s.radius) { //sphere is behind plane
			return false;
		}
	}
	return true;
}

//Classifies whether AABB is behind, in front of plane or intersects it
float classify(const AABB& aabb, const Plane& plane) {
	//extents projected onto normal
	float r = fabsf(aabb.size.x * plane.normal.x) +
		fabsf(aabb.size.y * plane.normal.y) +
		fabsf(aabb.size.z * plane.normal.z);
	//distance between AABB center and plane
	float d = dot(plane.normal, aabb.center) + plane.distance;
	if (fabsf(d) < r) return 0.0f;
	else if (d < 0.0f) return d + r;
	return d - r;
}

//Classifies whether OBB is behind, in front of plane or intersects it
float classify(const OBB& obb, const Plane& plane) {
	//transform normal into OBB local space
	vec3 normal = multiplyVector(plane.normal, obb.orientation);
	//extents projected onto normal
	float r = fabsf(obb.size.x * normal.x) +
		fabsf(obb.size.y * normal.y) +
		fabsf(obb.size.z * normal.z);
	//distance between OBB center and plane
	float d = dot(plane.normal, obb.center) + plane.distance;
	if (fabsf(d) < r) return 0.0f;
	else if (d < 0.0f) return d + r;
	return d - r;
}

//Checks if frustum and AABB intersect
bool frustumAABBIntxn(const Frustum& f, const AABB& aabb) {
	for (int i = 0; i < 6; ++i) {
		if (classify(aabb, f.planes[i]) < 0) {
			return false;
		}
	}
	return true;
}

//Checks if frustum and OBB intersect
bool frustumOBBIntxn(const Frustum& f, const OBB& obb) {
	for (int i = 0; i < 6; ++i) {
		if (classify(obb, f.planes[i]) < 0) {
			return false;
		}
	}
	return true;
}

//Turns screen-space pixel into world-space vector
vec3 unproject(const vec3& viewportPoint, const vec2& viewportOrigin,
	const vec2& viewportSize, const mat4& view, const mat4& projection) {

	//normalize input point vector to viewport
	float normd[4] = {
		(viewportPoint.x - viewportOrigin.x) / viewportSize.x,
		(viewportPoint.y - viewportOrigin.y) / viewportSize.y,
		viewportPoint.z, 1.0f
	};
	//translate normalized vector to ndc (normalized device coords) space
	float ndcSpace[4] = { normd[0],normd[1],normd[2],normd[3] };
	//clamp x to (-1,1) range, y to (-1,1), z to (0,1)
	//change to (-1,1) if using openGL style ranges
	ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f;
	ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f; //y is flipped
	if (ndcSpace[2] < 0.0f) ndcSpace[2] = 0.0f;
	if (ndcSpace[2] > 1.0f) ndcSpace[2] = 1.0f;
	//transform into eye space
	mat4 invProj = inverse(projection);
	float eyeSpace[4] = { 0.0f,0.0f,0.0f,0.0f };
	multiply(eyeSpace, ndcSpace, 1, 4, invProj.asArray, 4, 4);
	//translate to world space
	mat4 invView = inverse(view);
	float worldSpace[4] = { 0.0f,0.0f,0.0f,0.0f };
	multiply(worldSpace, eyeSpace, 1, 4, invView.asArray, 4, 4);
	//undo perspective divide
	if (!CMP(worldSpace[3], 0.0f)) {
		worldSpace[0] /= worldSpace[3];
		worldSpace[1] /= worldSpace[3];
		worldSpace[2] /= worldSpace[3];
	}
	return vec3(worldSpace[0], worldSpace[1], worldSpace[2]);
}

//Returns picking ray from screen-space pixel
Ray getPickRay(const vec2& viewportPoint, const vec2& viewportOrigin,
	const vec2& viewportSize, const mat4& view, const mat4& projection) {

	vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f);
	vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);
	//transform pixel/screen space points into world
	vec3 nearP = unproject(nearPoint, viewportOrigin, viewportSize, view, projection);
	vec3 farP = unproject(farPoint, viewportOrigin, viewportSize, view, projection);
	vec3 normal = normalized(farP - nearP);
	vec3 origin = nearP;
	return Ray(origin, normal);
}

//Resets raycast result structure
void resetRaycastResult(RaycastResult* result) {
	if (result != 0) {
		result->t = -1;
		result->hit = false;
		result->normal = vec3(0, 0, 1);
		result->point = vec3(0, 0, 0);
	}
}

bool raycast(const Sphere& sphere, const Ray& ray, RaycastResult* result) {
	resetRaycastResult(result);
	vec3 e = sphere.center - ray.origin;
	float rSq = sphere.radius * sphere.radius;
	float eSq = magnitudeSq(e);
	float a = dot(e, ray.direction);
	float bSq = eSq - (a*a);
	float f = sqrt(rSq - bSq);
	float t = a - f;
	if (rSq - bSq < 0.0f) return false;
	else if (eSq < rSq) {
		t = a + f;
	}
	if (result != 0) {
		result->t = t;
		result->hit = true;
		result->point = ray.origin + ray.direction * t;
		result->normal = normalized(result->point - sphere.center);
	}
	return true;
}

bool raycast(const AABB& aabb, const Ray& ray, RaycastResult* result) {
	resetRaycastResult(result);
	vec3 min = getMin(aabb);
	vec3 max = getMax(aabb);
	float t[] = { 0,0,0,0,0,0 };
	t[0] = min.x - ray.origin.x;
	t[1] = max.x - ray.origin.x;
	t[2] = min.y - ray.origin.y;
	t[3] = max.y - ray.origin.y;
	t[4] = min.z - ray.origin.z;
	t[5] = max.z - ray.origin.z;
	if (!CMP(ray.direction.x,0)) {
		t[0] = t[0] / ray.direction.x;
		t[1] = t[1] / ray.direction.x;
	}
	if (!CMP(ray.direction.y, 0)) {
		t[2] = t[2] / ray.direction.y;
		t[3] = t[3] / ray.direction.y;
	}
	if (!CMP(ray.direction.z, 0)) {
		t[4] = t[4] / ray.direction.z;
		t[5] = t[5] / ray.direction.z;
	}
	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
	if (tmax < 0) return false;
	if (tmin > tmax) return false;
	float t_result = tmin;
	if (tmin < 0.0f) {
		t_result = tmax;
	}
	if (result != 0) {
		result->t = t_result;
		result->hit = true;
		result->point = ray.origin + ray.direction * t_result;
		vec3 normals[] = {
			vec3(-1, 0, 0), vec3(1, 0, 0),
			vec3(0, -1, 0), vec3(0, 1, 0),
			vec3(0, 0, -1), vec3(0, 0, 1)
		};
		for (int i = 0; i < 6; ++i) {
			if (CMP(t_result, t[i])) {
				result->normal = normals[i];
			}
		}
	}
	return true;
}

bool raycast(const OBB& obb, const Ray& ray, RaycastResult* result) {
	resetRaycastResult(result);
	const float* o = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	vec3 p = obb.center - ray.origin;
	vec3 X(o[0], o[1], o[2]);
	vec3 Y(o[3], o[4], o[5]);
	vec3 Z(o[6], o[7], o[8]);
	vec3 f(dot(X, ray.direction), dot(Y, ray.direction), dot(Z, ray.direction));
	vec3 e(dot(X, p), dot(Y, p), dot(Z, p));
	float t[6] = { 0,0,0,0,0,0 };
	for (int i = 0; i < 3; ++i) {
		if (CMP(f[i], 0)) {
			//AND or OR?
			if (-e[i] - size[i] > 0 || -e.x + size[i] < 0) return false;
			f[i] = 0.00001f;
		}
		t[i * 2 + 0] = (e[i] + size[i]) / f[i];
		t[i * 2 + 1] = (e[i] - size[i]) / f[i];
	}
	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
	if (tmax < 0) return false;
	if (tmin > tmax) return false;
	float t_result = tmin;
	if (tmin < 0.0f) {
		t_result = tmax;
	}
	if (result != 0) {
		result->t = t_result;
		result->hit = true;
		result->point = ray.origin + ray.direction * t_result;
		vec3 normals[] = {
			X, X * -1.0f,
			Y, Y * -1.0f,
			Z, Z * -1.0f
		};
		for (int i = 0; i < 6; ++i) {
			if (CMP(t_result, t[i])) {
				result->normal = normalized(normals[i]);
			}
		}
	}
	return true;
}

bool raycast(const Plane& plane, const Ray& ray, RaycastResult* result) {
	resetRaycastResult(result);
	float nd = dot(ray.direction, plane.normal);
	float pn = dot(ray.origin, plane.normal);
	if (nd >= 0.0f) return false;
	float t = (plane.distance - pn) / nd;
	if (t >= 0.0f) {
		if (result != 0) {
			result->t = t;
			result->hit = true;
			result->point = ray.origin + ray.direction*t;
			result->normal = normalized(plane.normal);
		}
		return true;
	}
	return false;
}

bool raycast(const Triangle& triangle, const Ray& ray, RaycastResult* result) {
	resetRaycastResult(result);
	Plane plane = planeFromTriangle(triangle);
	RaycastResult planeResult;
	if (!raycast(plane, ray, &planeResult)) return false;
	float t = planeResult.t;
	Point res = ray.origin + ray.direction * t;
	vec3 bar = getBarycentric(res, triangle);
	if (bar.x >= 0.0f && bar.x <= 1.0f &&
		bar.y >= 0.0f && bar.y <= 1.0f &&
		bar.z >= 0.0f && bar.z <= 1.0f) {
		if (result != 0) {
			result->t = t;
			result->hit = true;
			result->point = ray.origin + ray.direction*t;
			result->normal = plane.normal;
		}
		return true;
	}
	return false;
}

void resetCollisionManifold(CollisionManifold* result) {
	if (result != 0) {
		result->colliding = false;
		result->normal = vec3(0, 0, 1);
		result->depth = FLT_MAX;
		result->contacts.clear();
	}
}

CollisionManifold findCollisionFeatures(const Sphere& s1, const Sphere& s2) {
	CollisionManifold result;
	resetCollisionManifold(&result);
	float r = s1.radius + s2.radius;
	vec3 d = s2.center - s1.center;
	if (magnitudeSq(d) - r * r > 0 || magnitudeSq(d) == 0.0f) {
		return result;
	}
	normalize(d);
	result.colliding = true;
	result.normal = d;
	result.depth = fabsf(magnitude(d) - r) * 0.5f;
	float dtp = s1.radius - result.depth;
	Point contact = s1.center + d * dtp;
	result.contacts.push_back(contact);
	return result;
}

CollisionManifold findCollisionFeatures(const OBB& obb, const Sphere& s) {
	CollisionManifold result;
	resetCollisionManifold(&result);
	Point closest = closestPoint(s.center, obb);
	float dSq = magnitudeSq(closest - s.center);
	if (dSq > s.radius * s.radius) {
		return result;
	}
	vec3 normal;
	if (CMP(dSq, 0.0f)) {
		float mSq = magnitudeSq(closest - obb.center);
		if (CMP(mSq, 0.0f)) {
			return result;
		}
		normal = normalized(closest - obb.center);
	}
	else {
		normal = normalized(s.center - closest);
	}
	Point outside = s.center - normal * s.radius;
	float dist = magnitude(closest - outside);
	result.colliding = true;
	result.contacts.push_back(closest + (outside - closest) * 0.5f);
	result.normal = normal;
	result.depth = dist * 0.5f;
	return result;
}

std::vector<Point> getVertices(const OBB& obb) {
	std::vector<vec3> v;
	v.resize(8);
	vec3 cent = obb.center;
	vec3 ext = obb.size;
	const float* orient = obb.orientation.asArray;
	vec3 axes[] = {
		vec3(orient[0],orient[1],orient[2]),
		vec3(orient[3],orient[4],orient[5]),
		vec3(orient[6],orient[7],orient[8])
	};
	v[0] = cent + axes[0] * ext[0] + axes[1] * ext[1] + axes[2] * ext[2];
	v[1] = cent - axes[0] * ext[0] + axes[1] * ext[1] + axes[2] * ext[2];
	v[2] = cent + axes[0] * ext[0] - axes[1] * ext[1] + axes[2] * ext[2];
	v[3] = cent + axes[0] * ext[0] + axes[1] * ext[1] - axes[2] * ext[2];
	v[4] = cent - axes[0] * ext[0] - axes[1] * ext[1] - axes[2] * ext[2];
	v[5] = cent + axes[0] * ext[0] - axes[1] * ext[1] - axes[2] * ext[2];
	v[6] = cent - axes[0] * ext[0] + axes[1] * ext[1] - axes[2] * ext[2];
	v[7] = cent - axes[0] * ext[0] - axes[1] * ext[1] + axes[2] * ext[2];
	return v;
}

std::vector<Line> getEdges(const OBB& obb) {
	std::vector<Line> result;
	result.reserve(12);
	std::vector<Point> v = getVertices(obb);
	//edges given as pair of indexes
	int index[][2] = {
		{ 6,1 },{ 6,3 },{ 6,4 },{ 2,7 },{ 2,5 },{ 2,0 },
	{ 0,1 },{ 0,3 },{ 7,1 },{ 7,4 },{ 4,5 },{ 5,3 }
	};
	for (int j = 0; j < 12; ++j) {
		result.push_back(Line(v[index[j][0]], v[index[j][1]]));
	}
	return result;
}

std::vector<Plane> getPlanes(const OBB& obb) {
	vec3 cent = obb.center;
	vec3 ext = obb.size;
	const float* orient = obb.orientation.asArray;
	vec3 axes[] = {
		vec3(orient[0],orient[1],orient[2]),
		vec3(orient[3],orient[4],orient[5]),
		vec3(orient[6],orient[7],orient[8])
	};
	std::vector<Plane> result;
	result.resize(6);
	result[0] = Plane(axes[0], dot(axes[0], (cent + axes[0] * ext.x)));
	result[1] = Plane(axes[0] * -1.0f, -dot(axes[0], (cent - axes[0] * ext.x)));
	result[2] = Plane(axes[1], dot(axes[1], (cent + axes[1] * ext.y)));
	result[3] = Plane(axes[1] * -1.0f, -dot(axes[1], (cent - axes[1] * ext.y)));
	result[4] = Plane(axes[2], dot(axes[2], (cent + axes[2] * ext.z)));
	result[5] = Plane(axes[2] * -1.0f, -dot(axes[2], (cent - axes[2] * ext.z)));
	return result;
}

bool clipToPlane(const Plane& plane, const Line& line, Point* outPoint) {
	vec3 ab = line.end - line.start;
	float n = dot(plane.normal, ab);
	if (CMP(n, 0)) { return false; }
	float nA = dot(plane.normal, line.start);
	float t = (plane.distance - nA) / n;
	if (t >= 0.0f && t <= 1.0f) {
		if (outPoint != 0) {
			*outPoint = line.start + ab * t;
		}
		return true;
	}
	return false;
}

std::vector<Point> clipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb) {
	std::vector<Point> result;
	result.reserve(edges.size());
	Point intxn;
	std::vector<Plane> planes = getPlanes(obb);
	for (int i = 0; i < planes.size(); ++i) {
		for (int j = 0; j < edges.size(); ++j) {
			if (clipToPlane(planes[i], edges[j], &intxn)) {
				if (pointInOBB(intxn, obb)) {
					result.push_back(intxn);
				}
			}
		}
	}
	return result;
}

float penetrationDepth(const OBB& obb1, const OBB& obb2, const vec3& axis, bool* outShouldFlip) {
	Interval i1 = getInterval(obb1, normalized(axis));
	Interval i2 = getInterval(obb2, normalized(axis));
	if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
		return 0.0f;
	}
	float len1 = i1.max - i1.min;
	float len2 = i2.max - i2.min;
	float min = fminf(i1.min, i2.min);
	float max = fmaxf(i1.max, i2.max);
	float len = max - min;
	//should collision normal be flipped
	if (outShouldFlip != 0) {
		*outShouldFlip = (i2.min < i1.min);
	}
	return (len1 + len2) - len;
}

CollisionManifold findCollisionFeatures(const OBB& obb1, const OBB& obb2) {
	CollisionManifold result;
	resetCollisionManifold(&result);
	const float* o1 = obb1.orientation.asArray;
	const float* o2 = obb2.orientation.asArray;
	vec3 test[15] = {
		vec3(o1[0], o1[1], o1[2]),
		vec3(o1[3], o1[4], o1[5]),
		vec3(o1[6], o1[7], o1[8]),
		vec3(o2[0], o2[1], o2[2]),
		vec3(o2[3], o2[4], o2[5]),
		vec3(o2[6], o2[7], o2[8])
	};
	for (int i = 0; i< 3; ++i) {
		test[6 + i * 3 + 0] = cross(test[i], test[0]);
		test[6 + i * 3 + 1] = cross(test[i], test[1]);
		test[6 + i * 3 + 2] = cross(test[i], test[2]);
	}
	vec3* hitNormal = 0;
	bool shouldFlip;
	for (int i = 0; i < 15; ++i) {
		if (magnitudeSq(test[i]) < 0.001f) {
			continue;
		}
		float depth = penetrationDepth(obb1, obb2, test[i], &shouldFlip);
		if (depth <= 0.0f) {
			return result;
		}
		else if (depth < result.depth) {
			if (shouldFlip) {
				test[i] = test[i] * -1.0f;
			}
			result.depth = depth;
			hitNormal = &test[i];
		}
	}
	if (hitNormal == 0) { return result; }
	vec3 axis = normalized(*hitNormal);
	std::vector<Point> clippedTo1 = clipEdgesToOBB(getEdges(obb2), obb1);
	std::vector<Point> clippedTo2 = clipEdgesToOBB(getEdges(obb1), obb2);
	result.contacts.reserve(clippedTo1.size() + clippedTo2.size());
	result.contacts.insert(result.contacts.end(), clippedTo1.begin(), clippedTo1.end());
	result.contacts.insert(result.contacts.end(), clippedTo2.begin(), clippedTo2.end());
	//project result of clipped points onto shared plane
	Interval i = getInterval(obb1, axis);
	float dist = (i.max - i.min) * 0.5f - result.depth * 0.5f;
	vec3 pOnPlane = obb1.center + axis * dist;
	for (int i = result.contacts.size() - 1; i >= 0; --i) {
		vec3 contact = result.contacts[i];
		result.contacts[i] = contact + (axis * dot(axis, pOnPlane - contact));
		//remove duplicate points
		for (int j = result.contacts.size() - 1; j >i; --j) {
			if (magnitudeSq(result.contacts[j] - result.contacts[i]) < 0.0001f) {
				result.contacts.erase(result.contacts.begin() + j);
				break;
			}
		}
	}
	result.colliding = true;
	result.normal = axis;
	return result;
}