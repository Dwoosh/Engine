#include "stdafx.h"
#include "2DGeometry.h"
#include "matrices.h"
#include <cmath>
#include <cfloat>

//macro definition of float numbers comparing function

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))


//Returns interval structure for rectangle
Interval2D getInterval(const Rectangle2D& rect, const vec2& axis) {
	Interval2D result;
	vec2 min = getMin(rect);
	vec2 max = getMax(rect);
	vec2 rectVert[] = {
		vec2(min.x, min.y), vec2(min.x, max.y),
		vec2(max.x, max.y), vec2(max.x, min.y)
	};
	result.min = result.max = dot(axis, rectVert[0]);
	for (int i = 1; i < 4; ++i) {
		float projection = dot(axis, rectVert[i]);
		if (projection < result.min) result.min = projection;
		if (projection > result.max) result.max = projection;
	}
	return result;
}

//Returns interval structure for oriented rectangle
Interval2D getInterval(const OrientedRectangle& rect, const vec2& axis) {
	Rectangle2D r = Rectangle2D(Point2D(rect.center - rect.extents), rect.extents * 2.0f);
	vec2 min = getMin(r);
	vec2 max = getMax(r);
	vec2 rectVert[] = {
		min, max,
		vec2(min.x, max.y), vec2(max.x, min.y)
	};
	float theta = DEG2RAD(rect.rotation);
	float zRotation[] = {
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	for (int i = 0; i < 4; ++i) {
		vec2 r = rectVert[i] - rect.center;
		multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation, 2, 2);
		rectVert[i] = r + rect.center;
	}
	Interval2D result;
	result.min = result.max = dot(axis, rectVert[0]);
	for (int i = 1; i < 4; ++i) {
		float projection = dot(axis, rectVert[i]);
		if (projection < result.min) result.min = projection;
		if (projection > result.max) result.max = projection;
	}
	return result;
}

//Check for rectangles overlapping on axis
bool overlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2, const vec2& axis) {
	Interval2D a = getInterval(rect1, axis);
	Interval2D b = getInterval(rect2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

//Check for rectangles overlapping on axis
bool overlapOnAxis(const Rectangle2D& rect1, const OrientedRectangle& rect2, const vec2& axis) {
	Interval2D a = getInterval(rect1, axis);
	Interval2D b = getInterval(rect2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

//Returns length of a line
float length(const Line2D& line) {
	return magnitude(line.end - line.start);
}

//Returns squared length of a line
float lengthSq(const Line2D& line) {
	return magnitudeSq(line.end - line.start);
}

//Returns minimum vertex of rectangle
vec2 getMin(const Rectangle2D& rect) {
	vec2 p1 = rect.origin;
	vec2 p2 = rect.origin + rect.size;
	return vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}

//Returns maximum vertex of rectangle
vec2 getMax(const Rectangle2D& rect) {
	vec2 p1 = rect.origin;
	vec2 p2 = rect.origin + rect.size;
	return vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}

//Constructs rectangle out of minimum and maximum points
Rectangle2D fromMinMax(const vec2& min, const vec2& max) {
	return Rectangle2D(min, max - min);
}

//Checks if point is on line
bool pointOnLine(const Point2D& point, const Line2D& line) {
	float dy = (line.end.y - line.start.y); //delta y
	float dx = (line.end.x - line.start.x); //delta x
	float A = dy / dx; //y = ax + b
	float B = line.start.y - A * line.start.x;
	return CMP(point.y, A * point.x + B);
}

//Checks if point is in circle
bool pointInCircle(const Point2D& point, const Circle& circle) {
	Line2D line(point, circle.center);
	if (lengthSq(line) < circle.radius * circle.radius) return true;
	return false;
}

//Checks if point is in rectangle
bool pointInRectangle(const Point2D& point, const Rectangle2D& rect) {
	vec2 min = getMin(rect);
	vec2 max = getMax(rect);
	return min.x <= point.x && min.y <= point.y &&
		point.x <= max.x && point.y <= max.y;
}

//Checks if point is in oriented rectangle
bool pointInOrientedRect(const Point2D& point, const OrientedRectangle& rect) {
	vec2 rotationVec = point - rect.center; //vector from center to point which we rotate
	float theta = -DEG2RAD(rect.rotation);	//calculate angle from rectangle rotation
	float zRotation[] = {	//rotation matrix
		cosf(theta), sinf(theta), -sinf(theta), cosf(theta)
	};
	//multiply matrices to rotate
	multiply(rotationVec.asArray, vec2(rotationVec.x, rotationVec.y).asArray, 1, 2, zRotation, 2, 2);
	Rectangle2D localRect(Point2D(), rect.extents * 2.0f);
	//local rect in given as point and size vector, need to add half-extents to rotation vector
	vec2 localPoint = rotationVec + rect.extents;
	return pointInRectangle(localPoint, localRect);
}

//Checks if line intersects circle
bool lineCircleIntxn(const Line2D& line, const Circle& circle) {
	vec2 ab = line.end - line.start;
	//project vector from start of line to center of circle and normalize
	float t = dot(circle.center - line.start, ab) / dot(ab, ab);
	if (t < 0.0f || t > 1.0f) return false;
	Point2D closestPoint = line.start + ab * t;
	Line2D circleToClosest(circle.center, closestPoint);
	return lengthSq(circleToClosest) < circle.radius * circle.radius;
}

//Checks if line intersects rectangle
bool lineRectIntxn(const Line2D& line, const Rectangle2D& rect) {
	if (pointInRectangle(line.start, rect) || pointInRectangle(line.end, rect)) return true;
	vec2 norm = normalized(line.end - line.start);
	norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
	norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
	//vectors to minimum and maximum divided by normalized vector
	vec2 min = (getMin(rect) - line.start) * norm;
	vec2 max = (getMax(rect) - line.start) * norm;
	//get largest minimum and smallest maximum
	float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
	float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));
	if (tmax < 0 || tmin > tmax) return false; //line is not intersecting
	float t = (tmin < 0.0f) ? tmax : tmin; //check case if line starts inside rect
	return t > 0.0f && t * t < lengthSq(line);
}

//Checks if line intersects oriented rectangle
bool lineOrientedRectIntxn(const Line2D& line, const OrientedRectangle& rect) {
	float theta = -DEG2RAD(rect.rotation);
	float zRotation[] = {
		cosf(theta), sinf(theta), -sinf(theta), cosf(theta)
	};
	Line2D localLine;
	//rotate line to rectangle local space
	vec2 rotatonVec = line.start - rect.center;
	multiply(rotatonVec.asArray, vec2(rotatonVec.x, rotatonVec.y).asArray, 1, 2, zRotation, 2, 2);
	localLine.start = rotatonVec + rect.extents;

	rotatonVec = line.end - rect.center;
	multiply(rotatonVec.asArray, vec2(rotatonVec.x, rotatonVec.y).asArray, 1, 2, zRotation, 2, 2);
	localLine.end = rotatonVec + rect.extents;

	Rectangle2D localRect(Point2D(), rect.extents * 2.0f);
	return lineRectIntxn(localLine, localRect);
}

//Checks for collision of two circles
bool circleCircleCol(const Circle& c1, const Circle& c2) {
	Line2D line(c1.center, c2.center);
	float sum = c1.radius + c2.radius;
	return lengthSq(line) <= sum * sum;
}

//Checks for collision of circle and rectangle
bool circleRectCol(const Circle& circle, const Rectangle2D& rect) {
	vec2 min = getMin(rect);
	vec2 max = getMax(rect);
	Point2D closest = circle.center;
	closest.x = (closest.x < min.x) ? min.x : closest.x;
	closest.x = (closest.x > max.x) ? max.x : closest.x;
	closest.y = (closest.y < min.y) ? min.y : closest.y;
	closest.y = (closest.y > max.y) ? max.y : closest.y;
	Line2D line(circle.center, closest);
	return lengthSq(line) <= circle.radius * circle.radius;
}

//Checks for collision of circle and oriented rectangle
bool circleOrientedRectCol(const Circle& circle, const OrientedRectangle& rect) {
	vec2 r = circle.center - rect.center;
	float theta = -DEG2RAD(rect.rotation);
	float zRotation[] = {
		cosf(theta), sinf(theta), -sinf(theta), cosf(theta)
	};
	//rotate line r to local space
	multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation, 2, 2);
	Circle localCircle(r + rect.extents, circle.radius);
	Rectangle2D localRect(Point2D(), rect.extents * 2.0f);
	return circleRectCol(localCircle, localRect);
}

//Checks for collision of two rectangles
bool rectRectCol(const Rectangle2D& rect1, const Rectangle2D& rect2) {
	vec2 aMin = getMin(rect1);
	vec2 aMax = getMax(rect1);
	vec2 bMin = getMin(rect2);
	vec2 bMax = getMax(rect2);
	bool xOverlap = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
	bool yOverlap = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));
	return xOverlap && yOverlap;
}

//Checks for collision of two rectangles using SAT test
bool rectRectColSAT(const Rectangle2D& rect1, const Rectangle2D& rect2) {
	vec2 axisToTest[] = { vec2(1, 0), vec2(0, 1) };
	for (int i = 0; i < 2; ++i) {
		if (!overlapOnAxis(rect1, rect2, axisToTest[i])) return false;
	}
	return true;
}

//Checks for collision of rectangle and oriented rectangle using SAT test
bool rectOrientedRectColSAT(const Rectangle2D& rect1, const OrientedRectangle& rect2) {
	vec2 axisToTest[] = { vec2(1, 0), vec2(0, 1), vec2(), vec2() };
	float theta = DEG2RAD(rect2.rotation);
	float zRotation[] = {
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	//get axes of oriented rectangle
	vec2 axis = normalized(vec2(rect2.extents.x, 0));
	multiply(axisToTest[2].asArray, axis.asArray, 1, 2, zRotation, 2, 2);
	axis = normalized(vec2(0, rect2.extents.y));
	multiply(axisToTest[3].asArray, axis.asArray, 1, 2, zRotation, 2, 2);
	//SAT test
	for (int i = 0; i < 4; ++i) {
		if (!overlapOnAxis(rect1, rect2, axisToTest[i])) return false;
	}
	return true;
}

//Checks for collision of two oriented rectangles using SAT test
bool orientedRectOrientedRectCol(const OrientedRectangle& rect1, const OrientedRectangle& rect2) {
	Rectangle2D local1(Point2D(), rect1.extents * 2.0f);
	vec2 r = rect2.center - rect1.center;
	OrientedRectangle local2(rect2.center, rect2.extents, rect2.rotation);
	local2.rotation = rect2.rotation - rect1.rotation;
	float theta = -DEG2RAD(rect1.rotation);
	float zRotation[] = {
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	//rotate r vector (essentially one rectangle) to local space of one of oriented rectangles
	multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation, 2, 2);
	local2.center = r + rect1.extents;
	return rectOrientedRectColSAT(local1, local2);
}

//Returns containing circle for set of points
Circle containingCircle(Point2D* points, int count) {
	Point2D center;
	//get average of points as center
	for (int i = 0; i < count; ++i) {
		center = center + points[i];
	}
	center = center * (1.0f / (float) count);
	Circle result(center, 1.0f);
	//get maximum distance to points
	result.radius = magnitudeSq(center - points[0]);
	for (int i = 1; i < count; ++i) {
		float dist = magnitudeSq(center - points[i]);
		if (dist > result.radius) result.radius = dist;
	}
	result.radius = sqrtf(result.radius);
	return result;
}

//Returns containing rectangle for set of points
Rectangle2D containingRectangle(Point2D* points, int count) {
	vec2 min = points[0];
	vec2 max = points[0];
	for (int i = 1; i < count; ++i) {
		min.x = points[i].x < min.x ? points[i].x : min.x;
		min.y = points[i].y < min.y ? points[i].y : min.y;
		max.x = points[i].x > max.x ? points[i].x : max.x;
		max.y = points[i].y > max.y ? points[i].y : min.y;
	}
	return fromMinMax(min, max);
}

//Checks if point is inside bounding shape
bool pointInShape(const BoundingShape& shape, const Point2D& point) {
	for (int i = 0; i < shape.circleNum; ++i) {
		if (pointInCircle(point, shape.circles[i])) return true;
	}
	for (int i = 0; i < shape.rectNum; ++i){
		if (pointInRectangle(point, shape.rectangles[i])) return true;
	}
	return false;
}