#pragma once

#include "vectors.h"

//2D geometry definitions

//2-dimensional vector to point redefinition
typedef vec2 Point2D;

//2-dimensional line
typedef struct Line2D {
	Point2D start;
	Point2D end;

	inline Line2D(){}
	inline Line2D(const Point2D& sp, const Point2D& ep) : start(sp), end(ep){}

} Line2D;

//circle shape
typedef struct Circle {
	Point2D center;
	float radius;

	inline Circle() : radius(1.0f){}
	inline Circle(const Point2D& cp, float rad) : center(cp), radius(rad){}

} Circle;

//rectangle shape
typedef struct Rectangle2D {
	Point2D origin;
	vec2 size;

	inline Rectangle2D() : size(1,1){}
	inline Rectangle2D(const Point2D& point, const vec2& sv) : origin(point), size(sv){}

} Rectangle2D;

//oriented rectangle
typedef struct OrientedRectangle {
	Point2D center;
	vec2 extents;
	float rotation;	//rotation in degrees

	inline OrientedRectangle() : extents(1.0f, 1.0f), rotation(0.0f){}
	inline OrientedRectangle(const Point2D& point, const vec2& size) 
		: center(point), extents(size), rotation(0.0f){}
	inline OrientedRectangle(const Point2D& point, const vec2& size, float rot) 
		: center(point), extents(size), rotation(rot) {}

} OrientedRectangle;

//interval structure for Separating Axis Theorem tests
typedef struct Interval2D {
	float min;
	float max;
} Interval2D;

//bounding shape
typedef struct BoundingShape {
	Circle* circles;
	int circleNum;
	Rectangle2D* rectangles;
	int rectNum;

	inline BoundingShape() : circles(0), circleNum(0), rectangles(0), rectNum(0) {}
};

//2D geometry functions

//interval getter and overlap test

/**
Returns interval structure for rectangle

@param rect Rectangle to project onto axis
@param axis Axis to project rect onto
@return Interval struct for projection
*/
Interval2D getInterval(const Rectangle2D& rect, const vec2& axis);

/**
Returns interval structure for oriented rectangle

@param rect Oriented rectangle to project onto axis
@param axis Axis to project rect onto
@return Interval struct for projection
*/
Interval2D getInterval(const OrientedRectangle& rect, const vec2& axis);

/**
Check for rectangles overlapping on axis

@param rect1 First rectangle to project onto axis
@param rect2 Second rectangle to project onto axis
@param axis Axis to project rectangles onto
@return true if rectangles overlap on axis
*/
bool overlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2, const vec2& axis);

/**
Check for rectangles overlapping on axis

@param rect1 Rectangle to project onto axis
@param rect2 Oriented rectangle to project onto axis
@param axis Axis to project rectangles onto
@return true if rectangles overlap on axis
*/
bool overlapOnAxis(const Rectangle2D& rect1, const OrientedRectangle& rect2, const vec2& axis);

//length of line functions

/**
Returns length of a line

@param line 2-dimensional line
@return Length of a line
*/
float length(const Line2D& line);

/**
Returns squared length of a line

@param line 2-dimensional line
@return Squared length of a line
*/
float lengthSq(const Line2D& line);

//min and max point getters

/**
Returns minimum vertex of rectangle

@param rect Rectangle
@return Minimum vertex of rectangle
*/
vec2 getMin(const Rectangle2D& rect);

/**
Returns maximum vertex of rectangle

@param rect Rectangle
@return Maximum vertex of rectangle
*/
vec2 getMax(const Rectangle2D& rect);

/**
Constructs rectangle out of minimum and maximum points

@param min Minimum point/vertex of rectangle
@param max Maximum point/vertex of rectangle
@return Constructed rectangle
*/
Rectangle2D fromMinMax(const vec2& min, const vec2& max);

//point containment functions

/**
Checks if point is on line

@param point Point to check
@param line Line to check
@return true if point is on line, false otherwise
*/
bool pointOnLine(const Point2D& point, const Line2D& line);

/**
Checks if point is in circle

@param point Point to check
@param circle Circle to check
@return true if point is on circle, false otherwise
*/
bool pointInCircle(const Point2D& point, const Circle& circle);

/**
Checks if point is in rectangle

@param point Point to check
@param rect Rectangle to check
@return true if point is on rectangle, false otherwise
*/
bool pointInRectangle(const Point2D& point, const Rectangle2D& rect);

/**
Checks if point is in oriented rectangle

@param point Point to check
@param rect Oriented rectangle to check
@return true if point is in oriented rectangle, false otherwise
*/
bool pointInOrientedRect(const Point2D& point, const OrientedRectangle& rect);

//line intersections functions

/**
Checks if line intersects circle

@param line Line to check
@param circle Circle to check
@return true if line intersects circle, false otherwise
*/
bool lineCircleIntxn(const Line2D& line, const Circle& circle);

/**
Checks if line intersects rectangle

@param line Line to check
@param rect Rectangle to check
@return true if line intersects rectangle, false otherwise
*/
bool lineRectIntxn(const Line2D& line, const Rectangle2D& rect);

/**
Checks if line intersects oriented rectangle

@param line Line to check
@param rect Oriented rectangle to check
@return true if line intersects oriented rectangle, false otherwise
*/
bool lineOrientedRectIntxn(const Line2D& line, const OrientedRectangle& rect);

//geometry collision functions

/**
Checks for collision of two circles

@param c1 First circle to check
@param c2 Second circle to check
@return true if two circles collide
*/
bool circleCircleCol(const Circle& c1, const Circle& c2);

/**
Checks for collision of circle and rectangle

@param circle Circle to check
@param rect Rectangle to check
@return true if circle and rectangle collide
*/
bool circleRectCol(const Circle& circle, const Rectangle2D& rect);

/**
Checks for collision of circle and oriented rectangle

@param circle Circle to check
@param rect Oriented rectangle to check
@return true if circle and oriented rectangle collide
*/
bool circleOrientedRectCol(const Circle& circle, const OrientedRectangle& rect);

/**
Checks for collision of two rectangles

@param rect1 First rectangle to check
@param rect2 Second rectangle to check
@return true if two rectangles collide
*/
bool rectRectCol(const Rectangle2D& rect1, const Rectangle2D& rect2);

/**
Checks for collision of two rectangles using SAT test

@param rect1 First rectangle to check
@param rect2 Second rectangle to check
@return true if two rectangles collide
*/
bool rectRectColSAT(const Rectangle2D& rect1, const Rectangle2D& rect2);

/**
Checks for collision of rectangle and oriented rectangle using SAT test

@param rect1 Rectangle to check
@param rect2 Oriented rectangle to check
@return true if two rectangles collide
*/
bool rectOrientedRectColSAT(const Rectangle2D& rect1, const OrientedRectangle& rect2);

/**
Checks for collision of two oriented rectangles using SAT test

@param rect1 First oriented rectangle to check
@param rect2 Second oriented rectangle to check
@return true if two oriented rectangles collide
*/
bool orientedRectOrientedRectCol(const OrientedRectangle& rect1, const OrientedRectangle& rect2);

//geometry bounding shapes functions

/**
Returns containing circle for set of points

@param points Set of points to contain
@param count Number of points
@return Containing circle for given set
*/
Circle containingCircle(Point2D* points, int count);

/**
Returns containing rectangle for set of points

@param points Set of points to contain
@param count Number of points
@return Containing rectangle for given set
*/
Rectangle2D containingRectangle(Point2D* points, int count);

/**
Checks if point is inside bounding shape

@param shape Bounding shape to check
@param point Point to check
@return true if point is inside bounding shape
*/
bool pointInShape(const BoundingShape& shape, const Point2D& point);