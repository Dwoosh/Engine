#pragma once

#include "vectors.h"
#include "matrices.h"
#include <vector>

//3D geometry definitions

//3-dimensional vector to point redefinition
typedef vec3 Point;

//3-dimensional line
typedef struct Line {
	Point start;
	Point end;

	inline Line(){}
	inline Line(const Point& s, const Point& e) : start(s), end(e) {}

} Line;

//ray (point and direction vector)
typedef struct Ray {
	Point origin;
	vec3 direction;

	inline Ray() : direction(0.0f,0.0f,1.0f){}
	inline Ray(const Point& o, const vec3& d) : origin(o), direction(d) { normalizeDirection(); }
	inline void normalizeDirection() { normalize(direction); }

} Ray;

//sphere
typedef struct Sphere {
	Point center;
	float radius;

	inline Sphere() : radius(1.0f) {}
	inline Sphere(const Point& c, float r) : center(c), radius(r) {}

} Sphere;

//axis-aligned bounding box
typedef struct AABB {
	Point center;
	vec3 size;

	inline AABB() : size(1,1,1) {}
	inline AABB(const Point& c, const vec3& s) : center(c), size(s) {}

} AABB;

//oriented bounding box
typedef struct OBB {
	Point center;
	vec3 size;
	mat3 orientation;

	inline OBB() : size(1,1,1){}
	inline OBB(const Point& c, const vec3& s) : center(c), size(s) {}
	inline OBB(const Point& c, const vec3& s, const mat3& o) 
		: center(c), size(s), orientation(o) {}

} OBB;

//plane (normal vector and distance from origin)
typedef struct Plane {
	vec3 normal;
	float distance;

	inline Plane() : normal(0,1,0){}
	inline Plane(const vec3& n, float d) : normal(n), distance(d) {}

} Plane;

//3-dimensional triangle
typedef struct Triangle {
	union {
		struct {
			Point a;
			Point b;
			Point c;
		};
		Point points[3];
		float values[9];
	};

	inline Triangle(){}
	inline Triangle(const Point& p1, const Point& p2, 
		const Point& p3) : a(p1), b(p2), c(p3) {}

} Triangle;

//bounding volume hierarchy object node
typedef struct BVHNode {
	AABB bounds;
	BVHNode* children;
	int triangleNum;
	int* triangles;

	BVHNode() : children(0), triangleNum(0), triangles(0) {}
} BVHNode;

//mesh object
typedef struct Mesh {
	int triangleNum;
	union {
		Triangle* triangles; //size = triangleNum
		Point* vertices; //size = triangleNum * 3
		float* values; //size = triangleNum * 9
	};
	BVHNode* accelerator;

	Mesh() : triangleNum(0), values(0), accelerator(0) {}
} Mesh;

//interval of an object projected onto axis
typedef struct Interval {
	float min;
	float max;
} Interval;

//model object
class Model {
protected:
	Mesh* contents;
	AABB bounds;
public:
	vec3 position;
	vec3 rotation;
	Model* parent;

	inline Model() : parent(0), contents(0){}
	inline Mesh* getMesh() const {
		return contents;
	}
	inline AABB getBounds() const {
		return bounds;
	}

	//model contents setter
	void setContents(Mesh* mesh);
};

#undef near
#undef far

//frustum object
typedef struct Frustum {
	union{
		struct {
			Plane top;
			Plane bottom;
			Plane left;
			Plane right;
			Plane near;
			Plane far;
		};
		Plane planes[6];
	};

	inline Frustum(){}

}Frustum;

//raycast result structure
typedef struct RaycastResult {
	vec3 point;
	vec3 normal;
	float t;
	bool hit;
} RaycastResult;

typedef struct CollisionManifold {
	bool colliding;
	vec3 normal;
	float depth;
	std::vector<vec3> contacts;
} CollisionManifold;

//3D geometry functions
//line length functions

/**
Calculates length of a line

@param line 3-dimensional line
@return length of a line
*/
float length(const Line& line);

/**
Calculates squared length of a line

@param line 3-dimensional line
@return squared length of a line
*/
float lengthSq(const Line& line);

//ray from points constructor

/**
Creates ray from two given points

@param from Point that will serve as origin for ray
@param to Point to which ray will be directed
@return constructed ray
*/
Ray fromPoints(const Point& from, const Point& to);

//AABB functions

/**
Returns minimum point of AABB

@param box AABB
@return minimum point given as vector
*/
vec3 getMin(const AABB& box);

/**
Returns maximum point of AABB

@param box AABB
@return maximum point given as vector
*/
vec3 getMax(const AABB& box);

/**
Returns AABB constructed from two points

@param min Minimum point of AABB
@param max Maximum point of AABB
@return AABB constructed from points
*/
AABB fromMinMax(const vec3& min, const vec3& max);

//plane functions

/**
Returns result of plane equation

@param p Point applied to plane equation
@param plane Plane which equation we calculate
@return Distance of a point from plane. 0 if point is on the plane,
positive if point is in front of plane, negative if point is behind plane
*/
float planeEq(const Point& p, const Plane& plane);

//point containment functions

/**
Checks if point is in sphere

@param point Point to check
@param sphere Sphere to check
@return true if point is in sphere, false otherwise
*/
bool pointInSphere(const Point& point, const Sphere& sphere);

/**
Checks if point is in AABB

@param point Point to check
@param aabb AABB to check
@return true if point is in AABB, false otherwise
*/
bool pointInAABB(const Point& point, const AABB& aabb);

/**
Checks if point is in OBB

@param point Point to check
@param obb OBB to check
@return true if point is in obb, false otherwise
*/
bool pointInOBB(const Point& point, const OBB& obb);

/**
Checks if point is on plane

@param point Point to check
@param plane Plane to check
@return true if point is on plane, false otherwise
*/
bool pointOnPlane(const Point& point, const Plane& plane);

/**
Checks if point is on line

@param point Point to check
@param line Line to check
@return true if point is on line, false otherwise
*/
bool pointOnLine(const Point& point, const Line& line);

/**
Checks if point is on ray

@param point Point to check
@param ray Ray to check
@return true if point is on ray, false otherwise
*/
bool pointOnRay(const Point& point, const Ray& ray);

/**
Returns sphere's closest point to given point

@param point Point to which function returns closest point
@param sphere Sphere that holds closest point
@return Closest point on sphere to given point
*/
Point closestPoint(const Point& point, const Sphere& sphere);

/**
Returns AABB's closest point to given point

@param point Point to which function returns closest point
@param aabb AABB that holds closest point
@return Closest point on AABB to given point
*/
Point closestPoint(const Point& point, const AABB& aabb);

/**
Returns OBB's closest point to given point

@param point Point to which function returns closest point
@param obb OBB that holds closest point
@return Closest point on OBB to given point
*/
Point closestPoint(const Point& point, const OBB& obb);

/**
Returns plane's closest point to given point

@param point Point to which function returns closest point
@param plane Plane that holds closest point
@return Closest point on plane to given point
*/
Point closestPoint(const Point& point, const Plane& plane);

/**
Returns line's closest point to given point

@param point Point to which function returns closest point
@param line Line that holds closest point
@return Closest point on line to given point
*/
Point closestPoint(const Point& point, const Line& line);

/**
Returns ray's closest point to given point

@param point Point to which function returns closest point
@param ray Ray that holds closest point
@return Closest point on ray to given point
*/
Point closestPoint(const Point& point, const Ray& ray);

//interval getter and overlap functions

/**
Returns interval of AABB projected onto axis

@param aabb AABB which is projected onto axis
@param axis Axis to project onto
@return Interval of projected AABB
*/
Interval getInterval(const AABB& aabb, const vec3& axis);

/**
Returns interval of OBB projected onto axis

@param obb OBB which is projected onto axis
@param axis Axis to project onto
@return Interval of projected OBB
*/
Interval getInterval(const OBB& obb, const vec3& axis);

/**
Checks if two objects overlap on axis

@param aabb Checked AABB
@param obb Checked OBB
@param axis Axis onto which objects are projected
@return true if objects overlap on given axis, false otherwise
*/
bool overlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis);

/**
Checks if two objects overlap on axis

@param obb1 First checked OBB
@param obb2 Second checked OBB
@param axis Axis onto which objects are projected
@return true if objects overlap on given axis, false otherwise
*/
bool overlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis);

//shape intersection functions

/**
Checks if two spheres intersect

@param s1 First checked sphere
@param s2 Second checked sphere
@return true if spheres intersect, false otherwise
*/
bool sphereSphereIntxn(const Sphere& s1, const Sphere& s2);

/**
Checks if sphere and AABB intersect

@param sphere Checked sphere
@param aabb Checked AABB
@return true if objects intersect, false otherwise
*/
bool sphereAABBIntxn(const Sphere& sphere, const AABB& aabb);

/**
Checks if sphere and OBB intersect

@param sphere Checked sphere
@param obb Checked OBB
@return true if objects intersect, false otherwise
*/
bool sphereOBBIntxn(const Sphere& sphere, const OBB& obb);

/**
Checks if sphere and plane intersect

@param sphere Checked sphere
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool spherePlaneIntxn(const Sphere& sphere, const Plane& plane);

/**
Checks if two AABBs intersect

@param aabb1 First checked AABB
@param aabb2 Second checked AABB
@return true if objects intersect, false otherwise
*/
bool AABBAABBIntxn(const AABB& aabb1, const AABB& aabb2);

/**
Checks if AABB and OBB intersect using SAT test

@param aabb Checked AABB
@param obb Checked OBB
@return true if objects intersect, false otherwise
*/
bool AABBOBBIntxn(const AABB& aabb, const OBB& obb);

/**
Checks if AABB and plane intersect

@param aabb Checked AABB
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool AABBPlaneIntxn(const AABB& aabb, const Plane& plane);

/**
Checks if two OBBs intersect using SAT test

@param obb1 First checked OBB
@param obb2 Second checked OBB
@return true if objects intersect, false otherwise
*/
bool OBBOBBIntxn(const OBB& obb1, const OBB& obb2);

/**
Checks if OBB and plane intersect

@param obb Checked OBB
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool OBBPlaneIntxn(const OBB& obb, const Plane& plane);

/**
Checks if two planes intersect

@param p1 First checked plane
@param p2 Second checked plane
@return true if objects intersect, false otherwise
*/
bool planePlaneIntxn(const Plane& p1, const Plane& p2);

//line intersection functions

/**
Raycasts given ray on sphere

@param sphere Sphere which is raycasted
@param ray Ray to use
@return -1 if theres no collision, distance/time from 
ray origin at which collision happens otherwise
*/
float raycast(const Sphere& sphere, const Ray& ray);

/**
Raycasts given ray on AABB using Cyrus-Beck clipping algorithm

@param aabb AABB which is raycasted
@param ray Ray to use
@return -1 if theres no collision, distance/time from
ray origin at which collision happens otherwise
*/
float raycast(const AABB& aabb, const Ray& ray);

/**
Raycasts given ray on OBB using Cyrus-Beck clipping algorithm

@param obb OBB which is raycasted
@param ray Ray to use
@return -1 if theres no collision, distance/time from
ray origin at which collision happens otherwise
*/
float raycast(const OBB& obb, const Ray& ray);

/**
Raycasts given ray on plane

@param plane Plane which is raycasted
@param ray Ray to use
@return -1 if theres no collision or if ray hits behind plane,
distance/time from ray origin at which collision happens otherwise
*/
float raycast(const Plane& plane, const Ray& ray);

/**
Checks if line and sphere intersect

@param sphere Checked sphere
@param line Checked line
@return true if line and sphere intersect
*/
bool linetest(const Sphere& sphere, const Line& line);

/**
Checks if line and AABB intersect

@param abb Checked AABB
@param line Checked line
@return true if line and AABB intersect
*/
bool linetest(const AABB& aabb, const Line& line);

/**
Checks if line and OBB intersect

@param obb Checked OBB
@param line Checked line
@return true if line and OBB intersect
*/
bool linetest(const OBB& obb, const Line& line);

/**
Checks if line and plane intersect

@param plane Checked plane
@param line Checked line
@return true if line and plane intersect
*/
bool linetest(const Plane& plane, const Line& line);

//triangle related functions

/**
Checks if point is in triangle

@param p Checked point
@param t Checked triangle
@return true if point is in triangle
*/
bool pointInTriangle(const Point& p, const Triangle& t);

/**
Creates plane from triangle

@param t Triangle
@return Constructed plane
*/
Plane planeFromTriangle(const Triangle& t);

/**
Returns Triangle's closest point to given point

@param point Point to which function returns closest point
@param t Triangle that holds closest point
@return Closest point on Triangle to given point
*/
Point closestPoint(const Point& point, const Triangle& t);

/**
Checks if triangle and sphere intersect

@param t Checked triangle
@param sphere Checked sphere
@return true if objects intersect, false otherwise
*/
bool triangleSphereIntxn(const Triangle& t, const Sphere& sphere);

/**
Returns interval of triangle projected onto axis

@param t Triangle which is projected onto axis
@param axis Axis to project onto
@return Interval of projected triangle
*/
Interval getInterval(const Triangle& t, const vec3& axis);

/**
Checks if two objects overlap on axis

@param t Checked triangle
@param aabb Checked AABB
@param axis Axis onto which objects are projected
@return true if objects overlap on given axis, false otherwise
*/
bool overlapOnAxis(const Triangle& t, const AABB& aabb, const vec3& axis);

/**
Checks if triangle and AABB intersect using SAT test

@param t Checked triangle
@param aabb Checked AABB
@return true if objects intersect, false otherwise
*/
bool triangleAABBIntxn(const Triangle& t, const AABB& aabb);

/**
Checks if two objects overlap on axis

@param t Checked triangle
@param obb Checked OBB
@param axis Axis onto which objects are projected
@return true if objects overlap on given axis, false otherwise
*/
bool overlapOnAxis(const Triangle& t, const OBB& obb, const vec3& axis);

/**
Checks if triangle and OBB intersect using SAT test

@param t Checked triangle
@param obb Checked OBB
@return true if objects intersect, false otherwise
*/
bool triangleOBBIntxn(const Triangle& t, const OBB& obb);

/**
Checks if triangle and plane intersect

@param t Checked triangle
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool trianglePlaneIntxn(const Triangle& t, const Plane& plane);

/**
Checks if two objects overlap on axis

@param t1 First checked triangle
@param t2 econd checked triangle
@param axis Axis onto which objects are projected
@return true if objects overlap on given axis, false otherwise
*/
bool overlapOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis);

/**
Checks for cross product result of triangle edges equal to zero

@param a First edge point of first triangle
@param b Second edge point of first triangle
@param c First edge point of second triangle
@param d Second edge point of second triangle
@return returns cross product of edges if it is not 0, 
else tries the same with cross product of edge and vector 
perpendicular to both edges, else returns zero vector
*/
vec3 crossEdgeSAT(const vec3& a, const vec3& b, const vec3& c, const vec3& d);

/**
Checks if two triangles intersect using SAT test

@param t1 First checked triangle
@param t2 Second checked plane
@return true if objects intersect, false otherwise
*/
bool triangleTriangleIntxn(const Triangle& t1, const Triangle& t2);

/**
Returns barycentric coordinates of a point with respect to triangle

@param p Point which barycentric coords will be returned
@param t Triangle to which point's coords will be relative to
@return Barycentric coords of point relative to triangle
*/
vec3 getBarycentric(const Point& p, const Triangle& t);

/**
Raycasts given ray on triangle

@param t Triangle which is raycasted
@param ray Ray to use
@return -1 if theres no collision, distance/time from
ray origin at which collision happens otherwise
*/
float raycast(const Triangle& t, const Ray& ray);

/**
Checks if line and triangle intersect

@param t Checked triangle
@param line Checked line
@return true if line and triangle intersect
*/
bool linetest(const Triangle& t, const Line& line);

//Bounding Volume Hierarchy related functions

/**
Creates BVHNode for given mesh

@param mesh Mesh to accelerate
*/
void accelerateMesh(Mesh& mesh);

/**
Recursively splits BVHNode until given depth is reached

@param node BVHNode to split
@param model Mesh model
@param depth Depth level to stop at
*/
void splitBVHNode(BVHNode* node, const Mesh& model, int depth);

/**
Deletes all children of given node recursively

@param node BVHNode which children are to be deleted
*/
void freeBVHNode(BVHNode* node);

//mesh intersection functions

/**
Checks if mesh and ray intersect

@param mesh Checked mesh
@param ray Checked ray
@return -1 if theres no collision or if ray hits behind plane,
distance/time from ray origin at which collision happens otherwise
*/
float meshRayIntxn(const Mesh& mesh, const Ray& ray);

/**
Checks if mesh and AABB intersect

@param mesh Checked mesh
@param aabb Checked AABB
@return true if objects intersect, false otherwise
*/
bool meshAABBIntxn(const Mesh& mesh, const AABB& aabb);

/**
Checks if mesh and OBB intersect

@param mesh Checked mesh
@param obb Checked obb
@return true if objects intersect, false otherwise
*/
bool meshOBBIntxn(const Mesh& mesh, const OBB& obb);

/**
Checks if mesh and plane intersect

@param mesh Checked mesh
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool meshPlaneIntxn(const Mesh& mesh, const Plane& plane);

/**
Checks if mesh and triangle intersect

@param mesh Checked mesh
@param t Checked triangle
@return true if objects intersect, false otherwise
*/
bool meshTriangleIntxn(const Mesh& mesh, const Triangle& t);

/**
Checks if mesh and line intersect

@param mesh Checked mesh
@param line Checked line
@return true if objects intersect, false otherwise
*/
bool meshLineIntxn(const Mesh& mesh, const Line& line);

/**
Checks if mesh and sphere intersect

@param mesh Checked mesh
@param sphere Checked sphere
@return true if objects intersect, false otherwise
*/
bool meshSphereIntxn(const Mesh& mesh, const Sphere& sphere);

//model functions

/**
Returns world matrix of the given model

@param model Model object
@return World matrix of model
*/
mat4 getWorldMatrix(const Model& model);

/**
Returns OBB surrounding model

@param model Model object
@return OBB that surrounds model
*/
OBB getOBB(const Model& model);

//model intersection functions

/**
Checks if model and ray intersect

@param model Checked model
@param ray Checked ray
@return -1 if theres no collision or if ray hits behind plane,
distance/time from ray origin at which collision happens otherwise
*/
float modelRayIntxn(const Model& model, const Ray& ray);

/**
Checks if model and AABB intersect

@param model Checked model
@param aabb Checked AABB
@return true if objects intersect, false otherwise
*/
bool modelAABBIntxn(const Model& model, const AABB& aabb);

/**
Checks if model and OBB intersect

@param model Checked model
@param obb Checked obb
@return true if objects intersect, false otherwise
*/
bool modelOBBIntxn(const Model& model, const OBB& obb);

/**
Checks if model and plane intersect

@param model Checked model
@param plane Checked plane
@return true if objects intersect, false otherwise
*/
bool modelPlaneIntxn(const Model& model, const Plane& plane);

/**
Checks if model and triangle intersect

@param model Checked model
@param t Checked triangle
@return true if objects intersect, false otherwise
*/
bool modelTriangleIntxn(const Model& model, const Triangle& t);

/**
Checks if model and line intersect

@param model Checked model
@param line Checked line
@return true if objects intersect, false otherwise
*/
bool modelLineIntxn(const Model& model, const Line& line);

/**
Checks if model and sphere intersect

@param model Checked model
@param sphere Checked sphere
@return true if objects intersect, false otherwise
*/
bool modelSphereIntxn(const Model& model, const Sphere& sphere);

//frustum related functions

/**
Computes intersection point of three planes using Cramers Rule

@param p1 First plane
@param p2 Second plane
@param p2 Third plane
@return Point of plane intersection
*/
Point planeIntxn(Plane p1, Plane p2, Plane p3);

/**
Computes all corners of a frustum

@param f Frustum object
@param output Output vector containing corners
*/
void getCorners(const Frustum& f, vec3* output);

/**
Checks if frustum and point intersect

@param f Checked frustum
@param p Checked point
@return true if objects intersect, false otherwise
*/
bool frustumPointIntxn(const Frustum& f, const Point& p);

/**
Checks if frustum and sphere intersect

@param f Checked frustum
@param s Checked sphere
@return true if objects intersect, false otherwise
*/
bool frustumSphereIntxn(const Frustum& f, const Sphere& s);

/**
Classifies whether AABB is behind, in front of plane or intersects it

@param aabb AABB object to classify
@param plane Plane to check
@return negative distance if AABB is behind plane,
positive if AABB is in front of plane, 0 if AABB intersects plane
*/
float classify(const AABB& aabb, const Plane& plane);

/**
Classifies whether OBB is behind, in front of plane or intersects it

@param obb OBB object to classify
@param plane Plane to check
@return negative distance if OBB is behind plane,
positive if OBB is in front of plane, 0 if OBB intersects plane
*/
float classify(const OBB& obb, const Plane& plane);

/**
Checks if frustum and AABB intersect

@param f Checked frustum
@param aabb Checked AABB
@return true if objects intersect, false otherwise
*/
bool frustumAABBIntxn(const Frustum& f, const AABB& aabb);

/**
Checks if frustum and OBB intersect

@param f Checked frustum
@param obb Checked OBB
@return true if objects intersect, false otherwise
*/
bool frustumOBBIntxn(const Frustum& f, const OBB& obb);

//geometry-screen interactions related functions

/**
Turns screen-space pixel into world-space vector

@param viewportPoint Point in viewport
@param viewportOrigin Origin of viewport
@param viewportSize Width and height of viewport
@param view View matrix
@param projection Projection matrix
@return Vector in world-space
*/
vec3 unproject(const vec3& viewportPoint, const vec2& viewportOrigin, 
	const vec2& viewportSize, const mat4& view, const mat4& projection);

/**
Returns picking ray from screen-space pixel

@param viewportPoint Point in viewport
@param viewportOrigin Origin of viewport
@param viewportSize Width and height of viewport
@param view View matrix
@param projection Projection matrix
@return Picking ray
*/
Ray getPickRay(const vec2& viewportPoint, const vec2& viewportOrigin,
	const vec2& viewportSize, const mat4& view, const mat4& projection);

//additional functions

//raycast result functions
/**
Resets raycast result structure

@param result RaycastResult structure
*/
void resetRaycastResult(RaycastResult* result);


bool raycast(const Sphere& sphere, const Ray& ray, RaycastResult* result);
bool raycast(const AABB& aabb, const Ray& ray, RaycastResult* result);
bool raycast(const OBB& obb, const Ray& ray, RaycastResult* result);
bool raycast(const Plane& plane, const Ray& ray, RaycastResult* result);
bool raycast(const Triangle& triangle, const Ray& ray, RaycastResult* result);

//collision manifold functions
/**
Resets collision manifold structure

@param result CollisionManifold structure
*/
void resetCollisionManifold(CollisionManifold* result);

CollisionManifold findCollisionFeatures(const Sphere& s1, const Sphere& s2);
CollisionManifold findCollisionFeatures(const OBB& obb, const Sphere& s);

std::vector<Point> getVertices(const OBB& obb);
std::vector<Line> getEdges(const OBB& obb);
std::vector<Plane> getPlanes(const OBB& obb);
bool clipToPlane(const Plane& plane, const Line& line, Point* outPoint);
std::vector<Point> clipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb);
float penetrationDepth(const OBB& obb1, const OBB& obb2, const vec3& axis, bool* outShouldFlip);
CollisionManifold findCollisionFeatures(const OBB& obb1, const OBB& obb2);