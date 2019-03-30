#pragma once

//macro definitons of radians <-> degrees functions

//radians to degrees macro function
#define RAD2DEG(x) ((x) * 57.295754f)
//degrees to radians macro function
#define DEG2RAD(x) ((x) * 0.0174533f)

//2-dimensional vector
typedef struct vec2 {
	union {
		struct 
		{
			float x;
			float y;
		};
		float asArray[2];
	};

	float& operator[](int i) {
		return asArray[i];
	}
	
	vec2() : x(0.0f), y(0.0f) {}
	vec2(float _x, float _y) : x(_x), y(_y) {}

} vec2;

//3-dimensional vector
typedef struct vec3 {
	union {
		struct
		{
			float x;
			float y;
			float z;
		};
		float asArray[3];
	};

	float& operator[](int i) {
		return asArray[i];
	}

	vec3() : x(0.0f), y(0.0f), z(0.0f) {}
	vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

} vec3;

//vector operations declarations
//vector sum

//2-dimensional vector - vector sum
vec2 operator+(const vec2& l, const vec2& r);

//3-dimensional vector - vector sum
vec3 operator+(const vec3& l, const vec3& r);

//vector subtraction

//2-dimensional vector - vector difference
vec2 operator-(const vec2& l, const vec2& r);

//3-dimensional vector - vector difference
vec3 operator-(const vec3& l, const vec3& r);

//vector-vector multiplication

//2-dimensional vector - vector multiplication
vec2 operator*(const vec2& l, const vec2& r);

//3-dimensional vector - vector multiplication
vec3 operator*(const vec3& l, const vec3& r);

//vector-scalar multiplication

//2-dimensional vector - scalar multiplication
vec2 operator*(const vec2& l, float r);

//3-dimensional vector - scalar multiplication
vec3 operator*(const vec3& l, float r);

//vector comparison

//2-dimensional vector - vector comparison
bool operator==(const vec2& l, const vec2& r);

//3-dimensional vector - vector comparison
bool operator==(const vec3& l, const vec3& r);

//2-dimensional vector - vector distinction
bool operator!=(const vec2& l, const vec2& r);

//3-dimensional vector - vector distinction
bool operator!=(const vec3& l, const vec3& r);

//vector dot product

/**
Computes dot product of two 2-dimensional vectors

@param l First vector
@param r Second vector
@return Result of dot product
*/
float dot(const vec2& l, const vec2& r);

/**
Computes dot product of two 3-dimensional vectors

@param l First vector
@param r Second vector
@return Result of dot product
*/
float dot(const vec3& l, const vec3& r);

//vector magnitude

/**
Computes magnitude of 2-dimensional vector

@param v 2-dimensional vector
@return Magnitude of vector
*/
float magnitude(const vec2& v);

/**
Computes magnitude of 3-dimensional vector

@param v 3-dimensional vector
@return Magnitude of vector
*/
float magnitude(const vec3& v);

/**
Computes squared magnitude of 2-dimensional vector

@param v 2-dimensional vector
@return Squared magnitude of vector
*/
float magnitudeSq(const vec2& v);

/**
Computes squared magnitude of 2-dimensional vector

@param v 2-dimensional vector
@return Squared magnitude of vector
*/
float magnitudeSq(const vec3& v);

//vector normalization

/**
Normalizes 2-dimensional vector

@param v 2-dimensional vector
*/
void normalize(vec2& v);

/**
Normalizes 3-dimensional vector

@param v 3-dimensional vector
*/
void normalize(vec3& v);

/**
Normalizes 2-dimensional vector

@param v 2-dimensional vector
@return New vector equal to normalized source vector
*/
vec2 normalized(const vec2& v);

/**
Normalizes 3-dimensional vector

@param v 2-dimensional vector
@return New vector equal to normalized source vector
*/
vec3 normalized(const vec3& v);

//3-dimensional vector cross product

/**
Computes cross product of two vectors

@param l First vector
@param r Second vector
@return New vector equal to cross product of l and r
*/
vec3 cross(const vec3& l, const vec3& r);

//functions computing angle between vectors

/**
Computes angle between two 2-dimensional vectors

@param l First vector
@param r Second vector
@return Angle between vectors in radians
*/
float angle(const vec2& l, const vec2& r);

/**
Computes angle between two 3-dimensional vectors

@param l First vector
@param r Second vector
@return Angle between vectors in radians
*/
float angle(const vec3& l, const vec3& r);

//vector projection functions

/**
Computes 2-dimensional vector projected onto another

@param length Vector that is being projected
@param direction Vector that is projected onto
@return New vector equal to projection of length onto direction
*/
vec2 project(const vec2& length, const vec2& direction);

/**
Computes 3-dimensional vector projected onto another

@param length Vector that is being projected
@param direction Vector that is projected onto
@return New vector equal to projection of length onto direction
*/
vec3 project(const vec3& length, const vec3& direction);

/**
Computes 2-dimensional vector perpendicular from one vector to another

@param length Vector which perpendicular component is being computed
@param direction Vector that result will be perpendicular to
@return New vector perpendicular to direction vector
*/
vec2 perpendicular(const vec2& length, const vec2& direction);

/**
Computes 3-dimensional vector perpendicular from one vector to another

@param length Vector which perpendicular component is being computed
@param direction Vector that result will be perpendicular to
@return New vector perpendicular to direction vector
*/
vec3 perpendicular(const vec3& length, const vec3& direction);

//vector reflection functions

/**
Computes 2-dimensional reflected vector

@param vector Vector which reflection will be computed
@param normal Normal of the surface which is being reflected from
@return New vector equal to the vector reflection from the surface
*/
vec2 reflection(const vec2& vector, const vec2& normal);

/**
Computes 3-dimensional reflected vector

@param vector Vector which reflection will be computed
@param normal Normal of the surface which is being reflected from
@return New vector equal to the vector reflection from the surface
*/
vec3 reflection(const vec3& vector, const vec3& normal);