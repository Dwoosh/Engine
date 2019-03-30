#pragma once

#include "vectors.h"

//2x2 matrix
typedef struct mat2 {
	union {
		struct {
			float _11, _12,
				  _21, _22;
		};
		float asArray[4];
	};

	inline float* operator[](int i) {
		return &(asArray[i * 2]);
	}
	//default - identity matrix constructor
	inline mat2() {
		_11 = _22 = 1.0f;
		_12 = _21 = 0.0f;
	}

	inline mat2(float f11, float f12,
				float f21, float f22) {
		_11 = f11; _12 = f12;
		_21 = f21; _22 = f22;
	}

} mat2;

//3x3 matrix
typedef struct mat3 {
	union {
		struct {
			float _11, _12, _13,
				  _21, _22, _23,
				  _31, _32, _33;
		};
		float asArray[9];
	};

	inline float* operator[](int i) {
		return &(asArray[i * 3]);
	}
	//default - identity matrix constructor
	inline mat3() {
		_11 = _22 = _33 = 1.0f;
		_12 = _13 = _21 = 0.0f;
		_23 = _31 = _32 = 0.0f;
	}

	inline mat3(float f11, float f12, float f13,
				float f21, float f22, float f23, 
				float f31, float f32, float f33) {
		_11 = f11; _12 = f12; _13 = f13;
		_21 = f21; _22 = f22; _23 = f23;
		_31 = f31; _32 = f32; _33 = f33;
	}

} mat3;

//4x4 matrix
typedef struct mat4 {
	union {
		struct {
			float _11, _12, _13, _14,
				  _21, _22, _23, _24,
				  _31, _32, _33, _34,
				  _41, _42, _43, _44;
		};
		float asArray[16];
	};

	inline float* operator[](int i) {
		return &(asArray[i * 4]);
	}
	//default - identity matrix constructor
	inline mat4() {
		_11 = _22 = _33 = _44 = 1.0f;
		_12 = _13 = _14 = _21 = 0.0f;
		_23 = _24 = _31 = _32 = 0.0f;
		_34 = _41 = _42 = _43 = 0.0f;
	}

	inline mat4(float f11, float f12, float f13, float f14,
				float f21, float f22, float f23, float f24,
				float f31, float f32, float f33, float f34,
				float f41, float f42, float f43, float f44) {
		_11 = f11; _12 = f12; _13 = f13; _14 = f14;
		_21 = f21; _22 = f22; _23 = f23; _24 = f24;
		_31 = f31; _32 = f32; _33 = f33; _34 = f34;
		_41 = f41; _42 = f42; _43 = f43; _44 = f44;
	}

} mat4;

//matrix transposition functions

/**
Transposes matrix given as array and stores result in argument

@param srcMat Source matrix
@param destMat Destination matrix - transposed source
@param srcRows Number of rows in source matrix
@param srcCols Number of columns in source matrix
*/
void transpose(const float *srcMat, float *destMat, int srcRows, int srcCols);

/**
Transposes given 2x2 matrix

@param matrix 2x2 matrix
@return Constructed transposed matrix
*/
mat2 transpose(const mat2& matrix);

/**
Transposes given 3x3 matrix

@param matrix 3x3 matrix
@return Constructed transposed matrix
*/
mat3 transpose(const mat3& matrix);

/**
Transposes given 4x4 matrix

@param matrix 4x4 matrix
@return Constructed transposed matrix
*/
mat4 transpose(const mat4& matrix);

//matrix multiplication functions
//matrix-scalar multiplication

//2x2 matrix - scalar multiplication
mat2 operator*(const mat2& matrix, float scalar);

//3x3 matrix - scalar multiplication
mat3 operator*(const mat3& matrix, float scalar);

//4x4 matrix - scalar multiplication
mat4 operator*(const mat4& matrix, float scalar);

//matrix-matrix multiplication

/**
Multiplies matrices given as array

@param output Matrix to store the result
@param matA First matrix as array
@param aRows Number of rows in first matrix
@param aCols Number of columns in first matrix
@param matB Second matrix as array
@param bRows Number of rows in second matrix
@param bCols Number of columns in second matrix
@return false if matrix sizes are wrong, true otherwise
*/
bool multiply(float* output, const float* matA, int aRows,
	int aCols, const float* matB, int bRows, int bCols);

//2x2 matrix multiplication
mat2 operator*(const mat2& matA, const mat2& matB);

//3x3 matrix multiplication
mat3 operator*(const mat3& matA, const mat3& matB);

//4x4 matrix multiplication
mat4 operator*(const mat4& matA, const mat4& matB);

//matrix-vector multiplication

/**
Multiplies 3-dimensional vector/point and 4x4 matrix.
Multiplies 4th row of matrix with 1 - hidden w component of point

@param vector 3-dimensional vector
@param matrix 4x4 matrix
@return Constructed result vector 
*/
vec3 multiplyPoint(const vec3& vector, const mat4& matrix);

/**
Multiplies 3-dimensional vector and 4x4 matrix.
Multiplies 4th row of matrix with 0 - hidden w component of vector

@param vector 3-dimensional vector
@param matrix 4x4 matrix
@return Constructed result vector
*/
vec3 multiplyVector(const vec3& vector, const mat4& matrix);

/**
Multiplies 3-dimensional vector and 3x3 matrix

@param vector 3-dimensional vector
@param matrix 3x3 matrix
@return Constructed result vector
*/
vec3 multiplyVector(const vec3& vector, const mat3& matrix);


//matrix determinant related functions

/**
Computes determinant of 2x2 matrix

@param matrix 2x2 matrix
@return Determinant of matrix
*/
float determinant(const mat2& matrix);

/**
Computes determinant of 3x3 matrix using Laplace Expansion

@param matrix 3x3 matrix
@return Determinant of matrix
*/
float determinant(const mat3& matrix);

/**
Computes determinant of 4x4 matrix using Laplace Expansion

@param matrix 4x4 matrix
@return Determinant of matrix
*/
float determinant(const mat4& matrix);

/**
Removes specified column and row from 3x3 matrix and returns minor matrix

@param matrix 3x3 matrix
@param row Row to delete
@param col Column to delete
@return Minor 2x2 matrix
*/
mat2 cut(const mat3& matrix, int row, int col);

/**
Removes specified column and row from 4x4 matrix and returns minor matrix

@param matrix 4x4 matrix
@param row Row to delete
@param col Column to delete
@return Minor 3x3 matrix
*/
mat3 cut(const mat4& matrix, int row, int col);

/**
Computes matrix of minors for 2x2 matrix

@param matrix 2x2 matrix
@return Matrix of minors
*/
mat2 minor(const mat2& matrix);

/**
Computes matrix of minors for 3x3 matrix

@param matrix 3x3 matrix
@return Matrix of minors
*/
mat3 minor(const mat3& matrix);

/**
Computes matrix of minors for 4x4 matrix

@param matrix 4x4 matrix
@return Matrix of minors
*/
mat4 minor(const mat4& matrix);

//matrix cofactor functions

/**
Computes cofactor matrix and stores result in argument

@param output Result matrix as array
@param minor Source matrix as array
@param rows Number of rows in source matrix
@param cols Number of columns in source matrix
*/
void cofactor(float* output, const float* minor, int rows, int cols);

/**
Computes cofactor matrix for 2x2 matrix

@param matrix 2x2 matrix
@return Cofactor matrix
*/
mat2 cofactor(const mat2& matrix);

/**
Computes cofactor matrix for 3x3 matrix

@param matrix 3x3 matrix
@return Cofactor matrix
*/
mat3 cofactor(const mat3& matrix);

/**
Computes cofactor matrix for 4x4 matrix

@param matrix 4x4 matrix
@return Cofactor matrix
*/
mat4 cofactor(const mat4& matrix);

//matrix adjugation functions

/**
Computes adjugated matrix for 2x2 matrix

@param matrix 2x2 matrix
@return Adjugated matrix
*/
mat2 adjugate(const mat2& matrix);

/**
Computes adjugated matrix for 3x3 matrix

@param matrix 3x3 matrix
@return Adjugated matrix
*/
mat3 adjugate(const mat3& matrix);

/**
Computes adjugated matrix for 4x4 matrix

@param matrix 4x4 matrix
@return Adjugated matrix
*/
mat4 adjugate(const mat4& matrix);

//matrix inverse functions

/**
Computes inversed matrix for 2x2 matrix.

@param matrix 2x2 matrix
@return Inversed source matrix, identity matrix if source's determinant is 0
*/
mat2 inverse(const mat2& matrix);

/**
Computes inversed matrix for 3x3 matrix.

@param matrix 3x3 matrix
@return Inversed source matrix, identity matrix if source's determinant is 0
*/
mat3 inverse(const mat3& matrix);

/**
Computes inversed matrix for 4x4 matrix.

@param matrix 4x4 matrix
@return Inversed source matrix, identity matrix if source's determinant is 0
*/
mat4 inverse(const mat4& matrix);

//matrix utility functions
//matrix translation functions

/**
Creates translation matrix from three floating-point numbers

@param x x component
@param y y component
@param z z component
@return Translation matrix
*/
mat4 translation(float x, float y, float z);

/**
Creates translation matrix from vector of numbers

@param vec Vector containing x,y,z components
@return Translation matrix
*/
mat4 translation(const vec3& vec);

/**
Creates translation vector from 4x4 matrix

@param matrix 4x4 matrix
@return Translation vector
*/
vec3 getTranslation(const mat4& matrix);

//matrix scale functions

/**
Creates scale matrix from three floating-point numbers

@param x x component
@param y y component
@param z z component
@return Scale matrix
*/
mat4 scale(float x, float y, float z);

/**
Creates translation matrix from vector of numbers

@param vec Vector containing x,y,z components
@return Scale matrix
*/
mat4 scale(const vec3& vec);

/**
Creates scale vector from 4x4 matrix

@param matrix 4x4 matrix
@return Scale vector
*/
vec3 getScale(const mat4& matrix);

//matrix rotation functions

/**
Creates 3x3 rotation matrix from three floating-point numbers

@param pitch Angle of rotation around lateral - x axis
@param yaw Angle of rotation around perpendicular - y axis
@param roll Angle of rotation around longitudal - z axis
@return 3x3 rotation matrix
*/
mat3 rotationMat3(float pitch, float yaw, float roll);

/**
Creates 4x4 rotation matrix from three floating-point numbers

@param pitch Angle of rotation around lateral - x axis
@param yaw Angle of rotation around perpendicular - y axis
@param roll Angle of rotation around longitudal - z axis
@return 4x4 rotation matrix
*/
mat4 rotation(float pitch, float yaw, float roll);

/**
Creates 3x3 rotation matrix around z axis

@param angle Angle of rotation
@return 3x3 rotation matrix
*/
mat3 zRotationMat3(float angle);

/**
Creates 4x4 rotation matrix around z axis

@param angle Angle of rotation
@return 4x4 rotation matrix
*/
mat4 zRotation(float angle);

/**
Creates 3x3 rotation matrix around x axis

@param angle Angle of rotation
@return 3x3 rotation matrix
*/
mat3 xRotationMat3(float angle);

/**
Creates 4x4 rotation matrix around x axis

@param angle Angle of rotation
@return 4x4 rotation matrix
*/
mat4 xRotation(float angle);

/**
Creates 3x3 rotation matrix around y axis

@param angle Angle of rotation
@return 3x3 rotation matrix
*/
mat3 yRotationMat3(float angle);

/**
Creates 4x4 rotation matrix around y axis

@param angle Angle of rotation
@return 4x4 rotation matrix
*/
mat4 yRotation(float angle);

/**
Creates 3x3 rotation matrix around arbitrary axis

@param axis Arbitrary axis to rotate along
@param angle Angle of rotation
@return 3x3 rotation matrix
*/
mat3 axisRotationMat3(const vec3& axis, float angle);

/**
Creates 4x4 rotation matrix around arbitrary axis

@param axis Arbitrary axis to rotate along
@param angle Angle of rotation
@return 4x4 rotation matrix
*/
mat4 axisRotation(const vec3& axis, float angle);

//transform matrix functions
//generates world transform matrix

/**
Creates 4x4 transform matrix. Rotation using euler angles

@param scale Scale vector
@param eulerRotation Vector containing three angles of rotation
@param translate Translation vector
@return 4x4 transform matrix
*/
mat4 transform(const vec3& scale, const vec3& eulerRotation, const vec3& translate);

/**
Creates 4x4 transform matrix. Rotation using arbitrary axis

@param scale Scale vector
@param axisRotation Arbitrary axis of rotation
@param translate Translation vector
@return 4x4 transform matrix
*/
mat4 transform(const vec3& scale, const vec3& axisRotation, float angle, const vec3& translate);

//generates view transform matrix

/**
Creates view matrix to position the camera to look at target

@param position Position of camera
@param target Position of target
@param up Vector representing up direction
@return 4x4 transform matrix
*/
mat4 lookAt(const vec3& position, const vec3& target, const vec3& up);

//generates projection transform matrix

/**
Creates projection matrix using perspective projection

@param fov Field of view
@param aspectRatio Aspect ratio of screen
@param zNear Near plane
@param zFar Far plane
@return 4x4 projection matrix
*/
mat4 projectionPersp(float fov, float aspectRatio, float zNear, float zFar);

/**
Creates projection matrix using orthographic projection

@param xLeft Left plane
@param xRight Right plane
@param yTop Top plane
@param yBotton Bottom plane
@param zNear Near plane
@param zFar Far plane
@return 4x4 projection matrix
*/
mat4 projectionOrtho(float xLeft, float xRight, float yTop, float yBottom, float zNear, float zFar);

mat4 FromMat3(const mat3& mat);

#ifndef NO_EXTRAS
mat3 FastInverse(const mat3& mat);
mat4 FastInverse(const mat4& mat);
#endif