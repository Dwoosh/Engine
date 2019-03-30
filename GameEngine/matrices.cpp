#include "stdafx.h"
#include "matrices.h"
#include <cmath>
#include <cfloat>

//macro definition of float numbers comparing function

#define CMP(x, y)\
	(fabsf((x)-(y)) <= FLT_EPSILON * \
	fmaxf(1.0f, fmaxf(fabsf(x),fabsf(y))))

//matrix operation definitions

//Transposes matrix given as array and stores result in argument
void transpose(const float *srcMat, float *destMat, int srcRows, int srcCols) {
	for (int i = 0; i < srcRows * srcCols; i++) {
		int row = i / srcRows;
		int col = i % srcRows;
		destMat[i] = srcMat[srcCols * col + row];
	}
}

//Transposes given 2x2 matrix
mat2 transpose(const mat2& matrix) {
	mat2 result;
	transpose(matrix.asArray, result.asArray, 2, 2);
	return result;
}

//Transposes given 3x3 matrix
mat3 transpose(const mat3& matrix) {
	mat3 result;
	transpose(matrix.asArray, result.asArray, 3, 3);
	return result;
}

//Transposes given 4x4 matrix
mat4 transpose(const mat4& matrix) {
	mat4 result;
	transpose(matrix.asArray, result.asArray, 4, 4);
	return result;
}

//2x2 matrix - scalar multiplication
mat2 operator*(const mat2& matrix, float scalar) {
	mat2 result;
	for (int i = 0; i < 4; ++i) {
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

//3x3 matrix - scalar multiplication
mat3 operator*(const mat3& matrix, float scalar) {
	mat3 result;
	for (int i = 0; i < 9; ++i) {
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

//4x4 matrix - scalar multiplication
mat4 operator*(const mat4& matrix, float scalar) {
	mat4 result;
	for (int i = 0; i < 16; ++i) {
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

//Multiplies matrices given as array
bool multiply(float* output, const float* matA, int aRows,
	int aCols, const float* matB, int bRows, int bCols) {
	if (aCols != bRows) return false;
	for (int i = 0; i < aRows; ++i) {
		for (int j = 0; j < bCols; ++j) {
			output[bCols * i + j] = 0.0f;
			for (int k = 0; k < bRows; ++k) {
				int a = aCols * i + k;
				int b = bCols * k + j;
				output[bCols * i + j] += matA[a] * matB[b];
			}
		}
	}
	return true;
}

//2x2 matrix multiplication
mat2 operator*(const mat2& matA, const mat2& matB) {
	mat2 result;
	multiply(result.asArray, matA.asArray, 2, 2, matB.asArray, 2, 2);
	return result;
}

//3x3 matrix multiplication
mat3 operator*(const mat3& matA, const mat3& matB) {
	mat3 result;
	multiply(result.asArray, matA.asArray, 3, 3, matB.asArray, 3, 3);
	return result;
}

//4x4 matrix multiplication
mat4 operator*(const mat4& matA, const mat4& matB) {
	mat4 result;
	multiply(result.asArray, matA.asArray, 4, 4, matB.asArray, 4, 4);
	return result;
}

//Multiplies 3-dimensional vector/point and 4x4 matrix.
//Multiplies 4th row of matrix with 1 - hidden w component of point
vec3 multiplyPoint(const vec3& vector, const mat4& matrix) {
	vec3 result;
	result.x = vector.x * matrix._11 + vector.y * matrix._21 
			 + vector.z * matrix._31 + 1.0f * matrix._41;
	result.y = vector.x * matrix._12 + vector.y * matrix._22
			 + vector.z * matrix._32 + 1.0f * matrix._42;
	result.z = vector.x * matrix._13 + vector.y * matrix._23
			 + vector.z * matrix._33 + 1.0f * matrix._43;
	return result;
}

//Multiplies 3 - dimensional vector and 4x4 matrix.
//Multiplies 4th row of matrix with 0 - hidden w component of vector
vec3 multiplyVector(const vec3& vector, const mat4& matrix) {
	vec3 result;
	result.x = vector.x * matrix._11 + vector.y * matrix._21
		+ vector.z * matrix._31 + 0.0f * matrix._41;
	result.y = vector.x * matrix._12 + vector.y * matrix._22
		+ vector.z * matrix._32 + 0.0f * matrix._42;
	result.z = vector.x * matrix._13 + vector.y * matrix._23
		+ vector.z * matrix._33 + 0.0f * matrix._43;
	return result;
}

//Multiplies 3-dimensional vector and 3x3 matrix
vec3 multiplyVector(const vec3& vector, const mat3& matrix) {
	vec3 result;
	result.x = dot(vector, vec3(matrix._11, matrix._21, matrix._31));
	result.y = dot(vector, vec3(matrix._12, matrix._22, matrix._32));
	result.z = dot(vector, vec3(matrix._13, matrix._23, matrix._33));
	return result;
}

//Computes determinant of 2x2 matrix
float determinant(const mat2& matrix) {
	return matrix._11 * matrix._22 - matrix._12 * matrix._21;
}

//Computes determinant of 3x3 matrix using Laplace Expansion
float determinant(const mat3& matrix) {
	float result = 0.0f;
	mat3 cof = cofactor(matrix);
	for (int index = 0; index < 3; ++index) {
		result += matrix.asArray[index] * cof[0][index];
	}
	return result;
}

//Computes determinant of 4x4 matrix using Laplace Expansion
float determinant(const mat4& matrix) {
	float result = 0.0f;
	mat4 cof = cofactor(matrix);
	for (int index = 0; index < 4; ++index) {
		result += matrix.asArray[index] * cof[0][index];
	}
	return result;
}

//Removes specified column and row from 3x3 matrix and returns minor matrix
mat2 cut(const mat3& matrix, int row, int col) {
	mat2 result;
	int index = 0;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (i == row || j == col) {
				continue;
			}
			int target = index++;
			int source = 3 * i + j;
			result.asArray[target] = matrix.asArray[source];
		}
	}
	return result;
}

//Removes specified column and row from 4x4 matrix and returns minor matrix
mat3 cut(const mat4& matrix, int row, int col) {
	mat3 result;
	int index = 0;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (i == row || j == col) {
				continue;
			}
			int target = index++;
			int source = 4 * i + j;
			result.asArray[target] = matrix.asArray[source];
		}
	}
	return result;
}

//Computes matrix of minors for 2x2 matrix
mat2 minor(const mat2& matrix) {
	return mat2(
		matrix._22, matrix._21,
		matrix._12, matrix._11
	);
}

//Computes matrix of minors for 3x3 matrix
mat3 minor(const mat3& matrix) {
	mat3 result;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result[i][j] = determinant(cut(matrix, i, j));
		}
	}
	return result;
}

//Computes matrix of minors for 4x4 matrix
mat4 minor(const mat4& matrix) {
	mat4 result;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result[i][j] = determinant(cut(matrix, i, j));
		}
	}
	return result;
}

//Computes cofactor matrix and stores result in argument
void cofactor(float* output, const float* minor, int rows, int cols) {
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			int index = cols * j + i;
			float sign = powf(-1.0f, i + j); // -1^(i+j)
			output[index] = minor[index] * sign;
		}
	}
}

//Computes cofactor matrix for 2x2 matrix
mat2 cofactor(const mat2& matrix) {
	mat2 result;
	cofactor(result.asArray, minor(matrix).asArray, 2, 2);
	return result;
}

//Computes cofactor matrix for 3x3 matrix
mat3 cofactor(const mat3& matrix) {
	mat3 result;
	cofactor(result.asArray, minor(matrix).asArray, 3, 3);
	return result;
}

//Computes cofactor matrix for 4x4 matrix
mat4 cofactor(const mat4& matrix) {
	mat4 result;
	cofactor(result.asArray, minor(matrix).asArray, 4, 4);
	return result;
}

//Computes adjugated matrix for 2x2 matrix
mat2 adjugate(const mat2& matrix) {
	return transpose(cofactor(matrix));
}

//Computes adjugated matrix for 3x3 matrix
mat3 adjugate(const mat3& matrix) {
	return transpose(cofactor(matrix));
}

//Computes adjugated matrix for 4x4 matrix
mat4 adjugate(const mat4& matrix) {
	return transpose(cofactor(matrix));
}

//Computes inversed matrix for 2x2 matrix.
mat2 inverse(const mat2& matrix) {
	float det = determinant(matrix);
	if (CMP(det, 0.0f)) return mat2();
	return adjugate(matrix) * (1.0f / det);
}

//Computes inversed matrix for 3x3 matrix.
mat3 inverse(const mat3& matrix) {
	float det = determinant(matrix);
	if (CMP(det, 0.0f)) return mat3();
	return adjugate(matrix) * (1.0f / det);
}

//Computes inversed matrix for 4x4 matrix.
mat4 inverse(const mat4& matrix) {
	float det = determinant(matrix);
	if (CMP(det, 0.0f)) return mat4();
	return adjugate(matrix) * (1.0f / det);
}

//Creates translation matrix from three floating-point numbers
mat4 translation(float x, float y, float z) {
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		x,	  y,    z,    1.0f
	);
}

//Creates translation matrix from vector of numbers
mat4 translation(const vec3& vec) {
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		vec.x, vec.y, vec.z, 1.0f
	);
}

//Creates translation vector from 4x4 matrix
vec3 getTranslation(const mat4& matrix) {
	return vec3( matrix._41, matrix._42, matrix._43 );
}

//Creates scale matrix from three floating-point numbers
mat4 scale(float x, float y, float z) {
	return mat4(
		x,	  0.0f, 0.0f, 0.0f,
		0.0f, y,    0.0f, 0.0f,
		0.0f, 0.0f, z,    0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

//Creates translation matrix from vector of numbers
mat4 scale(const vec3& vec) {
	return mat4(
		vec.x, 0.0f,  0.0f,  0.0f,
		0.0f,  vec.y, 0.0f,  0.0f,
		0.0f,  0.0f,  vec.z, 0.0f,
		0.0f,  0.0f,  0.0f,  1.0f
	);
}

//Creates scale vector from 4x4 matrix
vec3 getScale(const mat4& matrix) {
	return vec3(matrix._11, matrix._22, matrix._33);
}

//Creates 3x3 rotation matrix from three floating-point numbers
mat3 rotationMat3(float pitch, float yaw, float roll) {
	return zRotationMat3(roll) * xRotationMat3(pitch) * yRotationMat3(yaw);
}

//Creates 4x4 rotation matrix from three floating-point numbers
mat4 rotation(float pitch, float yaw, float roll) {
	return zRotation(roll) * xRotation(pitch) * yRotation(yaw);
}

//Creates 3x3 rotation matrix around z axis
mat3 zRotationMat3(float angle) {
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle),  sinf(angle), 0.0f,
		-sinf(angle), cosf(angle), 0.0f,
		0.0f,		  0.0f,		   1.0f
	);
}

//Creates 4x4 rotation matrix around z axis
mat4 zRotation(float angle) {
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle),  sinf(angle), 0.0f, 0.0f,
		-sinf(angle), cosf(angle), 0.0f, 0.0f,
		0.0f,		  0.0f,		   1.0f, 0.0f,
		0.0f,		  0.0f,		   0.0f, 1.0f
	);
}

//Creates 3x3 rotation matrix around x axis
mat3 xRotationMat3(float angle) {
	angle = DEG2RAD(angle);
	return mat3(
		1.0f, 0.0f,		    0.0f,
		0.0f, cosf(angle),  sinf(angle),
		0.0f, -sinf(angle),	cosf(angle)
	);
}

//Creates 4x4 rotation matrix around x axis
mat4 xRotation(float angle) {
	angle = DEG2RAD(angle);
	return mat4(
		1.0f, 0.0f,		    0.0f,	     0.0f,
		0.0f, cosf(angle),  sinf(angle), 0.0f,
		0.0f, -sinf(angle),	cosf(angle), 0.0f,
		0.0f, 0.0f,		    0.0f,		 1.0f
	);
}

//Creates 3x3 rotation matrix around y axis
mat3 yRotationMat3(float angle) {
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle), 0.0f, -sinf(angle),
		0.0f,		 1.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle)
	);
}

//Creates 4x4 rotation matrix around y axis
mat4 yRotation(float angle) {
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle), 0.0f, -sinf(angle), 0.0f,
		0.0f,		 1.0f, 0.0f,		 0.0f,
		sinf(angle), 0.0f, cosf(angle),  0.0f,
		0.0f,		 0.0f, 0.0f,		 1.0f
	);
}

//Creates 3x3 rotation matrix around arbitrary axis
mat3 axisRotationMat3(const vec3& axis, float angle) {
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - c;
	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	if (!CMP(magnitudeSq(axis), 1.0f)) {	//if axis is not normalized
		float invLen = 1.0f / magnitude(axis);	//normalize x, y and z
		x *= invLen;
		y *= invLen;
		z *= invLen;
	}
	return mat3(
		t*(x*x) + c, t*x*y + s * z, t*x*z - s * y,
		t*x*y - s * z, t*(y*y) + c, t*y*z + s * x,
		t*x*z + s * y, t*y*z - s * x, t*(z*z) + c
	);
}

//Creates 4x4 rotation matrix around arbitrary axis
mat4 axisRotation(const vec3& axis, float angle) {
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - c;
	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	if (!CMP(magnitudeSq(axis), 1.0f)) {	//if axis is not normalized
		float invLen = 1.0f / magnitude(axis);	//normalize x, y and z
		x *= invLen;
		y *= invLen;
		z *= invLen;
	}
	return mat4(
		t*(x*x) + c, t*x*y + s*z, t*x*z - s*y, 0.0f,
		t*x*y - s*z, t*(y*y) + c, t*y*z + s*x, 0.0f,
		t*x*z + s*y, t*y*z - s*x, t*(z*z) + c, 0.0f,
		0.0f,		 0.0f,		  0.0f,		   1.0f
	);
}

//Creates 4x4 transform matrix. Rotation using euler angles
mat4 transform(const vec3& scaleVec, const vec3& eulerRotation, const vec3& translateVec) {
	return scale(scaleVec) * rotation(eulerRotation.x, eulerRotation.y, eulerRotation.z) * translation(translateVec);
}

//Creates 4x4 transform matrix. Rotation using arbitrary axis
mat4 transform(const vec3& scaleVec, const vec3& rotationAxis, float angle, const vec3& translateVec) {
	return scale(scaleVec) * axisRotation(rotationAxis, angle) * translation(translateVec);
}

//Creates view matrix to position the camera to look at target
mat4 lookAt(const vec3& position, const vec3& target, const vec3& up) {
	//create rotation basis for camera
	vec3 forward = normalized(target - position);
	vec3 right = normalized(cross(up, forward));
	vec3 newUp = cross(forward, right);
	
	//return transposed rotation matrix with negated
	//dot products to avoid expensive operations
	return mat4(
		right.x, newUp.x, forward.x, 0.0f,
		right.y, newUp.y, forward.y, 0.0f,
		right.z, newUp.z, forward.z, 0.0f,
		-dot(right, position), -dot(newUp, position),
		-dot(forward, position), 1.0f
	);
}

//Creates projection matrix using perspective projection
mat4 projectionPersp(float fov, float aspectRatio, float zNear, float zFar) {
	float tanHalfFOV = tanf(DEG2RAD((fov*0.5f)));
	float fovY = 1.0f / tanHalfFOV;
	float fovX = fovY / aspectRatio;
	mat4 result;
	result._11 = fovX;
	result._22 = fovY;
	result._33 = zFar / (zFar - zNear);
	result._34 = 1.0f;
	result._43 = -zNear * result._33;
	result._44 = 0.0f;
	return result;
}

//Creates projection matrix using orthographic projection
mat4 projectionOrtho(float xLeft, float xRight, float yTop, float yBottom, float zNear, float zFar) {
	mat4 result;
	result._11 = 2.0f / (xRight - xLeft);
	result._22 = 2.0f / (yTop - yBottom);
	result._33 = 1.0f / (zFar - zNear);
	result._41 = (xLeft + xRight) / (xLeft - xRight);
	result._42 = (yBottom + yTop) / (yBottom - yTop);
	result._43 = zNear / (zNear - zFar);
	return result;
}

mat4 FromMat3(const mat3& mat) {
	mat4 result;

	result._11 = mat._11;
	result._12 = mat._12;
	result._13 = mat._13;

	result._21 = mat._21;
	result._22 = mat._22;
	result._23 = mat._23;

	result._31 = mat._31;
	result._32 = mat._32;
	result._33 = mat._33;

	return result;
}

#ifndef NO_EXTRAS
mat3 FastInverse(const mat3& mat) {
	return transpose(mat);
}

mat4 FastInverse(const mat4& mat) {

	mat4 inverse = transpose(mat);
	inverse._41 = inverse._14 = 0.0f;
	inverse._42 = inverse._24 = 0.0f;
	inverse._43 = inverse._34 = 0.0f;

	vec3 right = vec3(mat._11, mat._12, mat._13);
	vec3 up = vec3(mat._21, mat._22, mat._23);
	vec3 forward = vec3(mat._31, mat._32, mat._33);
	vec3 position = vec3(mat._41, mat._42, mat._43);

	inverse._41 = -dot(right, position);
	inverse._42 = -dot(up, position);
	inverse._43 = -dot(forward, position);

	return inverse;
}
#endif 