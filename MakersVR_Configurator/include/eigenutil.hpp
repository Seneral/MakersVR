/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_EIGEN_UTIL
#define DEF_EIGEN_UTIL

#include "Eigen/Core"
#include "Eigen/Geometry"

#define _USE_MATH_DEFINES
#include <math.h>


/* Structures */

typedef struct Point
{
	float X;
	float Y;
	float S;
} Point;

typedef struct
{
	Eigen::Isometry3f transform;
	int width, height;
	float fovV, fovH;
} Camera;


/* Variables */

static const float PI = (float)M_PI;


/* Functions */

/**
 * Return x,y,z angles in standard Blender notation, with the order in which they are applied being z,y,x
 */
inline Eigen::Vector3f getEulerXYZ(const Eigen::Matrix3f &rotMat)
{
	Eigen::Vector3f eulerZYX = rotMat.eulerAngles(2,1,0); // Decompose in order z,y,x
	return Eigen::Vector3f(eulerZYX[2], eulerZYX[1], eulerZYX[0]); // Swivel to get x,y,z values
}

/**
 * Return x,y,z angles in ZYX Blender notation, with the order in which they are applied being x,y,z
 */
inline Eigen::Vector3f getEulerZYX(const Eigen::Matrix3f &rotMat)
{
	return rotMat.eulerAngles(0,1,2); // Decompose in order x,y,z
}

/**
 * Get rotation matrix from x,y,z angles in standard Blender notation, with the order in which they are applied being z,y,x
 */
inline Eigen::Matrix3f getRotationXYZ(const Eigen::Vector3f &eulerAngles)
{
	return Eigen::Matrix3f(
		  Eigen::AngleAxisf(eulerAngles.z(), Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(eulerAngles.y(), Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(eulerAngles.x(), Eigen::Vector3f::UnitX())
	);
}

/**
 * Get rotation matrix from x,y,z angles in ZYX Blender notation, with the order in which they are applied being x,y,z
 */
inline Eigen::Matrix3f getRotationZYX(const Eigen::Vector3f &eulerAngles)
{
	return Eigen::Matrix3f(
		  Eigen::AngleAxisf(eulerAngles.x(), Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(eulerAngles.y(), Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(eulerAngles.z(), Eigen::Vector3f::UnitZ())
	);
}

/**
 * Create projection matrix using default clip planes and given field of view
 */
inline Eigen::Projective3f createProjectionMatrix(float fovH, float fovV)
{
	float zN = 10.0f, zF = 1000.0f; // 0.1m-10m
	float a = -(zF+zN)/(zF-zN), b = -2*zN*zF/(zF-zN);
	float sX = (float)(1.0f/std::tan(fovH*PI/360.0f)), sY = (float)(1.0f/std::tan(fovV*PI/360.0f));

	// Projection
	Eigen::Projective3f p;
	p.matrix() << 
		sX, 0, 0, 0,
		0, sY, 0, 0,
		0, 0, a, b,
		0, 0, -1, 0;
	return p;
}

/**
 * Create model matrix using given transformations
 */
inline Eigen::Affine3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale)
{
	Eigen::Affine3f m;
	m.linear() = rotation*scale;
	m.translation() = translation;
	return m;
}

/**
 * Create model matrix using given transformations
 */
inline Eigen::Isometry3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation)
{
	Eigen::Isometry3f m;
	m.linear() = rotation;
	m.translation() = translation;
	return m;
}

/**
 * Create view matrix using given camera transform
 */
inline Eigen::Isometry3f createViewMatrix(const Eigen::Isometry3f &camera)
{
	return camera.inverse();
}

/**
 * Projects the point into clip space and normalizes w
 */
inline Eigen::Vector3f projectPoint(const Eigen::Projective3f &projection, const Eigen::Vector3f &point)
{
	Eigen::Vector4f projected = projection * Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
	return projected.head<3>() / projected.w();
}

#endif