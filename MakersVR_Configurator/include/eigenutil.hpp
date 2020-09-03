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
Eigen::Vector3f getEulerXYZ(const Eigen::Matrix3f &rotMat);

/**
 * Return x,y,z angles in ZYX Blender notation, with the order in which they are applied being x,y,z
 */
Eigen::Vector3f getEulerZYX(const Eigen::Matrix3f &rotMat);

/**
 * Get rotation matrix from x,y,z angles in standard Blender notation, with the order in which they are applied being z,y,x
 */
Eigen::Matrix3f getRotationXYZ(const Eigen::Vector3f &eulerAngles);

/**
 * Get rotation matrix from x,y,z angles in ZYX Blender notation, with the order in which they are applied being x,y,z
 */
Eigen::Matrix3f getRotationZYX(const Eigen::Vector3f &eulerAngles);

/**
 * Create projection matrix using default clip planes and given field of view
 */
Eigen::Projective3f createProjectionMatrix(float fovH, float fovV);

/**
 * Create model matrix using given transformations
 */
Eigen::Affine3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale);

/**
 * Create model matrix using given transformations
 */
Eigen::Isometry3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation);

/**
 * Create view matrix using given camera transform
 */
Eigen::Isometry3f createViewMatrix(const Eigen::Isometry3f &camera);

/**
 * Projects the point into clip space and normalizes w
 */
Eigen::Vector3f projectPoint(const Eigen::Projective3f &projection, const Eigen::Vector3f &point);

#endif