/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef EIGEN_UTIL_H
#define EIGEN_UTIL_H

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <string>
#include <vector>

#define NEAR_CLIP 10.0f
#define FAR_CLIP 1000.0f

/* Structures */

struct Camera
{
	std::string label;
	Eigen::Isometry3f transform;
	int width, height;
	float fovV, fovH;
	struct {
		float k1;
		float k2;
		float p1;
		float p2;
		float k3;
	} distortion;
};

struct DefMarkerPoint
{
	Eigen::Vector3f pos;
	Eigen::Vector3f nrm;
	float fov;
};

struct DefMarker
{
	int id;
	std::string label;
	std::vector<DefMarkerPoint> points;
};

/* Variables */

static const float PI = 3.14159265358979323846f;


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
	float zN = NEAR_CLIP/2, zF = FAR_CLIP; // 0.1m-10m
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

/**
 * Calculates translational and rotational difference between poseA and poseB
 */
inline std::pair<float,float> calculatePoseError(const Eigen::Isometry3f &poseA, const Eigen::Isometry3f &poseB)
{
	Eigen::Vector3f tDiff = poseB.translation() - poseA.translation();
	Eigen::Matrix3f rDiff = poseA.rotation() * poseB.rotation().transpose();
	float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
	return { tError, rError };
}

/**
 * Distort the given point in clip space
 */
inline Eigen::Vector2f distortPoint(const Camera &camera, Eigen::Vector2f point)
{
	auto *dist = &camera.distortion;
	float fx = camera.width/std::tan(camera.fovH/180.0f*PI/2)/2, fy = camera.height/std::tan(camera.fovV/180.0f*PI/2)/2;
	float x = (point.x()-camera.width/2)/fx, y = (point.y()-camera.height/2)/fy;
	float xsq = x*x;
	float ysq = y*y;
	float rsq = xsq + ysq;
	float rd = 1 + rsq * (dist->k1 + rsq * (dist->k2 + (rsq*dist->k3)));
	float dx = 2*dist->p1*x*y + dist->p2*(rsq+2*xsq);
	float dy = dist->p1*(rsq+2*ysq) + 2*dist->p2*x*y;
	float xd = x*rd + dx;
	float yd = y*rd + dy;
	return Eigen::Vector2f(xd*fx + camera.width/2, yd*fy + camera.height/2);
}

/**
 * Undistorts the given point in pixel space
 */
inline Eigen::Vector2f undistortPoint(const Camera &camera, Eigen::Vector2f point)
{
	const int iterations = 5;
	auto *dist = &camera.distortion;
	float fx = camera.width/std::tan(camera.fovH/180.0f*PI/2)/2, fy = camera.height/std::tan(camera.fovV/180.0f*PI/2)/2;
	float x = (point.x()-camera.width/2)/fx, y = (point.y()-camera.height/2)/fy;
	float xu = x, yu = y;
	for (int i = 0; i < iterations; i++)
	{
		float xsq = x*x;
		float ysq = y*y;
		float rsq = xsq + ysq;
		float rd = 1 + rsq * (dist->k1 + rsq * (dist->k2 + (rsq*dist->k3)));
		float dx = 2*dist->p1*x*y + dist->p2*(rsq+2*xsq);
		float dy = dist->p1*(rsq+2*ysq) + 2*dist->p2*x*y;
		xu = (x - dx) / rd;
		yu = (y - dy) / rd;
	}
	return Eigen::Vector2f(xu*fx + camera.width/2, yu*fy + camera.height/2);
}

#endif // EIGEN_UTIL_H