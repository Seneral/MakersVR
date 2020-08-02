/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Eigen/Core"
#include "Eigen/Geometry"

static const float PI = (float)M_PI;

typedef struct
{
	double start, avg, diff, min, max, num, cur;
} StatValue;

typedef struct
{
	Eigen::Vector3f pos;
	Eigen::Vector3f nrm;
	float fov;
} DefMarkerPoint;

typedef struct
{
	std::string label;
	std::vector<DefMarkerPoint> pts;
} DefMarker;

typedef struct
{
	std::string label;
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot;
} DefCamera;

typedef struct
{
	struct {
		std::vector<std::string> markerDefinitionFiles;
		std::vector<DefCamera> cameraDefinitions;
	} testing;
} Config;

/**
 * Updates the given statistical value with the new value
 */
void UpdateStatValue(StatValue *stat, double cur);

/**
 * Parses the configuration file config.json
 */
void parseConfigFile(std::string path, Config *config);

/**
 * Parses a Marker Definition from a .obj file
 */
bool parseMarkerDataFile(std::string path, std::vector<DefMarker> *markers);

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

#endif // UTIL_H