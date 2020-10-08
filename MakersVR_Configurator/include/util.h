/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UTIL_H
#define UTIL_H

#include "eigenutil.hpp"

#include <vector>
#include <string>

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
	float distortion[5];
	float fovH, fovV;
} DefCamera;

typedef struct
{
	struct {
		std::vector<DefMarker> calibrationMarkers;
		std::vector<DefMarker> trackingMarkers;
		std::vector<DefCamera> cameraDefinitions;
		float blobPxStdDev;
	} testing;
	struct {
		float intersectError;
		float sigmaError;
	} tracking;
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
bool parseMarkerDataFile(std::string path, std::vector<DefMarker> *markers, int fov);

/**
 * Parses the given calibration
 */
void parseCalibrationFile(std::string path, std::vector<Camera> &cameraCalib);

/**
 * Writes the given calibration to file
 */
void writeCalibrationFile(std::string path, std::vector<Camera> &cameraCalib);

#endif // UTIL_H