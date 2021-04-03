/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "eigenutil.hpp"

#include "tracking.hpp"
#include "control.hpp"

#include <vector>
#include <string>

struct Config
{
	struct {
		int cameraResolutionX;
		int cameraResolutionY;
		int cameraFramerate;
		int cameraShutterSpeed;
		int cameraGain;
		int cameraAbsThreshold;
		int cameraEdgeThreshold;
	} mode;
	struct {
		float minIntersectError;
		float maxIntersectError;
		float maxTemporalStaticErrorT;
		float maxTemporalStaticErrorR;
		float maxTemporalDynamicErrorT;
		float maxTemporalDynamicErrorR;
		float sigmaError;
	} tracking;
	struct {
		IterativeParam<float> selectionThreshold;
		IterativeParam<int> maxMarkerCount;
		// Radial density
		IterativeParam<float> radialDensityGranularity;
		IterativeParam<float> radialDensityTarget;
		// Grid density
		int gridSize;
		IterativeParam<int> gridCountTarget;
	} intrinsicCalib;
	struct {
		float maxRelationCandidateDiffT;
		float maxRelationCandidateDiffR;
		float maxOriginCandidateDiffT;
		float maxOriginCandidateDiffR;
		float maxPoseMSE;
	} extrinsicCalib;
	struct {
		std::vector<DefMarker> calibrationMarkers;
		std::vector<DefMarker> trackingMarkers;
		std::vector<Camera> cameraDefinitions;
		float blobPxStdDev;
	} testing;
};

/**
 * Parses the configuration file config.json
 */
void parseConfigFile(std::string path, Config *config);

/**
 * Parses a Marker Definition from a .obj file
 */
bool parseMarkerDataFile(std::string path, std::vector<DefMarker> &markers, int fov);

/**
 * Parses the given calibration
 */
void parseCalibrationFile(std::string path, std::vector<Camera> &cameraCalib, std::vector<MarkerTemplate3D> &markerTemplates);

/**
 * Writes the given calibration to file
 */
void writeCalibrationFile(std::string path, const std::vector<Camera> &cameraCalib, const std::vector<MarkerTemplate3D> &markerTemplates);

#endif // CONFIG_H