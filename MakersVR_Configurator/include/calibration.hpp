/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#define MAX_POSE_MSE 1 //0.4f*0.4f

#include "eigenutil.hpp"

#include <vector>
#include <map>

/**
 * Calibration
 */


/* Structures */

struct Marker2D
{
	int id;
	std::vector<Eigen::Vector2f> points;
	Marker2D(int id, int N) : id(id), points(N) {}
	Marker2D(int id, std::vector<Eigen::Vector2f> &&PTS)
	{ // Copy from temporary initializer
		this->id = id;
		points.swap(PTS);
	}
};

struct TransformCandidate
{
	Eigen::Isometry3f transform;
	float weight;
	std::vector<std::pair<Eigen::Isometry3f, float>> datapoints;
};

struct TransformSample
{
	Eigen::Isometry3f transform;
	float weight;
	float stdDeviation;
};

struct CameraRelation
{
	int camA;
	int camB;
	TransformSample sample; // Result
	std::vector<TransformCandidate> candidates;
};


/* Functions */

/**
 * Initialize resources for calibration
 */
void initCalibration();

/**
 * Cleanup of resources
 */
void cleanCalibration();

/**
 * Fill with built-in markers that findMarkerCandidate can detect
 */
void getBuiltInMarkers(std::vector<DefMarker> &markers2D);

/**
 * Returns if marker ID can be detected by findMarkerCandidates
 */
bool isMarkerBuiltIn(int id);

/**
 * Finds all (potential) marker and also returns all free (unassociated) points left (Deprecated)
 */
void findMarkerCandidates(const std::vector<Eigen::Vector2f> &points2D, const std::vector<float> &pointSizes, std::vector<Marker2D> &markers2D, std::vector<int> &freePoints2D);

/**
 * Infer the pose of a (likely) marker in camera space given its image points and the intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPose(const Camera &camera, const Marker2D &marker2D, const DefMarker &markerTemplate2D);

/**
 * Calculate mean squared error in image space of detected pose
 */
float calculateMSE(const Eigen::Isometry3f &pose3D, const Camera &camera, const Marker2D &marker2D, const DefMarker &markerTemplate2D);

/**
 * Adds transform to a candidate within error margin or creates a new one
 */
int AddTransformToCandidates(std::vector<TransformCandidate> &candidates, Eigen::Isometry3f transform, float weight, float maxTError, float maxRError);

/**
 * Choose transform canidate among the given candidates with the most weight
 */
TransformSample chooseBestTransformCandidate(const std::vector<TransformCandidate> &candidates);

/**
 * Calculate absolute camera transforms with weighted samples of origin and relations between cameras, with focus on keeping the relations intact
 */
void calculateCameraTransforms(std::vector<TransformSample*> &origins, std::vector<std::vector<TransformSample*>> &relations, std::vector<Eigen::Isometry3f> &cameraTransforms);

/**
 * Adds markers that add value to the calibration to the calibration selection and adjusts selection parameters
 */
bool selectMarkersForCalibration(const Camera &camera, const DefMarker &markerTemplate2D, const std::vector<Marker2D> &markers, std::vector<Marker2D> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, float radialGranularity, float radialTarget, std::vector<std::vector<uint16_t>> &gridBuckets, int gridSize, int gridTarget, float threshold, int m);


/**
 * Calibrate using a range of detected markers (each detected marker has to have the same point count as the template and the same ID)
 */
float calculateIntrinsicCalibration(Camera &camera, const std::vector<Marker2D> &markers, const DefMarker &markerTemplate2D, std::vector<double> &errors);

/**
 * Takes the calibration selection and filters them using the errors from the last calibration round
 */
void filterMarkersForCalibration(const Camera &camera, const std::vector<double> &errors, std::vector<Marker2D> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, std::vector<std::vector<uint16_t>> &gridBuckets);

#endif // CALIBRATION_H