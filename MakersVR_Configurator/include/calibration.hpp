/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_CALIBRATION
#define DEF_CALIBRATION

#define MAX_POSE_MSE 0.4f*0.4f

#include "eigenutil.hpp"
#include "util.h"

#ifdef USE_CV
#include "opencv2/core.hpp"
#endif

#include <vector>
#include <map>

/**
 * Calibration
 */


/* Structures */

struct Marker
{
	std::vector<Eigen::Vector2f> pts;
	Marker(int N) : pts(N) {}
	Marker(std::vector<Eigen::Vector2f> &&PTS)
	{ // Copy from temporary initializer
		pts.swap(PTS);
	}
};

typedef struct TransformCandidate
{
	Eigen::Isometry3f transform;
	float weight;
	std::vector<std::pair<Eigen::Isometry3f, float>> datapoints;
} TransformCandidate;

typedef struct TransformSample
{
	Eigen::Isometry3f transform;
	float weight;
	float stdDeviation;
} TransformSample;

typedef struct CameraRelation
{
	int camA;
	int camB;
	TransformSample sample; // Result
	std::vector<TransformCandidate> candidates;
} CameraRelation;


/* Variables */

extern DefMarker calibMarker3D;

#ifdef USE_CV
extern std::vector<cv::Point3f> cv_marker3DTemplate;
#endif

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
 * Finds all (potential) marker and also returns all free (unassociated) points left (Deprecated)
 */
void findMarkerCandidates(const std::vector<Eigen::Vector2f> &points2D, const std::vector<float> &pointSizes, std::vector<Marker> &markers2D, std::vector<int> &freePoints2D);

/**
 * Infer the pose of a (likely) marker in camera space given its image points and the intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPose(const Camera &camera, const Marker &marker2D);

/**
 * Calculate mean squared error in image space of detected pose
 */
float calculateMSE(const Eigen::Isometry3f &pose3D, const Camera &camera, const Marker &marker2D);

/**
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount();

/**
 * Adds transform to a candidate within error margin or creates a new one
 */
int AddTransformToCandidates (std::vector<TransformCandidate> &candidates, Eigen::Isometry3f transform, float weight, float maxTError, float maxRError);

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
bool selectMarkersForCalibration(const Camera &camera, const std::vector<Marker> &markers, std::vector<Marker> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, float radialGranularity, float radialTarget, std::vector<std::vector<uint16_t>> &gridBuckets, int gridSize, int gridTarget, float *threshold, int m);

/**
 * Calibrate using a range of detected markers, returning standard deviation and outputing individual marker errors
 */
float calculateIntrinsicCalibration(Camera &camera, const std::vector<Marker> &markers, std::vector<double> &errors);

/**
 * Takes the calibration selection and filters them using the errors from the last calibration round
 */
void filterMarkersForCalibration(const Camera &camera, const std::vector<double> &errors, std::vector<Marker> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, std::vector<std::vector<uint16_t>> &gridBuckets, int m);

#endif