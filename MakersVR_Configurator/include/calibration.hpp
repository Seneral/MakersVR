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

/**
 * Calibration
 */


/* Structures */

typedef struct Marker
{
	Point *center;
	Point *endA;
	Point *endB;
	Point *header;
} Marker;

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
void initCalibration(int Width, int Height);

/**
 * Cleanup of resources
 */
void cleanCalibration();

/**
 * Returns the camera with intrinsic parameters calibrated
 */
Camera getCalibratedCamera();

/**
 * Finds all (potential) marker and also returns all free (unassociated) points left (Deprecated)
 */
void findMarkerCandidates(std::vector<Point> &points2D, std::vector<Marker> &markers2D, std::vector<Point*> &freePoints2D);

/**
 * Infer the pose of a (likely) marker in camera space given its image points and the intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPose(Marker *marker2D, const Camera &camera);

/**
 * Infer the pose of a marker in camera space given its image points and intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPoseGeneric(std::vector<Point> &points2D, const Camera &camera);


/**
 * Calculate mean squared error in image space of detected pose
 */
float calculateMSEGeneric(const Eigen::Isometry3f &pose3D, const Camera &camera, std::vector<Point> &points2D);

/**
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount();

#endif