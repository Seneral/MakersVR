/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_CALIBRATION
#define DEF_CALIBRATION

#include "eigenutil.hpp"

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


/* Variables */

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
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(std::vector<Point> &blobs, std::vector<Marker> &marker, std::vector<Point*> &freeBlobs);

/**
 * Infer the pose of a set of (likely) markers and the known camera position to transform into world space
 */
void inferMarkerPoses(std::vector<Marker> &marker, const Camera &camera, std::vector<Eigen::Isometry3f> &poses);

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void inferMarkerPoseGeneric(std::vector<Point> &point2D, const Camera &camera, Eigen::Isometry3f &pose);

/**
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount();

#endif