/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_POSE_INFERENCE
#define DEF_POSE_INFERENCE

#include "util.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <vector>

/*
 * Pose Inference
 * Responsible for trying to infer 3D poses from a set of 2D blobs in screen space using solvePnP
 * Currently works on a frame-by-frame basis, so no association to witch drone exists
 *
 * Stretch Goal:
 * Takes poses from previous frames and tries to match them, to provide solvePnP with an initial estimation
 * This will improve performance AND accuracy
 */

/* Structures  */

// Point with size
typedef struct Point{
	float X;
	float Y;
	float S;
} Point;

// Pose of LED marker
typedef struct Pose
{
	Point *center;
	Eigen::Vector3f trans;
	Eigen::Matrix3f rot;
	// TODO
	// 3D AND 2D pixel space of all LEDs
	// Need access to at least the center LED screen space position for color fetching later
} Pose;

typedef struct Marker
{
	Point *center;
	Point *endA;
	Point *endB;
	Point *header;
} Marker;
typedef struct MarkerCandidate
{
	uint8_t c;
	uint8_t a;
	uint8_t b;
	uint8_t h;
	float error;
} MarkerCandidate;

typedef struct LinePoint
{
	float X;
	float Y;
	float R;
	float G;
	float B;
} LinePoint;

typedef struct
{
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot;
} Transform;

/* Functions */

/*
 * Initialize resources for pose inference
 */
void initPoseInference(int Width, int Height);

/*
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(std::vector<Point> &blobs, std::vector<Marker> &marker, std::vector<Point*> &freeBlobs);

/*
 * Infer the pose of a set of (likely) markers and the known camera position to transform into world space
 */
void inferMarkerPoses(std::vector<Marker> &marker, const Transform &camera, std::vector<Pose> &poses);

/*
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void inferMarkerPoseGeneric(std::vector<Point> &marker, const Transform &camera, Pose &pose);

/*
 * Visualize poses
 */
void visualizePoses(const Transform &camera, const std::vector<Point> &blobs, const std::vector<Marker> &marker, const std::vector<Pose> &poses);

/**
 * Cleanup of resources
 */
void cleanPoseInference();

/*
 * Sets the specified marker data as the current target
 */
void setActiveMarkerData(DefMarker markerData);

/*
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount();

/*
 * Projects marker into image plane provided translation in centimeters and rotation, as well as a camera position
 */
void createMarkerProjection(std::vector<Point> &imagePoints, const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float ptScale);

#endif