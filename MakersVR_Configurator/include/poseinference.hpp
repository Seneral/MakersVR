/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_POSE_INFERENCE
#define DEF_POSE_INFERENCE

#define MAX_MARKER_POINTS 64

#include "util.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <vector>
#include <bitset>

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
	Eigen::Vector3f trans;
	Eigen::Matrix3f rot;
} Pose;

typedef struct Marker
{
	Point *center;
	Point *endA;
	Point *endB;
	Point *header;
} Marker;

typedef struct
{
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot;
} Transform;

typedef struct
{
	Eigen::Vector4f pos;
	Eigen::Vector4f dir;
	/* Exposed for performance reason, makes triangulateRayIntersections much easier */
	int intersectionCount;
	int conflictID; // if intersectionCount > 1
} Ray;
typedef struct
{
	Eigen::Vector4f pos;
	float error; // If valid, how accurate
	float confidence; // How likely to be valid
} TriangulatedPoint;

typedef struct PointRelation
{
	int pt1;
	int pt2;
	Eigen::Vector4f dir;
	float distance;
 } PointRelation;
struct ErrorRangeComp {
	float error;
	ErrorRangeComp(float error) { this->error = error; }
	bool operator() (const PointRelation& rel, float value);
	bool operator() (float value, const PointRelation& rel);
};

typedef struct
{
	DefMarker markerTemplate;
	std::vector<PointRelation> relationDist; // Shortest neighouring relations of all points, sorted by distance
	std::vector<std::vector<int>> pointRelation; // Index of relations for each point
} MarkerLookup;



extern MarkerLookup marker3D;


/* Functions */

/*
 * Initialize resources for pose inference
 */
void initPoseInference(int Width, int Height);


/* ---- Single-Camera Pose Inference ---- */

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
void inferMarkerPoseGeneric(std::vector<Point> &point2D, const Transform &camera, Pose &pose);


/* ---- Multi-Camera Pose Inference ---- */

/*
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void castRays(const std::vector<Point> &point2D, const Transform &camera, std::vector<Ray> &rays3D);

/*
 * Calculate the intersection points between rays of separate groups
 * Returns how many of the points are not conflicted (those are at the beginning of the array)
 */
int triangulateRayIntersections(std::vector<std::vector<Ray>*> &rayGroups, std::vector<TriangulatedPoint> &points3D, std::vector<std::vector<int>> &conflicts, float errorLimit);

/*
 * Detect Markers in the triangulated 3D Point cloud
 */
void detectMarkers3D(const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<Pose> &poses3D);


/* ---- General ---- */

/*
 * Visualize single-camera poses
 */
void visualizePoses(const Transform &camera, const std::vector<Point> &points2D, const std::vector<Marker> &markers2D, const std::vector<Pose> &poses3D);

/*
 * Visualize multi-camera triangulated markers
 * points2D: Visible points projected into camera view
 * rays3D: rays from all cameras
 * points3D: Points triangulated from the algorithm, without conflicts first
 * nonconflictedCount: Amount of nonconflicted points in points3D
 * triangulatedPoints3D: Points which could have been triangulated (are visible from more than one camera) -- used for testing only
 * poses3D: Inferred 3D poses
 */
void visualizeMarkers(const Transform &camera, const std::vector<Point> &points2D, const std::vector<Ray> &rays3D, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount, const std::vector<Eigen::Vector4f> &triangulatedPoints3D, const std::vector<Pose> &poses3D);

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
void createMarkerProjection(std::vector<Point> &points2D, std::bitset<MAX_MARKER_POINTS> &mask, const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float ptScale, float stdDeviation = 0.0f);

/*
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector4f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation);

#endif