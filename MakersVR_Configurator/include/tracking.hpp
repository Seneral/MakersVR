/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_TRACKING
#define DEF_TRACKING

#define MAX_MARKER_POINTS 64

#include "eigenutil.hpp"
#include "util.h" // Using DefMarker - later to be replaced with own structure

#include <vector>
#include <bitset>

/**
 * Tracking
 */


/* Structures  */

typedef struct
{
	Eigen::Vector3f pos;
	Eigen::Vector3f dir;
	/* Exposed for performance reason, makes triangulateRayIntersections much easier */
	int intersectionCount;
	int conflictID; // if intersectionCount > 1
} Ray;

typedef struct
{
	Eigen::Vector3f pos;
	float error; // If valid, how accurate
	float confidence; // How likely to be valid
} TriangulatedPoint;

typedef struct PointRelation
{
	int pt1;
	int pt2;
	Eigen::Vector3f dir;
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
	// TODO: Change to continuous vector, as most subvectors have similar lengths (NUM_CLOSEST_RELATIONS to 2*NUM_CLOSEST_RELATIONS)
} MarkerLookup;


/* Variables */

extern MarkerLookup marker3D;


/* Functions */

/**
 * Initialize resources for tracking
 */
void initTracking();

/**
 * Cleanup of resources
 */
void cleanTracking();

/**
 * Sets up lookup tables for quick marker identification
 */
void generateLookupTables(MarkerLookup *marker3D);

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void castRays(const std::vector<Eigen::Vector2f> &point2D, const Camera &camera, std::vector<Ray> &rays3D);

/**
 * Calculate the intersection points between rays of separate groups
 * Returns how many of the points are not conflicted (those are at the beginning of the array)
 */
int triangulateRayIntersections(std::vector<std::vector<Ray>*> &rayGroups, std::vector<TriangulatedPoint> &points3D, std::vector<std::vector<int>> &conflicts, float errorLimit);

/**
 * Detect Markers in the triangulated 3D Point cloud
 */
void detectMarkers3D(const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<Eigen::Isometry3f> &poses3D, std::vector<std::pair<float,int>> &posesMSE, float sigmaError = 3);

/**
 * Matches the current poses to the poses of the last frame using temporal information
 */
void matchTrackedPoses(const std::vector<Eigen::Isometry3f> &currentPose, const std::vector<Eigen::Isometry3f> &lastPose, const std::vector<Eigen::Vector3f> &lastDir, const std::vector<float> &lastRot, std::vector<int> &matching);

/**
 * Accepts the previously calculated matching and updates temporal information
 */
void matchAccept(std::vector<Eigen::Isometry3f> &currentPose, std::vector<Eigen::Isometry3f> &lastPose, std::vector<Eigen::Vector3f> &lastDir, std::vector<float> &lastRot, const std::vector<int> &matching);


#endif