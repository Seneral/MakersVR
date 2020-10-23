/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_TRACKING
#define DEF_TRACKING

#define MAX_MARKER_POINTS 64

#include "eigenutil.hpp"

#include <vector>

/**
 * Tracking
 */


/* Structures  */

struct Ray
{
	Eigen::Vector3f pos;
	Eigen::Vector3f dir;
	/* Exposed for performance reason, makes triangulateRayIntersections much easier */
	int intersectionCount;
	int conflictID; // if intersectionCount > 1
};

struct TriangulatedPoint
{
	Eigen::Vector3f pos;
	float error; // If valid, how accurate
	float confidence; // How likely to be valid
};

struct PointRelation
{
	int pt1;
	int pt2;
	Eigen::Vector3f dir;
	float distance;
 };
struct ErrorRangeComp {
	float error;
	ErrorRangeComp(float error) { this->error = error; }
	bool operator() (const PointRelation& rel, float value);
	bool operator() (float value, const PointRelation& rel);
};

struct MarkerTemplate3D
{
	std::string label;
	int id;
	std::vector<Eigen::Vector3f> points;
	std::vector<PointRelation> relationDist; // Shortest neighouring relations of all points, sorted by distance
	std::vector<std::vector<int>> pointRelation; // Index of relations for each point
	// TODO: Change to continuous vector, as most subvectors have similar lengths (NUM_CLOSEST_RELATIONS to 2*NUM_CLOSEST_RELATIONS)
};

struct MarkerCandidate3D
{
	std::vector<int> points;
	std::vector<int> pointMap;
	// To get triangulated points: points3D[points[i]]
	// To get actual points: marker3D[pointMap[points3D[i]]]
	Eigen::Isometry3f pose;
};


/* Functions */

/**
 * Sets up lookup tables for quick marker identification
 */
void generateLookupTables(MarkerTemplate3D *marker3D);

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void castRays(const std::vector<Eigen::Vector2f> &point2D, const Camera &camera, std::vector<Ray> &rays3D);

/**
 * Calculate the intersection points between rays of separate groups
 * Returns how many of the points are not conflicted (those are at the beginning of the array)
 */
int triangulateRayIntersections(std::vector<std::vector<Ray>*> &rayGroups, std::vector<TriangulatedPoint> &points3D, std::vector<std::vector<int>> &conflicts, float maxError, float minError = 0.0f);

/**
 * Calculate MSE of candidate marker in given point cloud
 */
float calculateCandidateMSE(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const MarkerCandidate3D *candidate);

/**
 * Picks the best candidate by point count and internal MSE.
 * Returns the index, MSE and count of other candidates with the same maximum point count
 */
std::tuple<int,float,int> getBestMarkerCandidate(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, const std::vector<MarkerCandidate3D> &candidates);

/**
 * Detect a marker in the triangulated 3D Point cloud, returns all candidates
 */
void detectMarker3D(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<MarkerCandidate3D> &candidates, float sigmaError = 3, bool quickAssign = true);

/**
 * Detect a marker in the triangulated 3D Point cloud, returns the best candidate
 */
float detectMarker3D(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, MarkerCandidate3D *bestCandidate, float sigmaError = 3, bool quickAssign = true);

/**
 * Detect a marker in the triangulated 3D Point cloud, returns all candidate poses with respective MSE and point count
 */
void detectMarker3D(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<Eigen::Isometry3f> &poses3D, std::vector<std::pair<float,int>> &posesMSE, float sigmaError = 3, bool quickAssign = true);

/**
 * Detect a marker in the triangulated 3D Point cloud, returns the best candidate pose with respective MSE and point count (point-count 0 if none found)
 */
std::tuple<Eigen::Isometry3f, float, int> detectMarker3D(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, float sigmaError = 3, bool quickAssign = true);

/**
 * Matches the current poses to the poses of the last frame using temporal information
 */
void matchTrackedPoses(const std::vector<Eigen::Isometry3f> &currentPose, const std::vector<Eigen::Isometry3f> &lastPose, const std::vector<Eigen::Vector3f> &lastDir, const std::vector<float> &lastRot, std::vector<int> &matching);

/**
 * Accepts the previously calculated matching and updates temporal information
 */
void matchAccept(std::vector<Eigen::Isometry3f> &currentPose, std::vector<Eigen::Isometry3f> &lastPose, std::vector<Eigen::Vector3f> &lastDir, std::vector<float> &lastRot, const std::vector<int> &matching);


#endif