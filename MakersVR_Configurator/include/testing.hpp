/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_TESTING
#define DEF_TESTING

#include "eigenutil.hpp"
#include "util.hpp" // DefMarker
#include "tracking.hpp"
#include "calibration.hpp"

#include <vector>
#include <bitset>

/*
 * Testing
 */


/* Structures */

struct Color
{
	float r,g,b;
};


/* Functions */

/**
 * Initialize resources for visualization
 */
void initVisualization();

/**
 * Cleanup of resources
 */
void cleanVisualization();

/**
 * Projects marker into image plane provided translation in centimeters and rotation, as well as a camera position
 */
void createMarkerProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<float> &pointSizes, std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker, const Camera &camera, const Eigen::Isometry3f &transform, float stdDeviation = 0.0f);

/**
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector3f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const MarkerTemplate3D &marker3D, const Eigen::Isometry3f &transform);

/**
 * Analyze possibility of tracking algorithm to extract the pose based on ground truth data
 */
void analyzeTrackingAlgorithm(const std::vector<int> &visibleCount, const std::bitset<MAX_MARKER_POINTS> &triangulationMask, const std::vector<TriangulatedPoint> &points3D, const MarkerTemplate3D marker3D, Eigen::Isometry3f gt);

/**
 * Visualize 2D points in pixel space
 */
void visualizePoints2D(const Camera &camera, const std::vector<Eigen::Vector2f> &points2D, Color color = { 1, 0, 0 }, float size = 6.0f, float depth = 0.9f, bool undistort = false);

/**
 * Visualize 3D points in world space
 */
void visualizePoints3D(const Camera &camera, const std::vector<Eigen::Vector3f> &points3D, Color color, float size, float depth = 0);

/**
 * Visualize poses in camera or world space
 */
void visualizePoses(const Camera &camera, const std::vector<Eigen::Isometry3f> &poses3D, bool cameraSpace);

/**
 * Visualize calibration markers in pixel space
 */
void visualizeMarkers(const Camera &camera, const std::vector<Marker2D> &markers2D);

/**
 * Visualize 3D Rays in world space
 */
void visualizeRays(const Camera &camera, const std::vector<Ray> &rays3D);

/**
 * Visualize triangulated point cloud in world space
 */
void visualizeTriangulation(const Camera &camera, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount);

/**
 * Visualize camera distortion using a grid of size num
 */
void visualizeDistortion(const Camera &cameraCB, const Camera &cameraGT);

#endif