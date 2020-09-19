/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_TESTING
#define DEF_TESTING

#define CAMERA_NOISE_STDDEV 0.05f

#include "eigenutil.hpp"
#include "tracking.hpp"
#include "calibration.hpp"

#include <vector>
#include <bitset>

/*
 * Testing
 */


/* Functions */

/**
 * Initialize resources for testing
 */
void initTesting();

/**
 * Initialize resources for visualization
 */
void initVisualization();

/**
 * Cleanup of resources
 */
void cleanTesting();

/**
 * Sets the specified marker data as the current calibration target
 */
void setActiveCalibrationMarker(const DefMarker &markerData);

/**
 * Sets the specified marker data as the current tracking target
 */
void setActiveTrackingMarker(const DefMarker &markerData);

/**
 * Projects marker into image plane provided translation in centimeters and rotation, as well as a camera position
 */
void createMarkerProjection(std::vector<Point> &points2D, std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker, const Camera &camera, const Eigen::Isometry3f &transform, float stdDeviation = 0.0f);

/**
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector3f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const Eigen::Isometry3f &transform);

/**
 * Analyze possibility of tracking algorithm to extract the pose based on ground truth data
 */
void analyzeTrackingAlgorithm(std::vector<int> &visibleCount, std::bitset<MAX_MARKER_POINTS> &triangulationMask, std::vector<TriangulatedPoint> &points3D, Eigen::Isometry3f gt);

/**
 * Visualize multi-camera triangulated markers
 * points2D: Visible points projected into camera view
 * rays3D: rays from all cameras
 * points3D: Points triangulated from the algorithm, without conflicts first
 * nonconflictedCount: Amount of nonconflicted points in points3D
 * triangulatedPoints3D: Points which could have been triangulated (are visible from more than one camera) -- used for testing only
 * poses3D: Inferred 3D poses
 */
void visualizeMarkers(const Camera &camera, const std::vector<Point> &points2D, const std::vector<Ray> &rays3D, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount, const std::vector<Eigen::Vector3f> &triangulatedPoints3D, const std::vector<Eigen::Isometry3f> &poses3D);

/**
 * Visualize single-camera poses
 */
void visualizePoses(const Camera &camera, const std::vector<Point> &points2D, const std::vector<Marker> &markers2D, const std::vector<Eigen::Isometry3f> &poses3D);

#endif