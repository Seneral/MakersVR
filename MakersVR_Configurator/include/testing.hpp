/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef TESTING_H
#define TESTING_H

#include "eigenutil.hpp"
#include "tracking.hpp"

#include <vector>
#include <bitset>

/*
 * Testing
 */


/* Functions */

/**
 * Projects marker into image plane provided translation in centimeters and rotation, as well as a camera position
 */
void createMarkerProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<float> &pointSizes, std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker, const Camera &camera, const Eigen::Isometry3f &transform, float stdDeviation = 0.0f);

/**
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector3f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker3D, const Eigen::Isometry3f &transform);

/**
 * Analyze possibility of tracking algorithm to extract the pose based on ground truth data
 */
void analyzeTrackingAlgorithm(const std::vector<int> &visibleCount, const std::bitset<MAX_MARKER_POINTS> &triangulationMask, const std::vector<TriangulatedPoint> &points3D, const MarkerTemplate3D &marker3D, Eigen::Isometry3f gt);

#endif // TESTING_H