/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "eigenutil.hpp"

#include "tracking.hpp"
#include "calibration.hpp"

#include <vector>

/*
 * Visualization
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
void visualizePoses(const Camera &camera, const std::vector<Eigen::Isometry3f> &poses3D, bool cameraSpace, Color color = { 0, 0, 1 }, float scale = 100.0f);

/**
 * Visualize calibration markers in pixel space
 */
void visualizeMarkers(const Camera &camera, const std::vector<Marker2D> &markers2D, Color color = { 1, 1, 0 });

/**
 * Visualize 3D Rays in world space
 */
void visualizeRays(const Camera &camera, const std::vector<Ray> &rays3D, Color color = { 0, 0.5f, 0 });

/**
 * Visualize triangulated point cloud in world space
 */
void visualizeTriangulation(const Camera &camera, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount, Color colorNC = { 1, 0, 0 }, Color colorC = { 1, 0, 1 });

/**
 * Visualize camera distortion using a grid of size num
 */
void visualizeDistortion(const Camera &cameraCB, const Camera &cameraGT);

#endif // VISUALIZATION_H