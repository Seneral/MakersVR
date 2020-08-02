/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define LINE_MERGE_ANGLE 10
#define LINE_MAX_CONNECTION_DIST 30 // In percent of full width
#define MAX_BASE_DIFF 0.2 // Max offset a base center can have relative to base line length
#define MAX_SIZE_DIFF 4 // Max size factor

#define USE_CV

#include "poseinference.hpp"

#include "mesh.hpp"
#include "main.h"

#include "util.h"

#include <bitset>
#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef USE_CV
// Don't include full library, only needed modules
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#endif

/* Variables  */

// Marker pose inference
int width, height;
double fovV, fovH;
DefMarker marker3DTemplate;

#ifdef USE_CV
std::vector<cv::Point3f> cv_marker3DTemplate;
cv::Matx33f cv_camMat;
cv::Matx14f cv_distort;
// Current values used for intrinsic guess
cv::Matx31d cv_rotation;
cv::Matx31d cv_translation;
#endif

// Visualization
static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3;
Mesh *vizCoordCross;
GLuint vizLinesVBO; // Line buffer for uploading visualization lines to GPU

// Helper functions for projection and visualization
Eigen::Matrix4f createProjectionMatrix();
Eigen::Matrix4f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale);
Eigen::Matrix4f createViewMatrix(const Transform &camera);
Eigen::Matrix4f createMVP(const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale);

/*
 * Initialize resources for pose inference
 */
void initPoseInference(int Width, int Height)
{
	width = Width;
	height = Height;

	// Init visualization coordinate cross
	vizCoordCross = new Mesh ({ POS, COL }, {
		 0, 0, 0, 1, 0, 0,
		 1, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 1, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1,
		 0, 0, 1, 0, 0, 1,
	}, {});
	vizCoordCross->setMode(GL_LINES);
	// Setup VertexBufferObject for line data
	glGenBuffers(1, &vizLinesVBO);

#ifdef USE_CV
	// Init dummy camera matrix
	float focalLen = (float)width; // Approximation
	cv_camMat = cv::Matx33f(
		focalLen, 0, (float)width/2,
		0, focalLen, (float)height/2,
		0, 0, 1);
	cv_distort = cv::Matx14f(0, 0, 0, 0);
	// Get camera parameters
	double focalLength, aspectRatio;
	cv::Point2d principalPoint;
	cv::calibrationMatrixValues(cv_camMat, cv::Size2i(width, height), 3.68f, 2.76f, fovH, fovV, focalLength, principalPoint, aspectRatio);
#endif

	// TODO
	// Init resources and save camera parameters (size, fov, distortion coefficients, etc.)
}

/*
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(std::vector<Point> &blobs, std::vector<Marker> &markers, std::vector<Point*> &freeBlobs) 
{
	static std::vector<MarkerCandidate> markerCandidates;
	static std::bitset<128> markerTag;

	// Find and extract all markers candidates (base line of 3 dots plus header)
	// Step 1: Find all base lines (3 points in a straight line with equal distance)
	// Step 2: For each line, find the leading point on one side
	markerCandidates.clear();
	for (uint8_t c = 0; c < blobs.size(); c++)
	{
		Point *blobC = &blobs[c];

		for (uint8_t a = 0; a < blobs.size(); a++)
		{
			if (a == c) continue;
			Point *blobA = &blobs[a];

			// Check size difference
			float sizeMin = std::min(blobC->S, blobA->S);
			float sizeMax = std::max(blobC->S, blobA->S);
			if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

			for (uint8_t b = a+1; b < blobs.size(); b++)
			{
				if (b == c) continue;

				// Check size difference
				Point *blobB = &blobs[b];
				sizeMin = std::min(sizeMin, blobB->S);
				sizeMax = std::max(sizeMax, blobB->S);
				if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

				// Get base parameters
				float baseX = blobA->X - blobB->X;
				float baseY = blobA->Y - blobB->Y;
				float baseLen = std::sqrt(baseX*baseX + baseY*baseY);

				// Check if c is at the center of a and b, building a base line
				float diffX = blobC->X * 2 - blobA->X - blobB->X;
				float diffY = blobC->Y * 2 - blobA->Y - blobB->Y;
				float centerDiff = std::sqrt((float)diffX*diffX + diffY*diffY) / 2;
				float centerError = centerDiff / baseLen;

				// if (std::abs(diffX) + std::abs(diffY) <= MAX_BASE_PX_DIST)
				if (centerError <= MAX_BASE_DIFF)
				{ // Found a base
					float baseAngle = std::atan(baseY / baseX) / PI * 180;

					// Find closest blob as heading
					int h = -1;
					float headLen = (float)width;
					for (int i = 0; i < blobs.size(); i++)
					{
						if (i == c || i == a || i == b) continue;
						Point *blob = &blobs[i];

						// Check size difference
						if (std::min(sizeMin, blob->S) * MAX_SIZE_DIFF < std::max(sizeMax, blob->S)) continue;

						// Get Head parameters
						float hX = blobC->X - blob->X;
						float hY = blobC->Y - blob->Y;
						float hLen = std::sqrt(hX*hX + hY*hY);
						if (hLen < baseLen*10.0 && hLen < headLen)
						{ // Best head candidate so far
							headLen = hLen;
							h = i;
						}
					}
					if (h != -1)
					{
						Point *blobH = &blobs[h];

						// Get Head parameters
						float headX = blobC->X - blobH->X;
						float headY = blobC->Y - blobH->Y;
						float headLen = std::sqrt(headX*headX + headY*headY);

						// Get inner angle between base and head
						float innerDot = baseX*headX + baseY*headY;
						float innerAngle = std::acos(innerDot / (baseLen * headLen)) / PI * 180 - 90;

						// Register marker
						markerCandidates.push_back({
							c, a, b, (uint8_t)h, centerError
						});
#ifdef MARKER_DEBUG
						std::cout << "Registered marker with base angle " << baseAngle << ", length " << baseLen << " and error " << centerError << " head rel angle " << innerAngle << " and length " << headLen << "!" << std::endl;
#endif
						goto foundMarker;
					}
				}
			}

			foundMarker: continue;
		}
	}

	// Sort marker candidates by accuracy
	std::sort(markerCandidates.begin(), markerCandidates.end(), [](MarkerCandidate m1, MarkerCandidate m2) {
		return m1.error < m2.error;
	});

	// Find best marker candidates and associate them
	markers.clear();
	markerTag.reset();
	for (int m = 0; m < markerCandidates.size(); m++)
	{
		MarkerCandidate *mc = &markerCandidates[m];
		if (markerTag[mc->c] || markerTag[mc->a] || markerTag[mc->b]) continue;

		// Set blobs as used
		markerTag.set(mc->a);
		markerTag.set(mc->b);
		markerTag.set(mc->c);
		markerTag.set(mc->h);

		markers.push_back({
			&blobs[mc->c],
			&blobs[mc->a],
			&blobs[mc->b],
			&blobs[mc->h],
		});

#ifdef MARKER_DEBUG
		std::cout << "Chose marker with error " << mc->error << "!" << std::endl;
#endif
	}

	// Find blobs not part of any marker
	freeBlobs.clear();
	if (blobs.size() > markers.size()*4)
	{ // There are blobs not part of any marker
		for (int b = 0; b < blobs.size(); b++)
		{
			if (!markerTag.test(b))
			{ // Blob is not part of any marker
				freeBlobs.push_back(&blobs[b]);
			}
		}
	}
}

/**
 * Interprets OpenCV position and rotation in camera spaces and transforms it into correct scene transformations 
 */
void interpretCVSpace(const Transform &camera, const cv::Matx31d &cvPos, const cv::Matx31d &cvRot, Eigen::Vector3f *position, Eigen::Matrix3f *rotation)
{
	// Z-Axis of OpenGL/Blender coordinate frame is opposite of OpenCV
	// OpenGL/Blender is -z foward, +z towards viewer
	Eigen::Vector3f pos(cvPos(0), cvPos(1), -cvPos(2));

	// Convert rodrigues rotation parameters into rotation matrix
	cv::Matx33d cvRotMat;
	cv::Rodrigues(cvRot, cvRotMat);
	// Copy rotation matrix into Eigen rotation matrix (with transpose to account for row/column storage)
	Eigen::Matrix3d rotD;
	cv::Mat rotMat_Shadow(3, 3, CV_64F, rotD.data(), (size_t)(rotD.stride()*sizeof(double)));
	cv::transpose(cvRotMat, rotMat_Shadow);
	// Convert to euler angles temporarily
	Eigen::Vector3f rotAngles = getEulerXYZ(rotD.cast<float>());
	// OpenCV rotation needs to be inverted, in addition to accounting for the z-axis flip
	rotAngles = Eigen::Vector3f(-rotAngles.x(), -rotAngles.y(), +rotAngles.z());
	// Convert back to matrix
	Eigen::Matrix3f rot = getRotationXYZ(rotAngles);

	// Finally, remove camera transform
	*position = camera.rot*pos + camera.pos;
	*rotation = camera.rot*rot;
}

/*
 * Infer the pose of a set of (likely) markers and the known camera position to transform into world space
 */
void inferMarkerPoses(std::vector<Marker> &markers, const Transform &camera, std::vector<Pose> &poses)
{
	poses.clear();
	poses.reserve(markers.size());

#ifdef USE_CV
	// Try to infer pose for each marker
	for (int m = 0; m < markers.size(); m++)
	{
		Marker *marker = &markers[m];

		// Create CV 2D array of marker points
		std::vector<cv::Point2f> marker2D {
			cv::Point2f(marker->center->X, marker->center->Y),
			cv::Point2f(marker->endA->X, marker->endA->Y),
			cv::Point2f(marker->endB->X, marker->endB->Y),
			cv::Point2f(marker->header->X, marker->header->Y)
		};

		// use solvePnP to recreate rotation and translation
		cv::solvePnP(cv_marker3DTemplate, marker2D, cv_camMat, cv_distort, cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

		// Convert from OpenCV to Blender/OpenGL space and account for camera transform
		Eigen::Vector3f pos;
		Eigen::Matrix3f rot;
		interpretCVSpace(camera, cv_translation, cv_rotation, &pos, &rot);
		poses.push_back({ marker->center, pos, rot });
	}

#endif
}

/*
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void inferMarkerPoseGeneric(std::vector<Point> &marker, const Transform &camera, Pose &pose)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	std::vector<cv::Point2f> marker2D;
	for (int i = 0; i < marker.size(); i++)
		marker2D.push_back(cv::Point2f(marker[i].X, marker[i].Y));

	// use solvePnP to recreate rotation and translation
	cv::solvePnP(cv_marker3DTemplate, marker2D, cv_camMat, cv_distort, cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

	// Convert from OpenCV to Blender/OpenGL space and account for camera transform
	pose.center = &marker[0];
	interpretCVSpace(camera, cv_translation, cv_rotation, &pose.trans, &pose.rot);
#endif
}

/*
 * Visualize poses
 */
void visualizePoses(const Transform &camera, const std::vector<Point> &blobs, const std::vector<Marker> &markers, const std::vector<Pose> &poses)
{
	// Reset matrices
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Render all blobs
	glColor3f(1,0,0);
	for (int i = 0; i < blobs.size(); i++)
	{
		GLfloat s = blobs[i].S*2.0f;
		//glPointSize(s <= 1? 1.0f : s);
		glPointSize(6.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(blobs[i].X/width*2-1, blobs[i].Y/height*2-1, 0.9f);
		glEnd();
	}

	// Render inferred poses' dots
/*	glColor3f(0,1,0);
	for (int i = 0; i < poses.size(); i++)
	{
		const Pose *pose = &poses[i];
		Eigen::Matrix4f mvp = createMVP(camera, pose->trans, pose->rot, 1.0f);
		for (int j = 0; j < marker3DTemplate.pts.size(); j++)
		{
			DefMarkerPoint *markerPt = &marker3DTemplate.pts[j];
			Eigen::Vector4f point(markerPt->pos.x(), markerPt->pos.y(), markerPt->pos.z(), 1.0f);
			point = mvp * point;
			glPointSize(4.0f);
			glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
			glVertex3f(point.x() / point.w(), point.y() / point.w(), point.z() / point.w());
			glEnd();
		}
	}
	glLoadIdentity();*/

	// Render inferred poses
	glColor3f(0, 0, 1);
	for (int i = 0; i < poses.size(); i++)
	{
		const Pose *pose = &poses[i];
		glLoadMatrixf((float*)createMVP(camera, pose->trans, pose->rot, 4.0f).data());
		vizCoordCross->draw();
	}
	glLoadIdentity();

	// Construct line mesh of detected markers
	std::vector<LinePoint> vizLinePoints;
	for (int m = 0; m < markers.size(); m++)
	{
		const Marker *marker = &markers[m];
		// Display base
		vizLinePoints.push_back({
			(marker->endA->X / width - 0.5f) * 2.0f,
			(marker->endA->Y / height - 0.5f) * 2.0f,
			0.0, 0.0, 1.0
		});
		vizLinePoints.push_back({
			(marker->endB->X / width - 0.5f) * 2.0f,
			(marker->endB->Y / height - 0.5f) * 2.0f,
			0.0, 0.0, 1.0 });
		// Display heading
		vizLinePoints.push_back({
			(marker->center->X / width - 0.5f) * 2.0f,
			(marker->center->Y / height - 0.5f) * 2.0f,
			1.0, 0.0, 1.0 });
		vizLinePoints.push_back({
			(marker->header->X / width - 0.5f) * 2.0f,
			(marker->header->Y / height - 0.5f) * 2.0f,
			1.0, 0.0, 1.0 });
	}
	
	// Render lines
	glColor3f(0, 1, 0);
	glLineWidth(2.0f);
	glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5 * vizLinePoints.size(), &vizLinePoints[0], GL_STREAM_DRAW);
	glVertexAttribPointer(vPosAdr, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void *)0);
	glEnableVertexAttribArray(vPosAdr);
//	glVertexAttribPointer(vColAdr, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void *)2);
//	glEnableVertexAttribArray(vColAdr);
	glDrawArrays(GL_LINES, 0, (int)vizLinePoints.size());
}

/**
 * Cleanup of resources
 */
void cleanPoseInference()
{
	// TODO
}

/*
 * Sets the specified marker data as the current target
 */
void setActiveMarkerData(DefMarker markerData)
{
	marker3DTemplate = markerData;
#ifdef USE_CV
	cv_marker3DTemplate.clear();
	for (int i = 0; i < marker3DTemplate.pts.size(); i++)
	{
		DefMarkerPoint *pt = &marker3DTemplate.pts[i];
		cv_marker3DTemplate.push_back(cv::Point3f(pt->pos.x(), pt->pos.y(), pt->pos.z()));
	}
#endif
}

/*
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount()
{
	return marker3DTemplate.pts.size();
}

/*
 * Projects marker into image plane provided translation in centimeters and rotation, both relative to camera
 */
void createMarkerProjection(std::vector<Point> &imagePoints, const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float ptScale)
{
	// Create MVP in camera space, translation in centimeters
	Eigen::Matrix4f mvp = createMVP(camera, translation, rotation, 1.0f);
	// Project each marker point into image space
	imagePoints.clear();
	for (int i = 0; i < marker3DTemplate.pts.size(); i++)
	{
		DefMarkerPoint *markerPt = &marker3DTemplate.pts[i];
		Eigen::Vector4f point(markerPt->pos.x(), markerPt->pos.y(), markerPt->pos.z(), 1.0f);
		point = mvp * point;
		if (std::abs(point.x() / point.w()) > 1 || std::abs(point.y() / point.w()) > 1) continue;
		imagePoints.push_back({
			(point.x() / point.w() + 1)/2 * width,
			(point.y() / point.w() + 1)/2 * height,
			ptScale * 100/translation.z()
		});
	}
}

/* Utility */

Eigen::Matrix4f createProjectionMatrix()
{
	float zN = 10.0f, zF = 1000.0f; // 0.1m-10m
	float a = -(zF+zN)/(zF-zN), b = -2*zN*zF/(zF-zN);
	float sX = (float)(1.0f/std::tan(fovH*PI/360.0f)), sY = (float)(1.0f/std::tan(fovV*PI/360.0f));

	// Projection
	Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
	p << sX, 0, 0, 0,
		0, sY, 0, 0,
		0, 0, a, b,
		0, 0, -1, 0;

	return p;
}
Eigen::Matrix4f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale)
{
	// Rotation
	Eigen::Matrix4f r = Eigen::Matrix4f::Identity();
	r.block(0,0,3,3) = rotation;

	// Translation + Scale
	Eigen::Matrix4f ts;
	ts << scale, 0, 0, translation.x(),
		  0, scale, 0, translation.y(),
		  0, 0, scale, translation.z(),
		  0, 0, 0, 1;

	// Combine to model view projection matrix
	return ts*r;
}
Eigen::Matrix4f createViewMatrix(const Transform &camera)
{
	return createModelMatrix(camera.pos, camera.rot, 1).inverse();
}
Eigen::Matrix4f createMVP(const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale)
{
	return createProjectionMatrix() * createViewMatrix(camera) * createModelMatrix(translation, rotation, scale);
}