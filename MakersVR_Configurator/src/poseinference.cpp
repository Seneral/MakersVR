/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "poseinference.hpp"

#include "mesh.hpp"
#include <bitset>
#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef USE_EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

#define LINE_MERGE_ANGLE 10
#define LINE_MAX_CONNECTION_DIST 30 // In percent of full width
#define MAX_BASE_DIFF 0.2 // Max offset a base center can have relative to base line length
#define MAX_SIZE_DIFF 4 // Max size factor

/* Structures  */


/* Variables  */

static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3;
static const float PI = 3.14159265358979323846f;

int width, height;

// Viz resources
Mesh *vizCoordCross;
//ShaderProgram *shaderESLines;
//ShaderProgram *vizShader;
//GLuint vizMVPAdr;
GLuint vizLinesVBO; // Line buffer for uploading visualization lines to GPU

// Marker pose inference
double fovV, fovH;
#ifdef USE_CV
std::vector<cv::Point3f> marker3DTemplate;
cv::Matx33f camMat;
cv::Matx14f distort;
// Current values used for intrinsic guess
cv::Matx31d rotation;
cv::Matx31d translation;
#endif


/*
 * Initialize resources required for LED Tracking and setup camera parameters
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
//	vizShader = new ShaderProgram("../gl_shaders/UnlitES/vert.glsl", "../gl_shaders/UnlitES/frag.glsl");
//	vizMVPAdr = glGetUniformLocation(vizShader->ID, "MVP");
//	shaderESLines = new ShaderProgram("../gl_shaders/LineES/vert.glsl", "../gl_shaders/LineES/frag.glsl");
	// Setup VertexBufferObject for line data
	glGenBuffers(1, &vizLinesVBO);

#ifdef USE_CV
	marker3DTemplate = {
		cv::Point3f( 0.0f, 0.0f, 0.0f), // Center
		cv::Point3f( 4.0f, 0.0f, 0.0f), // Right
		cv::Point3f(-4.0f, 0.0f, 0.0f), // Left
		cv::Point3f( 0.0f, 4.0f, 0.0f)  // Header
	};
	// init dummy camera matrix
	float focalLen = (float)width; // Approximation
	camMat = cv::Matx33f(
		focalLen, 0, (float)width/2,
		0, focalLen, (float)height/2,
		0, 0, 1);
	distort = cv::Matx14f(0, 0, 0, 0);
	// Get camera parameters
	double focalLength, aspectRatio;
	cv::Point2d principalPoint;
	cv::calibrationMatrixValues(camMat, cv::Size2i(width, height), 3.68f, 2.76f, fovH, fovV, focalLength, principalPoint, aspectRatio);
#endif

	// TODO
	// Init resources and save camera parameters (size, fov, distortion coefficients, etc.)
}

/*
 * Finds all (potential) marker and also returns all free (unassociated) blobs left
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

/*
 * Infer the pose of a set of (likely) markers
 */
void inferMarkerPoses(std::vector<Marker> &markers, std::vector<Pose> &poses)
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
		cv::solvePnP(marker3DTemplate, marker2D, camMat, distort, rotation, translation, false, cv::SOLVEPNP_ITERATIVE);
		rotation = rotation * (180.0f/PI);
		translation = translation;

#ifdef MARKER_DEBUG
		std::cout << "Translation: " << translation << std::endl;
		std::cout << "Rotation: " << rotation << std::endl;
#endif

		poses.push_back({
			marker->center,
			{ (double)translation(0), (double)translation(1), (double)translation(2) },
			{ (double)rotation(0), (double)std::abs(rotation(1)), (double)rotation(2) }
		});
	}

#endif
}

/*
 * Infer the pose of a set of (likely) markers
 */
void inferMarkerPose(std::vector<Point> &marker, Pose &pose)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	std::vector<cv::Point2f> marker2D {
		cv::Point2f(marker[0].X, marker[0].Y),
		cv::Point2f(marker[1].X, marker[1].Y),
		cv::Point2f(marker[2].X, marker[2].Y),
		cv::Point2f(marker[3].X, marker[3].Y)
	};

	// use solvePnP to recreate rotation and translation
	cv::solvePnP(marker3DTemplate, marker2D, camMat, distort, rotation, translation, false, cv::SOLVEPNP_ITERATIVE);
	rotation = rotation * (180.0f/PI);
	translation = translation;

#ifdef MARKER_DEBUG
	std::cout << "Translation: " << translation << std::endl;
	std::cout << "Rotation: " << rotation << std::endl;
#endif

	/*pose = {
		&marker[0],
		{ (double)translation(0), (double)translation(1), (double)translation(2) },
		{ (double)rotation(0), (double)std::abs(rotation(1)), (double)rotation(2) }
	};*/

#endif
}

#ifdef USE_EIGEN
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
Eigen::Matrix4f createModelMatrix(const double *translation, const double *rotation, float scale)
{
	// Rotation
	Eigen::Matrix4f r = Eigen::Matrix4f::Identity();
	r.block(0,0,3,3) = Eigen::Matrix3f(Eigen::Quaternionf(
		  Eigen::AngleAxisf((float)rotation[0]*PI/180.0f, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf((float)rotation[1]*PI/180.0f, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf((float)rotation[2]*PI/180.0f, Eigen::Vector3f::UnitZ())
	));

	// Translation + Scale
	Eigen::Matrix4f ts;
	ts << scale, 0, 0, +(float)translation[0],
		  0, scale, 0, +(float)translation[1],
		  0, 0, scale, -(float)translation[2],
		  0, 0, 0, 1;

	// Combine to model view projection matrix
	return ts*r;
}
Eigen::Matrix4f createMVP(const double *translation, const double *rotation, float scale)
{
	return createProjectionMatrix() * createModelMatrix(translation, rotation, scale);
}
#endif

/*
 * Projects marker into image plane provided translation in centimeters and rotation, both relative to camera
 */
void projectMarker(std::vector<Point> &imagePoints, double *translation, double *rotation, float ptScale)
{
	#if defined(USE_EIGEN) && defined(USE_CV)
	// Create MVP in camera space, translation in centimeters
	Eigen::Matrix4f mvp = createMVP(translation, rotation, 1.0f);
	// Project each marker point into image space
	imagePoints.resize(marker3DTemplate.size());
	for (int i = 0; i < marker3DTemplate.size(); i++)
	{
		cv::Point3f *markerPt = &marker3DTemplate[i];
		Eigen::Vector4f point(markerPt->x, markerPt->y, markerPt->z, 1.0f);
		point = mvp * point;
		imagePoints[i].X = (point.x() / point.w()+1)/2 * width;
		imagePoints[i].Y = (point.y() / point.w()+1)/2 * height;
		imagePoints[i].S = ptScale * 100/((float)translation[2]);
	}
	#endif
}

/*
 * Visualize poses
 */
void visualizePoses(const std::vector<Point> &blobs, const std::vector<Marker> &markers, const std::vector<Pose> &poses)
{
	glColor3f(1,0,0);
//	glBegin(GL_POINTS);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for (int i = 0; i < blobs.size(); i++)
	{
		GLfloat s = blobs[i].S*2.0f;
		glPointSize(s <= 1? 1.0f : s);
		glBegin(GL_POINTS);
		glVertex3f(blobs[i].X/width*2-1, blobs[i].Y/height*2-1, 0.5f);
		glEnd();
	}
//	glEnd();

	// Just render them using normal OpenGL in 3D Space, assuming render targets, etc. have been set

	//vizShader->use();
	glColor3f(0, 0, 1);

	for (int i = 0; i < poses.size(); i++)
	{
		const Pose *pose = &poses[i];

//		printf("Drawing Translation: (%f, %f, %f)\n", pose->translation[0], pose->translation[1], pose->translation[2]);
//		printf("Drawing Rotation: (%f, %f, %f)\n", pose->rotation[0], pose->rotation[1], pose->rotation[2]);

#ifdef USE_EIGEN
		// Create projection and model(view) matrix from pose and set
//		glMatrixMode(GL_PROJECTION);
//		glLoadMatrixf((float*)createProjectionMatrix().data());
//		glMatrixMode(GL_MODELVIEW);
//		glLoadMatrixf((float*)createModelMatrix(pose->translation, pose->rotation, 4.0f).data());
//		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf((float*)createMVP(pose->translation, pose->rotation, 10.0f).data());
#endif

		vizCoordCross->draw();
	}


	// Add marker points for visualization
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

	// Render viz points
	glLoadIdentity();
	glColor3f(0, 1, 0);
	glLineWidth(2.0f);
	//shaderESLines->use();
	glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5 * vizLinePoints.size(), &vizLinePoints[0], GL_STREAM_DRAW);
	glVertexAttribPointer(vPosAdr, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void *)0);
	glEnableVertexAttribArray(vPosAdr);
	glVertexAttribPointer(vColAdr, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void *)2);
	glEnableVertexAttribArray(vColAdr);
	glDrawArrays(GL_LINES, 0, (int)vizLinePoints.size());
}

void cleanPoseInference()
{
	// TODO
}
