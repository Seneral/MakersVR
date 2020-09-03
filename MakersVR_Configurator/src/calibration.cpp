/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define USE_CV

#define LINE_MERGE_ANGLE 10
#define LINE_MAX_CONNECTION_DIST 30 // In percent of full width
#define MAX_BASE_DIFF 0.2 // Max offset a base center can have relative to base line length
#define MAX_SIZE_DIFF 4 // Max size factor

#include "calibration.hpp"
#include "wxbase.hpp" // wxLog*

#include <bitset>

#ifdef USE_CV
// Don't include full library, only needed modules
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#endif

/**
 * Calibration
 */


/* Structures */

typedef struct MarkerCandidate
{
	uint8_t c;
	uint8_t a;
	uint8_t b;
	uint8_t h;
	float error;
} MarkerCandidate;


/* Variables */

Camera camera;

#ifdef USE_CV
std::vector<cv::Point3f> cv_marker3DTemplate;
cv::Matx33f cv_camMat;
cv::Matx14f cv_distort;
// Current values used for intrinsic guess
cv::Matx31d cv_rotation;
cv::Matx31d cv_translation;
#endif


/* Functions */

/**
 * Initialize resources for calibration
 */
void initCalibration(int Width, int Height)
{
	camera.width = Width;
	camera.height = Height;

#ifdef USE_CV
	// Init dummy camera matrix
	float focalLen = (float)camera.width; // Approximation
	cv_camMat = cv::Matx33f(
		focalLen, 0, (float)camera.width/2,
		0, focalLen, (float)camera.height/2,
		0, 0, 1);
	cv_distort = cv::Matx14f(0, 0, 0, 0);

	// Get camera parameters
	double focalLength, aspectRatio;
	double fovH, fovV;
	cv::Point2d principalPoint;
	cv::calibrationMatrixValues(cv_camMat, cv::Size2i(camera.width, camera.height), 3.68f, 2.76f, fovH, fovV, focalLength, principalPoint, aspectRatio);
	camera.fovH = (float)fovH;
	camera.fovV = (float)fovV;
#endif
}

/**
 * Cleanup of resources
 */
void cleanCalibration()
{

}

/**
 * Returns the camera with intrinsic parameters calibrated
 */
Camera getCalibratedCamera()
{
	return camera;
}

/**
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(std::vector<Point> &points2D, std::vector<Marker> &markers2D, std::vector<Point*> &freePoints2D)
{
	static std::vector<MarkerCandidate> markerCandidates;
	static std::bitset<128> markerTag;

	// Find and extract all markers candidates (base line of 3 dots plus header)
	// Step 1: Find all base lines (3 points in a straight line with equal distance)
	// Step 2: For each line, find the leading point on one side
	markerCandidates.clear();
	for (uint8_t c = 0; c < points2D.size(); c++)
	{
		Point *ptC = &points2D[c];

		for (uint8_t a = 0; a < points2D.size(); a++)
		{
			if (a == c) continue;
			Point *ptA = &points2D[a];

			// Check size difference
			float sizeMin = std::min(ptC->S, ptA->S);
			float sizeMax = std::max(ptC->S, ptA->S);
			if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

			for (uint8_t b = a+1; b < points2D.size(); b++)
			{
				if (b == c) continue;

				// Check size difference
				Point *ptB = &points2D[b];
				sizeMin = std::min(sizeMin, ptB->S);
				sizeMax = std::max(sizeMax, ptB->S);
				if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

				// Get base parameters
				float baseX = ptA->X - ptB->X;
				float baseY = ptA->Y - ptB->Y;
				float baseLen = std::sqrt(baseX*baseX + baseY*baseY);

				// Check if c is at the center of a and b, building a base line
				float diffX = ptC->X * 2 - ptA->X - ptB->X;
				float diffY = ptC->Y * 2 - ptA->Y - ptB->Y;
				float centerDiff = std::sqrt((float)diffX*diffX + diffY*diffY) / 2;
				float centerError = centerDiff / baseLen;

				// if (std::abs(diffX) + std::abs(diffY) <= MAX_BASE_PX_DIST)
				if (centerError <= MAX_BASE_DIFF)
				{ // Found a base
					float baseAngle = std::atan(baseY / baseX) / PI * 180;

					// Find closest blob as heading
					int h = -1;
					float headLen = (float)1000000.0f;
					for (int i = 0; i < points2D.size(); i++)
					{
						if (i == c || i == a || i == b) continue;
						Point *ptHC = &points2D[i];

						// Check size difference
						if (std::min(sizeMin, ptHC->S) * MAX_SIZE_DIFF < std::max(sizeMax, ptHC->S)) continue;

						// Get Head parameters
						float hX = ptC->X - ptHC->X;
						float hY = ptC->Y - ptHC->Y;
						float hLen = std::sqrt(hX*hX + hY*hY);
						if (hLen < baseLen*10.0 && hLen < headLen)
						{ // Best head candidate so far
							headLen = hLen;
							h = i;
						}
					}
					if (h != -1)
					{
						Point *ptH = &points2D[h];

						// Get Head parameters
						float headX = ptC->X - ptH->X;
						float headY = ptC->Y - ptH->Y;
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
	markers2D.clear();
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

		markers2D.push_back({
			&points2D[mc->c],
			&points2D[mc->a],
			&points2D[mc->b],
			&points2D[mc->h],
		});

#ifdef MARKER_DEBUG
		std::cout << "Chose marker with error " << mc->error << "!" << std::endl;
#endif
	}

	// Find blobs not part of any marker
	freePoints2D.clear();
	if (points2D.size() > markers2D.size()*4)
	{ // There are blobs not part of any marker
		for (int b = 0; b < points2D.size(); b++)
		{
			if (!markerTag.test(b))
			{ // Blob is not part of any marker
				freePoints2D.push_back(&points2D[b]);
			}
		}
	}
}

/**
 * Interprets OpenCV position and rotation in camera spaces and transforms it into correct scene transformations
 */
void interpretCVSpace(const cv::Matx31d &cvPos, const cv::Matx31d &cvRot, Eigen::Vector3f *position, Eigen::Matrix3f *rotation)
{
	// Z-Axis of OpenGL/Blender coordinate frame is opposite of OpenCV
	// OpenGL/Blender is -z foward, +z towards viewer
	*position = Eigen::Vector3f(cvPos(0), cvPos(1), -cvPos(2));

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
	*rotation = getRotationXYZ(rotAngles);
}

/**
 * Infer the pose of a set of (likely) markers and the known camera position to transform into world space
 */
void inferMarkerPoses(std::vector<Marker> &markers2D, const Camera &camera, std::vector<Eigen::Isometry3f> &poses3D)
{
	poses3D.clear();
	poses3D.reserve(markers2D.size());

#ifdef USE_CV
	// Try to infer pose for each marker
	for (int m = 0; m < markers2D.size(); m++)
	{
		Marker *marker = &markers2D[m];

		// Create CV 2D array of marker points
		std::vector<cv::Point2f> marker2D {
			cv::Point2f(marker->center->X, marker->center->Y),
			cv::Point2f(marker->endA->X, marker->endA->Y),
			cv::Point2f(marker->endB->X, marker->endB->Y),
			cv::Point2f(marker->header->X, marker->header->Y)
		};

		// use solvePnP to recreate rotation and translation
		cv::solvePnP(cv_marker3DTemplate, marker2D, cv_camMat, cv_distort, cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

		// Convert from OpenCV to Blender/OpenGL space
		Eigen::Vector3f pos;
		Eigen::Matrix3f rot;
		interpretCVSpace(cv_translation, cv_rotation, &pos, &rot);

		// Finally, account for camera transform
		Eigen::Isometry3f pose;
		pose.linear() = camera.transform.linear() * rot;
		pose.translation() = camera.transform * pos;
		poses3D.push_back(pose);
	}
#endif
}

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void inferMarkerPoseGeneric(std::vector<Point> &points2D, const Camera &camera, Eigen::Isometry3f &pose3D)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	std::vector<cv::Point2f> cv_points2D;
	for (int i = 0; i < points2D.size(); i++)
		cv_points2D.push_back(cv::Point2f(points2D[i].X, points2D[i].Y));

	// use solvePnP to recreate rotation and translation
	cv::solvePnP(cv_marker3DTemplate, cv_points2D, cv_camMat, cv_distort, cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

	// Convert from OpenCV to Blender/OpenGL space
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot;
	interpretCVSpace(cv_translation, cv_rotation, &pos, &rot);

	// Account for camera transform
	pose3D.linear() = camera.transform.linear() * rot;
	pose3D.translation() = camera.transform * pos;
#endif
}

/**
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount()
{
#ifdef USE_CV
	return cv_marker3DTemplate.size() < 4? 4 : cv_marker3DTemplate.size();
#endif
}