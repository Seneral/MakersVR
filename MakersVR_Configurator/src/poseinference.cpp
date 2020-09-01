/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define LINE_MERGE_ANGLE 10
#define LINE_MAX_CONNECTION_DIST 30 // In percent of full width
#define MAX_BASE_DIFF 0.2 // Max offset a base center can have relative to base line length
#define MAX_SIZE_DIFF 4 // Max size factor
#define MAX_CAMERA_COUNT 3
#define NUM_CLOSEST_RELATIONS 4
#define MIN_DISTANCE_RELATIONS 2 // cm
#define SIGMA_ERROR 3 // Sigma value used to account for estimated error of triangulated points


#define USE_CV

#include "poseinference.hpp"

#include "mesh.hpp"
#include "main.h"

#include "util.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <set>

#include "Eigen/SVD"

#ifdef USE_CV
// Don't include full library, only needed modules
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#endif

/* Operators */
bool operator<(const struct PointRelation& a, const struct PointRelation& b) { return a.distance < b.distance; }
bool ErrorRangeComp::operator() (const PointRelation& rel, float value) { return rel.distance < value-error; }
bool ErrorRangeComp::operator() (float value, const PointRelation& rel) { return value+error < rel.distance; }


/* Variables  */

// Marker pose inference
int width, height;
double fovV, fovH;
MarkerLookup marker3D;

#ifdef USE_CV
std::vector<cv::Point3f> cv_marker3DTemplate;
cv::Matx33f cv_camMat;
cv::Matx14f cv_distort;
// Current values used for intrinsic guess
cv::Matx31d cv_rotation;
cv::Matx31d cv_translation;
#endif

// Visualization
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
void findMarkerCandidates(std::vector<Point> &points2D, std::vector<Marker> &markers2D, std::vector<Point*> &freePoints2D)
{
	typedef struct MarkerCandidate
	{
		uint8_t c;
		uint8_t a;
		uint8_t b;
		uint8_t h;
		float error;
	} MarkerCandidate;

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
					float headLen = (float)width;
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
void inferMarkerPoses(std::vector<Marker> &markers2D, const Transform &camera, std::vector<Pose> &poses3D)
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

		// Convert from OpenCV to Blender/OpenGL space and account for camera transform
		Eigen::Vector3f pos;
		Eigen::Matrix3f rot;
		interpretCVSpace(camera, cv_translation, cv_rotation, &pos, &rot);
		poses3D.push_back({ pos, rot });
	}

#endif
}

/*
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void inferMarkerPoseGeneric(std::vector<Point> &points2D, const Transform &camera, Pose &pose3D)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	std::vector<cv::Point2f> cv_points2D;
	for (int i = 0; i < points2D.size(); i++)
		cv_points2D.push_back(cv::Point2f(points2D[i].X, points2D[i].Y));

	// use solvePnP to recreate rotation and translation
	cv::solvePnP(cv_marker3DTemplate, cv_points2D, cv_camMat, cv_distort, cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

	// Convert from OpenCV to Blender/OpenGL space and account for camera transform
	interpretCVSpace(camera, cv_translation, cv_rotation, &pose3D.trans, &pose3D.rot);
#endif
}

/*
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void castRays(const std::vector<Point> &points2D, const Transform &camera, std::vector<Ray> &rays3D)
{
	Eigen::Matrix4f vpInv = (createProjectionMatrix() * createViewMatrix(camera)).inverse();
	for (int i = 0; i < points2D.size(); i++)
	{
		Eigen::Vector4f start(camera.pos.x(), camera.pos.y(), camera.pos.z(), 1.0f);
		Eigen::Vector4f end(points2D[i].X/width*2-1, points2D[i].Y/height*2-1, 1.0f, 1.0f);
		end = vpInv * end; // Clip space to world space
		end = end / end.w();
		rays3D.push_back({ start, (end - start).normalized(), 0 });
	}
}

void rayIntersect(const Ray *ray1, const Ray *ray2, float *sec1, float *sec2)
{
	// Pseudo ray intersection by computing the closest point of two lines
	// Due to usual camera placements, for most cases the following assumptions can be made:
	// Parallel case can be ignored, and rays can safely be considered as lines
	// Reference: http://geomalgorithms.com/a07-_distance.html

	float a = ray1->dir.dot(ray1->dir);
	float b = ray1->dir.dot(ray2->dir);
	float c = ray2->dir.dot(ray2->dir);

	float d = ray1->dir.dot(ray1->pos-ray2->pos);
	float e = ray2->dir.dot(ray1->pos-ray2->pos);

	float s = a*c - b*b;
	*sec1 = (b*e - c*d) / s;
	*sec2 = (a*e - b*d) / s;
}

/*
 * Calculate the intersection points between rays of separate groups
 */
int triangulateRayIntersections(std::vector<std::vector<Ray>*> &rayGroups, std::vector<TriangulatedPoint> &points3D, std::vector<std::vector<int>> &conflicts, float errorLimit)
{
	struct Intersection
	{
		Eigen::Vector4f center;
		float error;
		Intersection *merge;
		Ray* rays[MAX_CAMERA_COUNT];
	};

	int groupCnt = rayGroups.size();
	if (groupCnt > MAX_CAMERA_COUNT)
	{
		wxLogError("Exceeded maximum camera code of %d! Recompile needed.", MAX_CAMERA_COUNT);
		groupCnt = MAX_CAMERA_COUNT;
	}

	std::vector<Intersection> intersections;

	#define RAYNUM(rays, num) (int)(rays[num] == NULL? -1 : (((intptr_t)rays[num]-(intptr_t)rayGroups[num]->data())/(sizeof(Ray)) + num*10))
	#define IXNUM(ix) (int)(ix == NULL? -1 : (((intptr_t)ix-(intptr_t)intersections.data())/(sizeof(Intersection))))

	// Fill with candidate intersections
	for (int i = 0; i < groupCnt-1; i++)
	{
		std::vector<Ray> *rays1 = rayGroups[i];
		for (int j = i+1; j < groupCnt; j++)
		{
			std::vector<Ray> *rays2 = rayGroups[j];
			for (int v = 0; v < rays1->size(); v++)
			{
				Ray *ray1 = &rays1->at(v);
				for (int w = 0; w < rays2->size(); w++)
				{
					Ray *ray2 = &rays2->at(w);
					// Calculate ray intersection
					float sec1, sec2;
					rayIntersect(ray1, ray2, &sec1, &sec2);
					Eigen::Vector4f pos1 = ray1->pos + ray1->dir * sec1;
					Eigen::Vector4f pos2 = ray2->pos + ray2->dir * sec2;
					// Calculate distance
					float errorSq = (pos1-pos2).squaredNorm()/4;
					if (errorSq > errorLimit*errorLimit) continue;
					// Increase ray intersection count
					ray1->intersectionCount++;
					ray2->intersectionCount++;
					// Register intersection
					Intersection intersection;;
					memset(&intersection, 0, sizeof(Intersection));
					intersection.center = (pos1+pos2)/2;
					intersection.error = std::max(0.01f, std::sqrt(errorSq));
					intersection.rays[i] = ray1;
					intersection.rays[j] = ray2;
					intersection.merge = NULL;
					intersections.push_back(intersection);
//					wxLogMessage("Intersection with error %f", std::sqrt(errorSq));
				}
			}
		}
	}
//	wxLogMessage("Got %d intersections!", (int)intersections.size());

	std::vector<Intersection> mergedIntersections;
	// Have to reserve to prevent reallocation, since it relies on pointers to merged intersections
	mergedIntersections.reserve(intersections.size()/3);

	// Merge possible intersections between three rays
	std::vector<Intersection*> mergers;
	std::vector<Intersection*> potentialMergers;
	for (int i = 0; i < intersections.size(); i++)
	{
		Intersection *ix = &intersections[i];
		// Check if it has been merged yet
		if (ix->merge != NULL)
		{
//			wxLogMessage("------ Skipping merged intersections %d on rays %d %d %d!", i, RAYNUM(ix->rays, 0), RAYNUM(ix->rays, 1), RAYNUM(ix->rays, 2));
			continue;
		}
		// Find the indices of the two intersecting rays
		int r1 = 0;
		while (ix->rays[r1] == NULL && r1 < groupCnt) r1++;
		int r2 = r1+1;
		while (ix->rays[r2] == NULL && r2 < groupCnt) r2++;
		// Search for other intersections with one common ray and one new ray
		for (int j = i+1; j < intersections.size(); j++)
		{
			Intersection *ixm = &intersections[j];
			if (ixm->rays[r1] == ix->rays[r1])
			{ // Intersection on same ray, either merge or create conflict
				if (ixm->rays[r2] == NULL)
				{ // Intersection is with a different ray group, check proximity to determine if merge of conflict
					float error = (ixm->center - ix->center).norm();
					if (error <= errorLimit)
					{ // Merge intersections, now consisting of three rays intersecting
						mergers.push_back(ixm);
						continue;
					} // else two intersections with different ray groups, but distant, so one must be wrong
					// Register conflict
					potentialMergers.push_back(ixm);
				} // else two intersections with same ray group and definitely a conflict
			}
			else if (ixm->rays[r2] == ix->rays[r2])
			{ // Intersection on same ray, either merge or create conflict
				if (ixm->rays[r1] == NULL)
				{ // Intersection is with a different ray group, check proximity to determine if merge of conflict
					float error = (ixm->center - ix->center).norm();
					if (error <= errorLimit)
					{ // Merge intersections, now consisting of three rays intersecting
						mergers.push_back(ixm);
						continue;
					} // else two intersections with different ray groups, but distant, so one must be wrong
					// Register conflict
					potentialMergers.push_back(ixm);
				} // else two intersections with same ray group and definitely a conflict
			}
		}

/*		wxLogMessage("------ Intersections %d on rays %d %d %d!", i, RAYNUM(ix->rays, 0), RAYNUM(ix->rays, 1), RAYNUM(ix->rays, 2));
		std::stringstream cfDbg;
		for (int j = 0; j < potentialMergers.size(); j++)
			cfDbg << IXNUM(potentialMergers[j]) << " - ";
		wxLogMessage("Potential mergers: %s", cfDbg.str());
*/
		// Check if merge candidates found (only for 3 rays+)
		if (mergers.size() > 0)
		{ // Merge and add new intersection
			mergers.push_back(ix);

			// In some cases, an intersection of 3 or more rays includes intersections out of error range of the others
			// They have to be manually added

// Begin OOR fix
			// Get all rays involved in this intersection (more than 2, else it wouldn't need to merge)
			Ray *ixRays[MAX_CAMERA_COUNT];
			for (int i = 0; i < mergers.size(); i++)
				for (int j = 0; j < groupCnt; j++)
					if (mergers[i]->rays[j] != NULL)
						ixRays[j] = mergers[i]->rays[j];

/*			std::stringstream rayDbg;
			for (int j = 0; j < groupCnt; j++)
				rayDbg << RAYNUM(ixRays, j) << " - ";
			wxLogMessage("Involved Rays: %s", rayDbg.str());
*/
			// Go through conflicts (other intersections on the two rays of our main intersection ix respectively)
			// And find those that intersect with any two rays involved in this merging intersection
			// Then add them to the merge and remove them as conflicts
			for (int i = 0; i < potentialMergers.size(); i++)
			{
				bool match = true;
				for (int j = 0; j < groupCnt; j++)
				{
					if (potentialMergers[i]->rays[j] != NULL && potentialMergers[i]->rays[j] != ixRays[j])
					{
						match = false;
						break;
					}
				}
				if (match)
				{ // Accept as merger, probably out of error range
					mergers.push_back(potentialMergers[i]);
//					wxLogMessage("Added intersection %d to merge because of shared rays!", IXNUM(potentialMergers[i]));
				}
			}
// Emd OOR fix

/*			std::stringstream mergeDbg;
			for (int j = 0; j < mergers.size(); j++)
				mergeDbg << IXNUM(mergers[j]) << " - ";
			wxLogMessage("Merging: %s", mergeDbg.str());
*/
			// Merge
			mergedIntersections.push_back({});
			Intersection *ixm = &mergedIntersections[mergedIntersections.size()-1];
			ixm->center = Eigen::Vector4f::Zero();
			ixm->error = 0.0f;
			for (int i = 0; i < mergers.size(); i++)
			{
				// Mark as merged
				mergers[i]->merge = ixm;
				// Update merge center
				ixm->center += mergers[i]->center;
				// Somehow update error
				ixm->error += mergers[i]->error;
			}
			// Average out values
			ixm->center = ixm->center / mergers.size();
			ixm->error = ixm->error / mergers.size();
			// Correct error by number of involved intersections
			ixm->error = ixm->error * 2 / mergers.size();
			// Make sure involved rays are accurate
			for (int j = 0; j < groupCnt; j++)
				ixm->rays[j] = ixRays[j];

			// Update intersection counters
			// Remove mergers
			for (int i = 0; i < mergers.size(); i++)
				for (int j = 0; j < groupCnt; j++)
					if (mergers[i]->rays[j] != NULL)
						mergers[i]->rays[j]->intersectionCount--;
			// Add merged intersection
			for (int i = 0; i < groupCnt; i++)
				ixRays[i]->intersectionCount++;

			// Reset for next iteration
			mergers.clear();
		}
		potentialMergers.clear();
	}

	// Accumulate conflict count over rays
	int conflictCount = 0;
	for (int i = 0; i < groupCnt; i++)
	{
		std::vector<Ray> *rays = rayGroups[i];
		for (int j = 0; j < rays->size(); j++)
		{
//			wxLogMessage("Group %d - Ray %d: %d intersections!", i, j, rays->at(j).intersectionCount);
			if (rays->at(j).intersectionCount > 1)
				rays->at(j).conflictID = conflictCount++;
			else
				rays->at(j).conflictID = -1;
		}
	}

	// Accumulate point counts over intersections
	int pointCount = 0, conflictedPointCount = 0;
	auto accumPointCounts = [&groupCnt, &pointCount, &conflictedPointCount](Intersection *ix) {
		for (int j = 0; j < groupCnt; j++)
		{
			if (ix->rays[j] != NULL && ix->rays[j]->conflictID != -1)
			{
				conflictedPointCount++;
				return;
			}
		}
		pointCount++;
	};
	for (int i = 0; i < intersections.size(); i++)
		if (intersections[i].merge == NULL)
			accumPointCounts(&intersections[i]);
	for (int i = 0; i < mergedIntersections.size(); i++)
		accumPointCounts(&mergedIntersections[i]);

	// Enter points and conflicts
	int ctPos = 0, cfPos = pointCount;
	points3D.resize(pointCount + conflictedPointCount);
	conflicts.resize(conflictCount);
	auto handlePoints = [&groupCnt, &points3D, &conflicts, &ctPos, &cfPos](Intersection *ix) {
		bool conflicted = false;
		float confidence = 0.0f;
		int rayCount = 0;
		for (int j = 0; j < groupCnt; j++)
		{
			if (ix->rays[j] != NULL)
			{
				confidence += 1.0f/ix->rays[j]->intersectionCount;
				rayCount++;
				if (ix->rays[j]->conflictID != -1)
				{ // Add to conflict group
					conflicts[ix->rays[j]->conflictID].push_back(cfPos);
					conflicted = true;
				}
			}
		}
		// Assign point to appropriate position
		TriangulatedPoint *pt;
		if (conflicted) pt = &points3D[cfPos++];
		else pt = &points3D[ctPos++];
		pt->pos = ix->center;
		pt->error = ix->error;
		pt->confidence = confidence / rayCount;
	};
	for (int i = 0; i < intersections.size(); i++)
	{
		if (intersections[i].merge == NULL)
			handlePoints(&intersections[i]);
	}
	for (int i = 0; i < mergedIntersections.size(); i++)
	{
		handlePoints(&mergedIntersections[i]);
	}

	// Debug
/*	int ixCnt = 0;
	for (int i = 0; i < intersections.size(); i++)
		if (intersections[i].merge == NULL) ixCnt++;
	wxLogMessage("Merge Status: %d single intersections, plus %d merged intersections!", ixCnt, (int)mergedIntersections.size());
	wxLogMessage("Conflict Status: %d certain points, plus %d uncertain in a total of %d conflicts!", pointCount, conflictedPointCount, conflictCount);
*/

	return pointCount;
}

typedef struct {
	float validity;
	std::vector<int> points;
	std::vector<int> pointMap;
	// To get triangulated points: points[points[i]]
	// To get actual points: marker[pointMap[points3D[i]]]
	Eigen::Isometry3f estTransform;
} MarkerCandidate3D;

static void kabsch(MarkerCandidate3D *candidate, const std::vector<TriangulatedPoint> &points3D)
{
	int ptCount = candidate->points.size();
	// Step 3.0: Calculate center of mass for both marker and triangulated point set
	Eigen::Vector3f trCenter = Eigen::Vector3f::Zero();
	Eigen::Vector3f mkCenter = Eigen::Vector3f::Zero();
	for (int j = 0; j < ptCount; j++)
	{
		int trPt = candidate->points[j];
		int mkPt = candidate->pointMap[trPt];
		trCenter += points3D[trPt].pos.head<3>();
		mkCenter += marker3D.markerTemplate.pts[mkPt].pos;
	}
	trCenter /= ptCount;
	mkCenter /= ptCount;
	// Step 3.1: Arrange data matrices and calculate covariance matrix
	Eigen::MatrixX3f trMat(ptCount, 3);
	Eigen::Matrix3Xf mkMat(3, ptCount);
	for (int j = 0; j < ptCount; j++)
	{
		int trPt = candidate->points[j];
		int mkPt = candidate->pointMap[trPt];
		trMat.row(j) = points3D[trPt].pos.head<3>() - trCenter;
		mkMat.col(j) = marker3D.markerTemplate.pts[mkPt].pos - mkCenter;
	}
	Eigen::Matrix3f covarianceMat = mkMat * trMat;
	// Step 3.2: Compute SVD
	Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::NoQRPreconditioner> svd(covarianceMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU().transpose();
	Eigen::Matrix3f V = svd.matrixV();
	// Step 3.3: Compute optimal rotation matrix and transformation
	float d = (V * U).determinant();
	Eigen::Matrix3f rot = V * Eigen::AlignedScaling3f(1, 1, d) * U;
	candidate->estTransform.linear() = rot;
	candidate->estTransform.translation() = (trCenter - rot * mkCenter).transpose();
}

/*
 * Detect Markers in the triangulated 3D Point cloud
 */
void detectMarkers3D(const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<Pose> &poses3D)
{
//	wxLogMessage("--------------");

	// Step 1: Get closest neighbouring points for each point
	float maxRelevantDistance = marker3D.relationDist[marker3D.relationDist.size()-1].distance + 0.1f;
	float maxRelevantDistanceSq = maxRelevantDistance*maxRelevantDistance;
	std::vector<std::vector<PointRelation>> trRelations;
	trRelations.resize(points3D.size());
	for (int i = 0; i < points3D.size()-1; i++)
	{
		const TriangulatedPoint *pt1 = &points3D[i];
		for (int j = i+1; j < points3D.size(); j++)
		{
			const TriangulatedPoint *pt2 = &points3D[j];
			Eigen::Vector4f dir = pt2->pos - pt1->pos;
			//if (dir.sum() > maxRelevantDistance) continue;
			float distSq = dir.squaredNorm();
			if (distSq < maxRelevantDistanceSq)
			{
				float dist = std::sqrt(distSq);
				dir = dir / dist; // Normalize direction
				trRelations[i].push_back({ i, j,  dir, dist });
				trRelations[j].push_back({ j, i, -dir, dist });
//				wxLogMessage("Relation (%d - %d)", i, j);
			}
		}
//		std::sort(trRelations[i].begin(), trRelations[i].end());
	}

	// Step 2: Find candidates
	// Step 2.1: Find 3 points in triangulated point cloud and check against matches in marker based on length and then angle
	// Step 2.2: Calculate initial pose using kabsch algorithm on 3 points
	// Step 2.3: Iterate over unchecked points in both clouds and include in candidate if match is found
	// Step 2.4: Calculate pose again using kabsch algorithm
	std::vector<MarkerCandidate3D> candidates;
	int bestCandidatePtCount = 0;
	// Helper state, true for points which have been assigned to a candidate with more than 4 points
	std::vector<bool> assigned;
	assigned.resize(points3D.size());
	// Subroutine checking the given triangulated point combination for a match in the marker, already given the possible base matches
	auto check3PointCandidate = [&points3D, &trRelations, &candidates, &assigned, &bestCandidatePtCount](
		PointRelation *trRelBase, PointRelation *trRelArm,
		int trPtJ, int trPtB, int trPtA,
		float baseError, float armError,
		std::pair<std::vector<PointRelation>::iterator, std::vector<PointRelation>::iterator> mkRelBaseRange)
	{
		for (auto mkRelBase = mkRelBaseRange.first; mkRelBase < mkRelBaseRange.second; mkRelBase++)
		{ // Iterate over all matching base relation candidates
			int mkPts[2] = { mkRelBase->pt1, mkRelBase->pt2 };
//			wxLogMessage("|  _ - Mk Base Relation (%d - %d) of length %f!", mkRelBase->pt1, mkRelBase->pt2, mkRelBase->distance);

			// Since we don't know the direction of the marker relation, both sides need to be checked
			for (int m = 0; m < 2; m++)
			{
				int mkPtJ = mkPts[m];
				int mkPtB = mkPts[1-m];

				// Now candidates for the arm
				for (int n = 0; n < marker3D.pointRelation[mkPtJ].size(); n++)
				{
					PointRelation *mkRelArm = &marker3D.relationDist[marker3D.pointRelation[mkPtJ][n]];
					int mkPtA = mkRelArm->pt1 == mkPtJ? mkRelArm->pt2 : mkRelArm->pt1;
					if (mkPtA == mkPtB) continue;

					if (std::abs(trRelArm->distance - mkRelArm->distance) < armError)
					{ // Found potential arm with same distance
//						wxLogMessage("|  |  _ - Mk Arm Relation (%d - %d) with length %f (error %f +- %f)!", mkRelArm->pt1, mkRelArm->pt2, mkRelArm->distance, std::abs(trRelArm->distance - mkRelArm->distance), armError);

						// Make sure they have the same angle
						int mkDirFlip = mkRelBase->pt1 == mkRelArm->pt1 || mkRelBase->pt2 == mkRelArm->pt2? 1 : -1;
						float mkAngle = std::acos(mkDirFlip * mkRelBase->dir.dot(mkRelArm->dir));
						int trDirFlip = trRelBase->pt1 == trRelArm->pt1 || trRelBase->pt2 == trRelArm->pt2? 1 : -1;
						float trAngle = std::acos(trDirFlip * trRelBase->dir.dot(trRelArm->dir));
						float angleErrorLimit = std::asin(baseError / mkRelBase->distance) + std::asin(armError / mkRelArm->distance);
						if (std::abs(trAngle-mkAngle) > angleErrorLimit)
						{
//							wxLogMessage("|  |  |    - Expected angle %f but is %f!", mkAngle/PI*180.0f, trAngle/PI*180.0f);
							continue;
						}
						// Register as marker candidate - candidate pose could now be extracted
						candidates.push_back({});
						MarkerCandidate3D *candidate = &candidates[candidates.size()-1];
						candidate->validity = 1.0f;
						candidate->pointMap.resize(points3D.size());
						memset(candidate->pointMap.data(), -1, points3D.size()*sizeof(int));
						// Add initial three points
						candidate->points.push_back(trPtA);
						candidate->pointMap[trPtA] = mkPtA;
						candidate->points.push_back(trPtJ);
						candidate->pointMap[trPtJ] = mkPtJ;
						candidate->points.push_back(trPtB);
						candidate->pointMap[trPtB] = mkPtB;
						// Debug
//						wxLogMessage("|  |  |  _ - Candidate %d: (%d->%d), (%d->%d), (%d->%d)", (int)candidates.size()-1, trPtA, mkPtA, trPtJ, mkPtJ, trPtB, mkPtB);
/*						wxLogMessage("|  |  |  _ - Candidate %d: (%d->%d), (%d->%d), (%d->%d); Angle %f %+f (+-%f); 1-2 %f %+f (+-%f); 2-3 %f %+f (+-%f)", (int)candidates.size()-1,
							trPtA, mkPtA, trPtJ, mkPtJ, trPtB, mkPtB,
							mkAngle/PI*180.0f, (trAngle-mkAngle)/PI*180.0f, angleErrorLimit/PI*180.0f,
							mkRelBase->distance, (trRelBase->distance-mkRelBase->distance), baseError,
							mkRelArm->distance, (trRelArm->distance-mkRelArm->distance), armError);
*/
						// ----- Add more points -----

						// Calculate estimated pose of the three points
						kabsch(candidate, points3D);

						// Prepare assigned for marker points, used to avoid duplicate checks
						std::vector<bool> mkPtHandled;
						mkPtHandled.resize(marker3D.markerTemplate.pts.size());
						for (int o = 0; o < candidate->points.size(); o++)
							mkPtHandled[candidate->pointMap[candidate->points[o]]] = true;

						// Find extends of the candidate
						float poseError = 0;
						for (int o = 0; o < candidate->points.size(); o++)
						{
							int trPt = candidate->points[o];
							float distFromCoM = (points3D[trPt].pos.head<3>() - candidate->estTransform.translation()).norm();
							poseError += points3D[trPt].error / distFromCoM;
						}
						poseError = poseError / candidate->points.size();

						// Brute force check all points
						Eigen::Isometry3f invTransform = candidate->estTransform.inverse();
						for (int j = 0; j < points3D.size(); j++)
						{
							if (assigned[j]) continue;
							if (candidate->pointMap[j] >= 0) continue;
							Eigen::Vector3f estMkPos = invTransform * points3D[j].pos.head<3>();
							float ptError = SIGMA_ERROR * points3D[j].error;
							float distFromCoM = (points3D[j].pos.head<3>() - candidate->estTransform.translation()).norm();
							float errorLimit = ptError + poseError * distFromCoM;
							for (int k = 0; k < marker3D.markerTemplate.pts.size(); k++)
							{
								if (mkPtHandled[k]) continue;
								float errorSq = (estMkPos - marker3D.markerTemplate.pts[k].pos).squaredNorm();
								if (errorSq < errorLimit*errorLimit)
								{
//									wxLogMessage("|  |  |  |    - Added (%d - %d) - dist %f, max %f!", j, k, std::sqrt(errorSq), errorLimit);
									candidate->points.push_back(j);
									candidate->pointMap[j] = k;
									mkPtHandled[k] = true;
									break;
								}
								else if (errorSq < errorLimit*errorLimit*2)
								{
//									wxLogMessage("|  |  |  |    - Rejected (%d - %d) - dist %f, max %f!", j, k, std::sqrt(errorSq), errorLimit);
								}
							}
						}

						if (candidate->points.size() > 4)
						{ // Candidate is almost certain, mark points as assigned
							for (int o = 0; o < candidate->points.size(); o++)
								assigned[candidate->points[o]] = true;
						}
						if (candidate->points.size() > bestCandidatePtCount)
							bestCandidatePtCount = candidate->points.size();

						// Calculate final candidate pose
						kabsch(candidate, points3D);

						// One candidate for these three points is enough
						return;
					}
					else if (std::abs(trRelArm->distance - mkRelArm->distance) < armError*2)
					{ // Found potential arm with same distance
//						wxLogMessage("|  |  _ - Dropped Mk Arm Relation (%d - %d) with length %f (error %f +- %f)!", mkRelArm->pt1, mkRelArm->pt2, mkRelArm->distance, std::abs(trRelArm->distance - mkRelArm->distance), armError);
					}
				}
			}
		}
	};
	auto haveSharedArm = [&trRelations](int ptA, int ptB)
	{
		bool sharedArm = false;
		for (int l = 0; l < trRelations[ptA].size(); l++)
			if (sharedArm = (trRelations[ptA][l].pt2 == ptB))
				break;
		return sharedArm;
	};
	// Find 3 point combinations to check
	for (int i = 0; i < points3D.size(); i++)
	{
		if (assigned[i]) continue;

		// Point i is our current triangulated joint point
		for (int j = 0; j < trRelations[i].size(); j++)
		{
			PointRelation *trRelBase = &trRelations[i][j];
			if (trRelBase->pt2 < trRelBase->pt1) continue;
			if (assigned[trRelBase->pt2]) continue;
			// trRelBase is a relation in the triangulated point cloud which has not been used as a base before

			// Find potential matches for the base using the marker lookup table with distance within the error range
			float baseError = SIGMA_ERROR * (points3D[trRelBase->pt1].error + points3D[trRelBase->pt2].error);
			auto mkRelBaseRange = std::equal_range(marker3D.relationDist.begin(), marker3D.relationDist.end(), trRelBase->distance, ErrorRangeComp(baseError));
//			wxLogMessage("Tr Base Relation (%d - %d) of length %f += %f: %d potential matches in marker!", trRelBase->pt1, trRelBase->pt2, trRelBase->distance, baseError, std::distance(mkRelBaseRange.first, mkRelBaseRange.second));
			if (mkRelBaseRange.first >= mkRelBaseRange.second) continue;
			// Found some possible candidates within error range

			// Check for arms of major base point
			for (int k = 0; k < trRelations[i].size(); k++)
			{
				PointRelation *trRelArm = &trRelations[i][k];
				if (assigned[trRelArm->pt2]) continue;
				// Make sure no duplicates are checked
				if (trRelArm->pt2 < trRelBase->pt1)
				{ // This relation has already served as a base before
					if (haveSharedArm(trRelBase->pt2, trRelArm->pt2)) continue;
				}
				else if (trRelArm->pt2 <= trRelBase->pt2) continue;

				// Now we have a set of three points around joint point trRelBase->pt1 which have not been handled before
				assert(trRelBase->pt1 == trRelArm->pt1);

				float armError = SIGMA_ERROR * (points3D[trRelArm->pt1].error + points3D[trRelArm->pt2].error);
//				wxLogMessage("_ - Tr Arm Relation (%d - %d) with length %f += %f!", trRelArm->pt1, trRelArm->pt2, trRelArm->distance, armError);

				// Check 3 points if they are a valid candidate
				// and if so, extend to include matching neighboring points
				// if 5+ points have been found, it is an almost certain match and points are assigned to not be checked again
				check3PointCandidate(
					trRelBase, trRelArm,
					trRelBase->pt1, trRelBase->pt2, trRelArm->pt2,
					baseError, armError,
					mkRelBaseRange);

				if (assigned[i]) break; // if true an almost certain candidate with 5+ points has been found
			}
			if (assigned[i]) break; // if true an almost certain candidate with 5+ points has been found

			// Check for arms of minor base point
			for (int k = 0; k < trRelations[trRelBase->pt2].size(); k++)
			{
				PointRelation *trRelArm = &trRelations[trRelBase->pt2][k];
				if (assigned[trRelArm->pt2]) continue;
				// Make sure no duplicates are checked
				if (trRelArm->pt2 <= trRelBase->pt1) continue;
				if (trRelArm->pt2 >= trRelBase->pt2) continue;
				if (haveSharedArm(trRelBase->pt1, trRelArm->pt2)) continue;

				// Now we have a set of three points around joint point trRelBase->pt2 which have not been handled before
				assert(trRelBase->pt2 == trRelArm->pt1);

				float armError = SIGMA_ERROR * (points3D[trRelArm->pt1].error + points3D[trRelArm->pt2].error);
//				wxLogMessage("_ - Tr Arm Relation (%d - %d) with length %f += %f!", trRelArm->pt1, trRelArm->pt2, trRelArm->distance, armError);

				// Check 3 points if they are a valid candidate
				// and if so, extend to include matching neighboring points
				// if 5+ points have been found, it is an almost certain match and points are assigned to not be checked again
				check3PointCandidate(
					trRelBase, trRelArm,
					trRelBase->pt2, trRelBase->pt1, trRelArm->pt2,
					baseError, armError,
					mkRelBaseRange);

				if (assigned[i]) break; // if true an almost certain candidate with 5+ points has been found
			}
		}
	}
//	wxLogMessage("------------- %d initial candidates -----", (int)candidates.size());

	// Step 3: Find the best candidate for this marker, first by amount of points and then by internal MSE
	float lowestMSE = 99999.0f;
	int lowestMSEInd = -1;
	for (int i = 0; i < candidates.size(); i++)
	{
		MarkerCandidate3D *candidate = &candidates[i];
		if (candidate->points.size() == bestCandidatePtCount)
		{
			// Calculate mean square error
			float meanSquaredError = 0.0f;
			for (int j = 0; j < candidate->points.size(); j++)
			{
				int trPt = candidate->points[j];
				int mkPt = candidate->pointMap[trPt];
				Eigen::Vector3f trPosGT = points3D[trPt].pos.head<3>();
				Eigen::Vector3f mkPos = marker3D.markerTemplate.pts[mkPt].pos;
				Eigen::Vector3f trPosRC = candidate->estTransform * mkPos;
				meanSquaredError += (trPosRC - trPosGT).squaredNorm();
			}
			meanSquaredError /= candidate->points.size();

			if (lowestMSE > meanSquaredError)
			{ // New lowest MSE
				lowestMSE = meanSquaredError;
				lowestMSEInd = i;
			}
		}
	}
	if (lowestMSEInd >= 0)
	{ // Got a candidate
		MarkerCandidate3D *candidate = &candidates[lowestMSEInd];
		poses3D.push_back({ candidate->estTransform.translation(), candidate->estTransform.rotation() });
		wxLogMessage("Best candidate %d of %d total with %d points has internal MSE of %f", lowestMSEInd, (int)candidates.size(), (int)candidate->points.size(), lowestMSE);
	}
	else
	{ // No candidate at all
		wxLogMessage("No candidate has been found! %d points, of those %d not conflicted", (int)points3D.size(), nonconflictedCount);
		for (int i = 0; i < points3D.size()-1; i++)
		{
			float min = 0, max = maxRelevantDistance;
			if (trRelations[i].size() > 0)
			{
				min = trRelations[i][0].distance;
				max = trRelations[i][trRelations[i].size()-1].distance;
			}
			wxLogMessage("--> Point %d got a total of %d points from %f to %f cm", i, (int)trRelations[i].size(), min, max);
		}
	}

//	wxLogMessage("---------");
}

/*
 * Visualize single-camera poses
 */
void visualizePoses(const Transform &camera, const std::vector<Point> &points2D, const std::vector<Marker> &markers2D, const std::vector<Pose> &poses3D)
{
	struct Line { float startX; float startY; float endX; float endY; };

	// Render all blobs
	glColor3f(1,0,0);
	for (int i = 0; i < points2D.size(); i++)
	{
		//GLfloat s = points2D[i].S*2.0f;
		//glPointSize(s <= 1? 1.0f : s);
		glPointSize(6.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(points2D[i].X/width*2-1, points2D[i].Y/height*2-1, 0.9f);
		glEnd();
	}

	// Render inferred poses
	glColor3f(0, 0, 1);
	glLineWidth(2.0f);
	for (int i = 0; i < poses3D.size(); i++)
	{
		const Pose *pose = &poses3D[i];
		glLoadMatrixf((float*)createMVP(camera, pose->trans, pose->rot, 10.0f).data());
		vizCoordCross->draw();
	}
	glLoadIdentity();

	// Render marker lines
	std::vector<struct Line> markerLines;
	for (int m = 0; m < markers2D.size(); m++)
	{
		const Marker *marker = &markers2D[m];
		// Display base
		markerLines.push_back({
			(marker->endA->X / width - 0.5f) * 2.0f,
			(marker->endA->Y / height - 0.5f) * 2.0f,
			(marker->endB->X / width - 0.5f) * 2.0f,
			(marker->endB->Y / height - 0.5f) * 2.0f
		});
		// Display heading
		markerLines.push_back({
			(marker->center->X / width - 0.5f) * 2.0f,
			(marker->center->Y / height - 0.5f) * 2.0f,
			(marker->header->X / width - 0.5f) * 2.0f,
			(marker->header->Y / height - 0.5f) * 2.0f
		});
	}
	glLineWidth(2.0f);
	glColor3f(1, 1, 0);
	glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line) * markerLines.size(), &markerLines[0], GL_STREAM_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line)/2, (void *)0);
	glEnableVertexAttribArray(0);
	glDrawArrays(GL_LINES, 0, (int)markerLines.size()*2);
}

/*
 * Visualize multi-camera triangulated markers
 */
void visualizeMarkers(const Transform &camera, const std::vector<Point> &points2D, const std::vector<Ray> &rays3D, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount, const std::vector<Eigen::Vector4f> &triangulatedPoints3D, const std::vector<Pose> &poses3D)
{
	struct Line { float startX; float startY; float endX; float endY; };
	Eigen::Matrix4f vp = createProjectionMatrix() * createViewMatrix(camera);

	// Render ray lines
	std::vector<struct Line> rayLines;
	for (int r = 0; r < rays3D.size(); r++)
	{
		const Ray *ray = &rays3D[r];
		const int segCnt = 5;
		for (int i = 0; i < segCnt; i ++)
		{ // Display ray in segments to prevent weird clipping
			Eigen::Vector4f start = vp * (ray->pos + ray->dir * i * 1000/segCnt);
			Eigen::Vector4f end = vp * (ray->pos + ray->dir * (i+1) * 1000/segCnt);
			// Register ray segment
			rayLines.push_back({
				start.x()/start.w(),
				start.y()/start.w(),
				end.x()/end.w(),
				end.y()/end.w()
			});
		}
	}
	glLineWidth(0.5f);
	glColor3f(0, 1, 0);
	glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line) * rayLines.size(), &rayLines[0], GL_STREAM_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line)/2, (void *)0);
	glEnableVertexAttribArray(0);
	glDrawArrays(GL_LINES, 0, (int)rayLines.size()*2);

	// Render all blobs
/*	glColor3f(1,0,0);
	for (int i = 0; i < points2D.size(); i++)
	{
		//GLfloat s = points2D[i].S*2.0f;
		//glPointSize(s <= 1? 1.0f : s);
		glPointSize(6.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(points2D[i].X/width*2-1, points2D[i].Y/height*2-1, 0.9f);
		glEnd();
	}*/

	// Testing: Render points which could have been triangulated
	glColor3f(1,0,0);
	for (int i = 0; i < triangulatedPoints3D.size(); i++)
	{
		Eigen::Vector4f pt = vp * triangulatedPoints3D[i];
		glPointSize(6.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x()/pt.w(), pt.y()/pt.w(), pt.z()/pt.w());
		glEnd();
	}

	// Render all triangulated points
	glColor3f(0,1,1);
	for (int i = 0; i < nonconflictedCount; i++)
	{
		Eigen::Vector4f pt = vp * points3D[i].pos;
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x()/pt.w(), pt.y()/pt.w(), pt.z()/pt.w());
		glEnd();
//		wxLogMessage("Point %d confidence: %f", i, points[i].confidence);
	}
	glColor3f(1,1,0);
	for (int i = nonconflictedCount; i < points3D.size(); i++)
	{
		Eigen::Vector4f pt = vp * points3D[i].pos;
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x()/pt.w(), pt.y()/pt.w(), pt.z()/pt.w());
		glEnd();
//		wxLogMessage("Conflicted Point %d confidence: %f", i, conflictedPoints[i].confidence);
	}

	// Render inferred poses
	glColor3f(0, 0, 1);
	glLineWidth(2.0f);
	for (int i = 0; i < poses3D.size(); i++)
	{
		const Pose *pose = &poses3D[i];
		glLoadMatrixf((float*)createMVP(camera, pose->trans, pose->rot, 10.0f).data());
		vizCoordCross->draw();
	}
	glLoadIdentity();
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
	marker3D.markerTemplate = markerData;
#ifdef USE_CV
	cv_marker3DTemplate.clear();
	for (int i = 0; i < marker3D.markerTemplate.pts.size(); i++)
	{
		DefMarkerPoint *pt = &marker3D.markerTemplate.pts[i];
		cv_marker3DTemplate.push_back(cv::Point3f(pt->pos.x(), pt->pos.y(), pt->pos.z()));
	}
#endif

	// Setup lookup tables for quick marker identification
	marker3D.relationDist.clear();
	marker3D.pointRelation.clear();
	marker3D.pointRelation.resize(marker3D.markerTemplate.pts.size());

	// Step 0: Determine upper bound for total number of relations (binominal coefficient)
	// This estimate (n^k / k!) is excellent for small k (here k=2)
	int relUpperBound = marker3D.markerTemplate.pts.size() * marker3D.markerTemplate.pts.size() / 2;

	// Step 1: Create all relations
	std::vector<PointRelation> relations;
	relations.reserve(relUpperBound);
	for (int i = 0; i < marker3D.markerTemplate.pts.size()-1; i++)
	{
		DefMarkerPoint *pt1 = &marker3D.markerTemplate.pts[i];
		for (int j = i+1; j < marker3D.markerTemplate.pts.size(); j++)
		{
			DefMarkerPoint *pt2 = &marker3D.markerTemplate.pts[j];
			Eigen::Vector3f dir = pt2->pos - pt1->pos;
			float distance = dir.norm();
			relations.push_back({ i, j, Eigen::Vector4f(dir.x(), dir.y(), dir.z(), 0) / distance, distance });
		}
	}

	// Step 2: Sort based on distance
	std::sort(relations.begin(), relations.end());

	// Step 3: Enter relevant relations
	int *pointRelationCount = new int[marker3D.markerTemplate.pts.size()];
	memset(pointRelationCount, 0, marker3D.markerTemplate.pts.size()*sizeof(int));
	for (int i = 0; i < relations.size(); i++)
	{
		PointRelation *rel = &relations[i];
		if (pointRelationCount[rel->pt1] < NUM_CLOSEST_RELATIONS || pointRelationCount[rel->pt2] < NUM_CLOSEST_RELATIONS || rel->distance < MIN_DISTANCE_RELATIONS)
		{ // Each point should have at least NUM_CLOSEST_RELATIONS stored relations
			pointRelationCount[rel->pt1]++;
			pointRelationCount[rel->pt2]++;
			int relIndex = marker3D.relationDist.size();
			marker3D.relationDist.push_back(*rel);
			marker3D.pointRelation[rel->pt1].push_back(relIndex);
			marker3D.pointRelation[rel->pt2].push_back(relIndex);
		}
	}

	// markerDistLookup is now a list of relations sorted by distance with at least NUM_CLOSEST_RELATIONS of the closest relations per point
}

/*
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount()
{
	return marker3D.markerTemplate.pts.size() < 4? 4 : marker3D.markerTemplate.pts.size();
}

/*
 * Projects marker into image plane provided translation in centimeters and rotation, both relative to camera
 */
void createMarkerProjection(std::vector<Point> &points2D, std::bitset<MAX_MARKER_POINTS> &mask, const Transform &camera, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float ptScale, float stdDeviation)
{
	// Create MVP in camera space, translation in centimeters
	Eigen::Matrix4f mv = createViewMatrix(camera) * createModelMatrix(translation, rotation, 1.0f);
	Eigen::Matrix4f mvp = createProjectionMatrix() * mv;
	// Create random noise generator
	std::mt19937 gen(std::random_device{}());
    std::normal_distribution<float> noise(0, stdDeviation);
	// Project each marker point into image space
	points2D.clear();
	for (int i = 0; i < marker3D.markerTemplate.pts.size(); i++)
	{
		mask[i] = false;
		DefMarkerPoint *markerPt = &marker3D.markerTemplate.pts[i];
		Eigen::Vector4f ptPos(markerPt->pos.x(), markerPt->pos.y(), markerPt->pos.z(), 1.0f);
		Eigen::Vector4f ptNrm(markerPt->nrm.x(), markerPt->nrm.y(), markerPt->nrm.z(), 0.0f);
		// Calculate and clip marker points not facing the camera in regards to their field of view
		ptNrm = (mv * ptNrm).normalized();
		Eigen::Vector4f viewNrm = (mv * ptPos).normalized();
		float facing = -ptNrm.dot(viewNrm);
		float limit = std::cos(markerPt->fov/360*PI);
		if (facing < limit) continue;
		// Project point and clip
		ptPos = mvp * ptPos;
		ptPos = ptPos / ptPos.w();
		if (std::abs(ptPos.x()) > 1 || std::abs(ptPos.y()) > 1) continue;
		// Generate noise
		const int maxNoise = 3*stdDeviation;
		float noiseX = noise(gen), noiseY = noise(gen);
		if (std::abs(noiseX) > maxNoise) noiseX /= std::ceil(std::abs(noiseX)/maxNoise);
		if (std::abs(noiseY) > maxNoise) noiseY /= std::ceil(std::abs(noiseY)/maxNoise);
		// Register projected marker point
		points2D.push_back({
			(ptPos.x() + 1)/2 * width + noiseX,
			(ptPos.y() + 1)/2 * height + noiseY,
			ptScale * 100/translation.z()
		});
		mask[i] = true;
	}
}

/*
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector4f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation)
{
	Eigen::Matrix4f m = createModelMatrix(translation, rotation, 1.0f);
	points3D.clear();
	for (int i = 0; i < marker3D.markerTemplate.pts.size(); i++)
	{
		if (!mask[i]) continue;
		DefMarkerPoint *markerPt = &marker3D.markerTemplate.pts[i];
		Eigen::Vector4f ptPos(markerPt->pos.x(), markerPt->pos.y(), markerPt->pos.z(), 1.0f);
		ptPos = m * ptPos;
		ptPos = ptPos / ptPos.w();
		points3D.push_back(ptPos);
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