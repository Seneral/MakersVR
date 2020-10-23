/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "testing.hpp"

#include "mesh.hpp"
#include "wxbase.hpp" // wxLog*

#include <random>


/* Structures */

struct Line2D
{
	float startX;
	float startY;
	float endX;
	float endY;
};


/* Variables */

Mesh *vizCoordCross;
GLuint vizLinesVBO; // Line buffer for uploading visualization lines to GPU
std::mt19937 gen = std::mt19937(std::random_device{}());

/* Functions */

/**
 * Initialize resources for visualization
 */
void initVisualization()
{
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
}

/**
 * Cleanup of resources
 */
void cleanVisualization()
{
	if (vizCoordCross != nullptr)
		delete vizCoordCross;
	if (vizLinesVBO != 0)
		glDeleteBuffers(1, &vizLinesVBO);
}

/**
 * Projects marker into image plane provided translation in centimeters and rotation, both relative to camera
 */
void createMarkerProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<float> &pointSizes, std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker, const Camera &camera, const Eigen::Isometry3f &transform, float stdDeviation)
{
	// Create MVP in camera space, translation in centimeters
	Eigen::Isometry3f mv = createViewMatrix(camera.transform) * transform;
	Eigen::Projective3f mvp = createProjectionMatrix(camera.fovH, camera.fovV) * mv;
	// Create random noise generator
	std::normal_distribution<float> noise(0, stdDeviation);
	const float maxNoise = 3*stdDeviation;
	// Project each marker point into image space
	points2D.reserve(points2D.size() + marker.points.size());
	pointSizes.reserve(pointSizes.size() + marker.points.size());
	for (int i = 0; i < marker.points.size(); i++)
	{
		mask[i] = false;
		const DefMarkerPoint *markerPt = &marker.points[i];
		// Calculate and clip marker points not facing the camera in regards to their field of view
		Eigen::Vector3f ptNrm = (mv.linear() * markerPt->nrm).normalized();
		Eigen::Vector3f viewNrm = (mv * markerPt->pos).normalized();
		float facing = -ptNrm.dot(viewNrm);
		float limit = std::cos(markerPt->fov/360*PI);
		if (facing < limit) continue;
		// Project point
		Eigen::Vector3f proj = projectPoint(mvp, markerPt->pos);
		// Cull back
		if (proj.z() < 0)
			continue;
		// Convert to pixel space
		Eigen::Vector2f ptPos((proj.x()+1)/2*camera.width, (proj.y()+1)/2*camera.height);
		// Apply distortion
		ptPos = distortPoint(camera, ptPos);
		// Generate noise
		float noiseX = noise(gen), noiseY = noise(gen);
		if (std::abs(noiseX) > maxNoise) noiseX /= std::ceil(std::abs(noiseX)/maxNoise);
		if (std::abs(noiseY) > maxNoise) noiseY /= std::ceil(std::abs(noiseY)/maxNoise);
		ptPos += Eigen::Vector2f(noiseX, noiseY);
		// Clip
		if (ptPos.x() < 0 || ptPos.y() < 0 || ptPos.x() > camera.width || ptPos.y() > camera.height)
			continue;
		// Register projected marker point converted to pixel space
		points2D.push_back(ptPos);
		pointSizes.push_back(1.0f + 0.1f/proj.z());
		mask[i] = true;
	}
}

/**
 * Transforms marker points based on translation and rotation
 */
void transformMarkerPoints(std::vector<Eigen::Vector3f> &points3D, const std::bitset<MAX_MARKER_POINTS> &mask, const DefMarker &marker3D, const Eigen::Isometry3f &transform)
{
	points3D.reserve(points3D.size() + marker3D.points.size());
	for (int i = 0; i < marker3D.points.size(); i++)
	{
		if (!mask[i]) continue;
		points3D.push_back(transform * marker3D.points[i].pos);
	}
}

/**
 * Analyze possibility of tracking algorithm to extract the pose based on ground truth data
 */
void analyzeTrackingAlgorithm(const std::vector<int> &visibleCount, const std::bitset<MAX_MARKER_POINTS> &triangulationMask, const std::vector<TriangulatedPoint> &points3D, const MarkerTemplate3D &marker3D, Eigen::Isometry3f gt)
{
	// Analyze with knowledge which triangulated points could be used for the marker detection
	std::vector<int> relationMask;
	relationMask.resize(marker3D.points.size());
	std::vector<std::pair<int, float>> mk2trMap;
	mk2trMap.resize(marker3D.points.size());
	std::vector<const PointRelation*> mkRelations;
	
	for (int i = 0; i < visibleCount.size(); i++)
	{
		int mkPtA = i;

		Eigen::Vector3f trPos = gt * marker3D.points[i];
		float trError = 9999.0f;
		int trPt = -1;
		for (int j = 0; j < points3D.size(); j++)
		{
			float error = (points3D[j].pos.head<3>() - trPos).norm();
			if (error < trError)
			{
				trError = error;
				trPt = j;
			}
		}

		if (triangulationMask.test(mkPtA))
		{
			mk2trMap[i].first = trPt;
			mk2trMap[i].second = trError;
			if (trPt == -1)
			{
				wxLogMessage("!!!!!! Could not find matching marker point %d, although it should have been triangulated!", i);
				continue;
			}
			else
				wxLogMessage("Found matching Marker point %d => %d within %f cm!", i, trPt, trError);

			for (int j = 0; j < marker3D.pointRelation[mkPtA].size(); j++)
			{
				int relInd = marker3D.pointRelation[mkPtA][j];
				const PointRelation *rel = &marker3D.relationDist[relInd];
				int mkPtB = rel->pt1 == mkPtA? rel->pt2 : rel->pt1;
				if (mkPtB > mkPtA && triangulationMask.test(mkPtB))
				{ // rel could have been inferred in marker, as both points ARE in the triangulated Point Cloud
					float minError = 3 * (0.01f + 0.01f);
					auto mkRelRange = std::equal_range(marker3D.relationDist.begin(), marker3D.relationDist.end(), rel->distance, ErrorRangeComp(minError));
					wxLogMessage("--> Relation (%d - %d) is in triangulated point cloud with minimum of %d candidates!", mkPtA, mkPtB, (int)std::distance(mkRelRange.first, mkRelRange.second));
					mkRelations.push_back(rel);
					if (relationMask[mkPtA] != 0 || relationMask[mkPtB] != 0)
					{ // Found a triple of triangulated points, so a candidate SHOULD have been found
						//wxLogMessage("!!!!!! ----> Triple Detected!");
					}
					relationMask[mkPtA]++;
					relationMask[mkPtB]++;
				}
			}
		}
		else if (trError < 0.1f)
		{ // Even though it should NOT be triangulated, there is a fake point in it
			mk2trMap[i].first = trPt;
			mk2trMap[i].second = trError;
			wxLogMessage("!!!!!! Marker point %d has a fake triangulated point %d within %f cm!", i, trPt, trError);
		}
	}

	for (int i = 0; i < mkRelations.size(); i++)
	{
		if (relationMask[mkRelations[i]->pt1] > 1)
		{
			int mkPtJ = mkRelations[i]->pt1;
			int mkPtB = mkRelations[i]->pt2;
			for (int j = 0; j < mkRelations.size(); j++)
			{
				if (mkRelations[j]->pt1 == mkPtJ || mkRelations[j]->pt2 == mkPtJ)
				{
					int mkPtA = mkRelations[j]->pt1 == mkPtJ? mkRelations[j]->pt2 : mkRelations[j]->pt1;
					if (mkPtA == mkPtB) continue;
					bool sharedArm = false;
					for (int l = 0; l < marker3D.pointRelation[mkPtA].size(); l++)
						if (sharedArm = (marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt1 == mkPtB || marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt2 == mkPtB))
							break;

					/*wxLogMessage("Found triple (%d - %d / %f - %f), (%d - %d / %f - %f), (%d - %d / %f - %f)! B-A have shared arm: %s!",
						mkPtB, mk2trMap[mkPtB].first, mk2trMap[mkPtB].second, 3*points3D[mk2trMap[mkPtB].first].error,
						mkPtJ, mk2trMap[mkPtJ].first, mk2trMap[mkPtJ].second, 3*points3D[mk2trMap[mkPtJ].first].error,
						mkPtA, mk2trMap[mkPtA].first, mk2trMap[mkPtA].second, 3*points3D[mk2trMap[mkPtA].first].error,
						sharedArm? "True" : "False");*/

				}
			}
		}
		else if (relationMask[mkRelations[i]->pt2] > 1)
		{
			int mkPtJ = mkRelations[i]->pt2;
			int mkPtB = mkRelations[i]->pt1;
			for (int j = 0; j < mkRelations.size(); j++)
			{
				if (mkRelations[j]->pt1 == mkPtJ || mkRelations[j]->pt2 == mkPtJ)
				{
					int mkPtA = mkRelations[j]->pt1 == mkPtJ? mkRelations[j]->pt2 : mkRelations[j]->pt1;
					if (mkPtA == mkPtB) continue;
					bool sharedArm = false;
					for (int l = 0; l < marker3D.pointRelation[mkPtA].size(); l++)
						if (sharedArm = (marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt1 == mkPtB || marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt2 == mkPtB))
							break;

					/*wxLogMessage("Found triple (%d - %d / %f - %f), (%d - %d / %f - %f), (%d - %d / %f - %f)! B-A have shared arm: %s!",
						mkPtB, mk2trMap[mkPtB].first, mk2trMap[mkPtB].second, 3*points3D[mk2trMap[mkPtB].first].error,
						mkPtJ, mk2trMap[mkPtJ].first, mk2trMap[mkPtJ].second, 3*points3D[mk2trMap[mkPtJ].first].error,
						mkPtA, mk2trMap[mkPtA].first, mk2trMap[mkPtA].second, 3*points3D[mk2trMap[mkPtA].first].error,
						sharedArm? "True" : "False");*/
				}
			}
		}

	}
}

/**
 * Visualize 2D points in pixel space
 */
void visualizePoints2D(const Camera &camera, const std::vector<Eigen::Vector2f> &points2D, Color color, float size, float depth, bool undistort)
{
	glColor3f(color.r, color.g, color.b);
	glPointSize(size);
	glBegin(GL_POINTS);
	for (int i = 0; i < points2D.size(); i++)
		glVertex3f(points2D[i].x()/camera.width*2-1, points2D[i].y()/camera.height*2-1, depth);
	glEnd();
}

/**
 * Visualize 3D points in world space
 */
void visualizePoints3D(const Camera &camera, const std::vector<Eigen::Vector3f> &points3D, Color color, float size, float depth)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);
	glColor3f(color.r, color.g, color.b);
	glPointSize(size);
	glBegin(GL_POINTS);
	for (int i = 0; i < points3D.size(); i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i]);
		glVertex3f(pt.x(), pt.y(), depth == 0? pt.z() : depth);
	}
	glEnd();
}

/**
 * Visualize poses in camera or world space
 */
void visualizePoses(const Camera &camera, const std::vector<Eigen::Isometry3f> &poses3D, bool cameraSpace)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV);
	if (!cameraSpace) vp = vp * createViewMatrix(camera.transform);

	// Render poses
	glColor3f(0, 0, 1);
	glLineWidth(2.0f);
	for (int i = 0; i < poses3D.size(); i++)
	{
		glLoadMatrixf((vp * poses3D[i] * Eigen::Scaling(10.0f)).data());
		vizCoordCross->draw();
	}
	glLoadIdentity();
}

/**
 * Visualize calibration markers in pixel space
 */
void visualizeMarkers(const Camera &camera, const std::vector<Marker2D> &markers2D)
{
	// Render marker lines
	std::vector<struct Line2D> markerLines;
	for (int m = 0; m < markers2D.size(); m++)
	{
		const Marker2D *marker = &markers2D[m];
		// Display base
		markerLines.push_back({
			marker->points[1].x() / camera.width * 2 - 1,
			marker->points[1].y() / camera.height * 2 - 1,
			marker->points[2].x() / camera.width * 2 - 1,
			marker->points[2].y() / camera.height * 2 - 1
		});
		// Display spine
		markerLines.push_back({
			marker->points[0].x() / camera.width * 2 - 1,
			marker->points[0].y() / camera.height * 2 - 1,
			marker->points[3].x() / camera.width * 2 - 1,
			marker->points[3].y() / camera.height * 2 - 1
		});
		// Display head left
		markerLines.push_back({
			marker->points[3].x() / camera.width * 2 - 1,
			marker->points[3].y() / camera.height * 2 - 1,
			marker->points[4].x() / camera.width * 2 - 1,
			marker->points[4].y() / camera.height * 2 - 1
		});
		// Display head right
		markerLines.push_back({
			marker->points[3].x() / camera.width * 2 - 1,
			marker->points[3].y() / camera.height * 2 - 1,
			marker->points[5].x() / camera.width * 2 - 1,
			marker->points[5].y() / camera.height * 2 - 1
		});
	}
	if (markerLines.size() > 0)
	{
		glLineWidth(2.0f);
		glColor3f(1, 1, 0);
		glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line2D) * markerLines.size(), &markerLines[0], GL_STREAM_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line2D)/2, (void *)0);
		glEnableVertexAttribArray(0);
		glDrawArrays(GL_LINES, 0, (int)markerLines.size()*2);
	}
}

/**
 * Visualize 3D Rays in world space
 */
void visualizeRays(const Camera &camera, const std::vector<Ray> &rays3D)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);

	// Render ray lines
	std::vector<struct Line2D> rayLines;
	for (int r = 0; r < rays3D.size(); r++)
	{
		const Ray *ray = &rays3D[r];
		const int segCnt = 5;
		for (int i = 0; i < segCnt; i ++)
		{ // Display ray in segments to prevent weird clipping
			Eigen::Vector3f start = projectPoint(vp, (ray->pos + ray->dir * i * 1000/segCnt));
			Eigen::Vector3f end = projectPoint(vp, (ray->pos + ray->dir * (i+1) * 1000/segCnt));
			// Register ray segment
			rayLines.push_back({
				start.x(),
				start.y(),
				end.x(),
				end.y()
			});
		}
	}
	if (rayLines.size() > 0)
	{	
		glLineWidth(0.5f);
		glColor3f(0, 0.5f, 0);
		glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line2D) * rayLines.size(), &rayLines[0], GL_STREAM_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line2D)/2, (void *)0);
		glEnableVertexAttribArray(0);
		glDrawArrays(GL_LINES, 0, (int)rayLines.size()*2);
	}
}

/**
 * Visualize triangulated point cloud in world space
 */
void visualizeTriangulation(const Camera &camera, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);

	// Render all triangulated points
	glColor3f(1,0,0);
	for (int i = 0; i < nonconflictedCount; i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i].pos);
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x(), pt.y(), pt.z());
		glEnd();
	}
	glColor3f(1,0,1);
	for (int i = nonconflictedCount; i < points3D.size(); i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i].pos);
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x(), pt.y(), pt.z());
		glEnd();
	}
}

/**
 * Visualize camera distortion using a grid of size num
 */
void visualizeDistortion(const Camera &cameraCB, const Camera &cameraGT)
{
	// Camera projections
	Eigen::Projective3f PGT = createProjectionMatrix(cameraGT.fovH, cameraGT.fovV);
	Eigen::Projective3f PCB = createProjectionMatrix(cameraCB.fovH, cameraCB.fovV);

	// Grid parameters
	int num = 10;
	float hSize = 150, vSize = 150, dist = 100;

	// Resulting points
	std::vector<Eigen::Vector2f> undistorted, distortedGT, distortedCB;
	undistorted.reserve(num*num);
	distortedGT.reserve(num*num);
	distortedCB.reserve(num*num);

	for (int i = 1; i < num+1; i++)
	{
		for (int j = 1; j < num+1; j++)
		{
			Eigen::Vector3f point (i * hSize/(num+1) - hSize/2, j * vSize/(num+1) - vSize/2, -dist);
			Eigen::Vector2f projGT = projectPoint(PGT, point).head<2>();
			Eigen::Vector2f projCB = projectPoint(PCB, point).head<2>();
			projGT.x() = (projGT.x()+1)/2 * cameraGT.width;
			projGT.y() = (projGT.y()+1)/2 * cameraGT.height;
			projCB.x() = (projCB.x()+1)/2 * cameraCB.width;
			projCB.y() = (projCB.y()+1)/2 * cameraCB.height;
			undistorted.push_back(projGT);
			distortedGT.push_back(distortPoint(cameraGT, projGT));
			distortedCB.push_back(distortPoint(cameraCB, projCB));
		}
	}

	visualizePoints2D(cameraGT, undistorted, { 0,0,1 }, 2.0f, 0.6f);
	visualizePoints2D(cameraGT, distortedGT, { 0,1,0 }, 2.0f, 0.5f);
	visualizePoints2D(cameraCB, distortedCB, { 1,1,0 }, 2.0f, 0.4f);
}