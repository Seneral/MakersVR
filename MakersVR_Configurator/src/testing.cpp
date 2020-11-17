/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "testing.hpp"

#include "wxbase.hpp" // wxLog*

#include <random>

/* Variables */

std::mt19937 gen = std::mt19937(std::random_device{}());

/* Functions */

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