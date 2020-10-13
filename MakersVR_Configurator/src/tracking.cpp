/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define MAX_CAMERA_COUNT 3
#define NUM_CLOSEST_RELATIONS 4
#define MIN_DISTANCE_RELATIONS 2 // cm

#include "tracking.hpp"

#include "wxbase.hpp" // wxLog*

#include "Eigen/SVD"


/**
 * Tracking
 */


/* Structures */

struct Intersection
{
	Eigen::Vector3f center;
	float error;
	Intersection *merge;
	Ray* rays[MAX_CAMERA_COUNT];
};

struct MarkerCandidate3D
{
	float validity;
	std::vector<int> points;
	std::vector<int> pointMap;
	// To get triangulated points: points[points[i]]
	// To get actual points: marker[pointMap[points3D[i]]]
	Eigen::Isometry3f estTransform;
};

struct MatchCandidate {
	int poseC; // Current
	int poseL; // Last
	float positionalError;
	float angularError;
	float directionalError;
};


/* Operators */

bool operator<(const struct PointRelation& a, const struct PointRelation& b) { return a.distance < b.distance; }
bool ErrorRangeComp::operator() (const PointRelation& rel, float value) { return rel.distance < value-error; }
bool ErrorRangeComp::operator() (float value, const PointRelation& rel) { return value+error < rel.distance; }


/* Functions */

/**
 * Sets up lookup tables for quick marker identification
 */
void generateLookupTables(MarkerTemplate3D *marker3D)
{
	if (MAX_MARKER_POINTS < marker3D->points.size())
	{
		wxLogError("Marker has more marker points (%d) than maximum code has been compiled with (%d)", (int)marker3D->points.size(), MAX_MARKER_POINTS);
	}

	// Setup lookup tables for quick marker identification
	marker3D->relationDist.clear();
	marker3D->pointRelation.clear();
	marker3D->pointRelation.resize(marker3D->points.size());

	// Step 0: Determine upper bound for total number of relations (binominal coefficient)
	// This estimate (n^k / k!) is excellent for small k (here k=2)
	int relUpperBound = marker3D->points.size() * marker3D->points.size() / 2;

	// Step 1: Create all relations
	std::vector<PointRelation> relations;
	relations.reserve(relUpperBound);
	for (int i = 0; i < marker3D->points.size(); i++)
	{
		Eigen::Vector3f pt1 = marker3D->points[i];
		for (int j = i+1; j < marker3D->points.size(); j++)
		{
			Eigen::Vector3f pt2 = marker3D->points[j];
			Eigen::Vector3f dir = pt2 - pt1;
			float distance = dir.norm();
			relations.push_back({ i, j, dir / distance, distance });
		}
	}

	// Step 2: Sort based on distance
	std::sort(relations.begin(), relations.end());

	// Step 3: Enter relevant relations
	int *pointRelationCount = new int[marker3D->points.size()];
	memset(pointRelationCount, 0, marker3D->points.size()*sizeof(int));
	for (int i = 0; i < relations.size(); i++)
	{
		PointRelation *rel = &relations[i];
		if (pointRelationCount[rel->pt1] < NUM_CLOSEST_RELATIONS || pointRelationCount[rel->pt2] < NUM_CLOSEST_RELATIONS || rel->distance < MIN_DISTANCE_RELATIONS)
		{ // Each point should have at least NUM_CLOSEST_RELATIONS stored relations
			pointRelationCount[rel->pt1]++;
			pointRelationCount[rel->pt2]++;
			int relIndex = marker3D->relationDist.size();
			marker3D->relationDist.push_back(*rel);
			marker3D->pointRelation[rel->pt1].push_back(relIndex);
			marker3D->pointRelation[rel->pt2].push_back(relIndex);
		}
	}

	// markerDistLookup is now a list of relations sorted by distance with at least NUM_CLOSEST_RELATIONS of the closest relations per point
}

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
void castRays(const std::vector<Eigen::Vector2f> &points2D, const Camera &camera, std::vector<Ray> &rays3D)
{
	Eigen::Projective3f vpInv = camera.transform * createProjectionMatrix(camera.fovH, camera.fovV).inverse();
	for (int i = 0; i < points2D.size(); i++)
	{
		Eigen::Vector3f start = camera.transform.translation();
		Eigen::Vector3f end(points2D[i].x()/camera.width*2-1, points2D[i].y()/camera.height*2-1, 1.0f);
		end = projectPoint(vpInv, end); // Clip space to world space
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

/**
 * Calculate the intersection points between rays of separate groups
 * Returns how many of the points are not conflicted (those are at the beginning of the array)
 */
int triangulateRayIntersections(std::vector<std::vector<Ray>*> &rayGroups, std::vector<TriangulatedPoint> &points3D, std::vector<std::vector<int>> &conflicts, float errorLimit)
{
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
					Eigen::Vector3f pos1 = ray1->pos + ray1->dir * sec1;
					Eigen::Vector3f pos2 = ray2->pos + ray2->dir * sec2;
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
			ixm->center = Eigen::Vector3f::Zero();
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

static void kabsch(const MarkerTemplate3D *marker3D, MarkerCandidate3D *candidate, const std::vector<TriangulatedPoint> &points3D)
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
		mkCenter += marker3D->points[mkPt];
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
		mkMat.col(j) = marker3D->points[mkPt] - mkCenter;
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
void detectMarkers3D(const MarkerTemplate3D *marker3D, const std::vector<TriangulatedPoint> &points3D, const std::vector<std::vector<int>> &conflicts, int nonconflictedCount, std::vector<Eigen::Isometry3f> &poses3D, std::vector<std::pair<float,int>> &posesMSE, float sigmaError)
{
//	wxLogMessage("--------------");

	// Step 1: Get closest neighbouring points for each point
	float maxRelevantDistance = marker3D->relationDist.size() == 0? 0.0f : 
		marker3D->relationDist[marker3D->relationDist.size()-1].distance + 0.1f;
	float maxRelevantDistanceSq = maxRelevantDistance*maxRelevantDistance;
	std::vector<std::vector<PointRelation>> trRelations;
	trRelations.resize(points3D.size());
	for (int i = 0; i < points3D.size(); i++)
	{
		const TriangulatedPoint *pt1 = &points3D[i];
		for (int j = i+1; j < points3D.size(); j++)
		{
			const TriangulatedPoint *pt2 = &points3D[j];
			Eigen::Vector3f dir = pt2->pos - pt1->pos;
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
	auto check3PointCandidate = [&points3D, &trRelations, &candidates, &assigned, &bestCandidatePtCount, &sigmaError](
		const MarkerTemplate3D *marker3D, PointRelation *trRelBase, PointRelation *trRelArm,
		int trPtJ, int trPtB, int trPtA,
		float baseError, float armError,
		std::pair<std::vector<PointRelation>::const_iterator, std::vector<PointRelation>::const_iterator> mkRelBaseRange)
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
				for (int n = 0; n < marker3D->pointRelation[mkPtJ].size(); n++)
				{
					const PointRelation *mkRelArm = &marker3D->relationDist[marker3D->pointRelation[mkPtJ][n]];
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
						kabsch(marker3D, candidate, points3D);

						// Prepare assigned for marker points, used to avoid duplicate checks
						std::vector<bool> mkPtHandled;
						mkPtHandled.resize(marker3D->points.size());
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
							float ptError = sigmaError * points3D[j].error;
							float distFromCoM = (points3D[j].pos.head<3>() - candidate->estTransform.translation()).norm();
							float errorLimit = ptError + poseError * distFromCoM;
							for (int k = 0; k < marker3D->points.size(); k++)
							{
								if (mkPtHandled[k]) continue;
								float errorSq = (estMkPos - marker3D->points[k]).squaredNorm();
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
						kabsch(marker3D, candidate, points3D);

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
			float baseError = sigmaError * (points3D[trRelBase->pt1].error + points3D[trRelBase->pt2].error);
			auto mkRelBaseRange = std::equal_range(marker3D->relationDist.begin(), marker3D->relationDist.end(), trRelBase->distance, ErrorRangeComp(baseError));
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

				float armError = sigmaError * (points3D[trRelArm->pt1].error + points3D[trRelArm->pt2].error);
//				wxLogMessage("_ - Tr Arm Relation (%d - %d) with length %f += %f!", trRelArm->pt1, trRelArm->pt2, trRelArm->distance, armError);

				// Check 3 points if they are a valid candidate
				// and if so, extend to include matching neighboring points
				// if 5+ points have been found, it is an almost certain match and points are assigned to not be checked again
				check3PointCandidate(marker3D,
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

				float armError = sigmaError * (points3D[trRelArm->pt1].error + points3D[trRelArm->pt2].error);
//				wxLogMessage("_ - Tr Arm Relation (%d - %d) with length %f += %f!", trRelArm->pt1, trRelArm->pt2, trRelArm->distance, armError);

				// Check 3 points if they are a valid candidate
				// and if so, extend to include matching neighboring points
				// if 5+ points have been found, it is an almost certain match and points are assigned to not be checked again
				check3PointCandidate(marker3D,
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
				Eigen::Vector3f mkPos = marker3D->points[mkPt];
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
		poses3D.push_back(candidate->estTransform);
		posesMSE.push_back({ lowestMSE, candidate->points.size() });
//		wxLogMessage("Best candidate %d of %d total with %d points has internal MSE of %f", lowestMSEInd, (int)candidates.size(), (int)candidate->points.size(), lowestMSE);
	}
	else
	{ // No candidate at all
		//wxLogMessage("No candidate has been found! %d points, of those %d not conflicted", (int)points3D.size(), nonconflictedCount);
		/*for (int i = 0; i < points3D.size(); i++)
		{
			float min = 0, max = maxRelevantDistance;
			if (trRelations[i].size() > 0)
			{
				min = trRelations[i][0].distance;
				max = trRelations[i][trRelations[i].size()-1].distance;
			}
			wxLogMessage("--> Point %d got a total of %d points from %f to %f cm", i, (int)trRelations[i].size(), min, max);
		}*/
	}

//	wxLogMessage("---------");
}

/**
 * Matches the current poses to the poses of the last frame using temporal information
 */
void matchTrackedPoses(const std::vector<Eigen::Isometry3f> &currentPose, const std::vector<Eigen::Isometry3f> &lastPose, const std::vector<Eigen::Vector3f> &lastDir, const std::vector<float> &lastRot, std::vector<int> &matching)
{
	float t = 1; // TODO: Delta T, just in case of frame drops;

	matching.resize(currentPose.size());
	for (int i = 0; i < matching.size(); i++) matching[i] = -1;

	// Step 1: Add all potential matches with large error of margin
	std::vector<MatchCandidate> candidates;
	std::vector<std::vector<int>> poseMapC (currentPose.size());
	std::vector<std::vector<int>> poseMapL (lastPose.size());
	for (int i = 0; i < lastPose.size(); i++)
	{
		Eigen::Vector3f predPos = lastPose[i].translation() + lastDir[i] * t;

		#define REL_ERROR 0.5 * t
		float acceptedPosErr = lastDir[i].norm() * REL_ERROR + 10;
		float acceptedPosErrSq = acceptedPosErr*acceptedPosErr;
		float acceptedRotErr = lastRot[i] * REL_ERROR + (10.0f/180.0f*PI);

		for (int j = 0; j < currentPose.size(); j++)
		{
			Eigen::Vector3f diff = currentPose[j].translation()-predPos;
			float posErrSq = diff.squaredNorm();
			if (posErrSq < acceptedPosErrSq)
			{ // Potential match
				int index = candidates.size();
				candidates.push_back({
					j, i, std::sqrt(posErrSq), NAN, NAN
				});
				poseMapL[i].push_back(index);
				poseMapC[j].push_back(index);
			}
			else
			{
				wxLogMessage("Discarded match %d/%d with error %f > %f", j, i, std::sqrt(posErrSq), std::sqrt(acceptedPosErrSq));
			}
		}
	}

	// Step 2: Register best matches
	for (int i = 0; i < candidates.size(); i++)
	{
		MatchCandidate *candidate = &candidates[i];
		// TODO: Currently simple check for the best match
		// might leave some heavily disputed matches (triangle) completely unregistered
		if (poseMapC[candidate->poseC].size() != 1)
		{ // Got competitor
			bool discard = false;
			for (int j = 0; j < poseMapC[candidate->poseC].size(); j++)
			{ // Check if there is a better match
				int can = poseMapC[candidate->poseC][j];
				if (discard = (can != i && candidates[can].positionalError < candidate->positionalError)) break;
			}
			if (discard) break;
		}
		if (poseMapL[candidate->poseL].size() != 1)
		{ // Got competitor
			bool discard = false;
			for (int j = 0; j < poseMapL[candidate->poseL].size(); j++)
			{ // Check if there is a better match
				int can = poseMapL[candidate->poseL][j];
				if (discard = (can != i && candidates[can].positionalError < candidate->positionalError)) break;
			}
			if (discard) break;
		}
		matching[candidate->poseC] = candidate->poseL;
	}
}

/**
 * Accepts the previously calculated matching and updates temporal information
 */
void matchAccept(std::vector<Eigen::Isometry3f> &currentPose, std::vector<Eigen::Isometry3f> &lastPose, std::vector<Eigen::Vector3f> &lastDir, std::vector<float> &lastRot, const std::vector<int> &matching)
{
	lastDir.resize(currentPose.size());
	lastRot.resize(currentPose.size());
	for (int i = 0; i < currentPose.size(); i++)
	{
		if (matching[i] != -1)
		{ // Got a match to track
			lastDir[i] = currentPose[i].translation()-lastPose[matching[i]].translation();
			lastRot[i] = Eigen::AngleAxisf(currentPose[i].rotation() * lastPose[matching[i]].rotation().transpose()).angle();
		}
		else
		{
			lastDir[i] = Eigen::Vector3f::Zero();
			lastRot[i] = 0;
		}
	}
}