/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "calibration.hpp"

#include "wxbase.hpp" // wxLog*

#include <bitset>

#define USE_OPENCV
#ifdef USE_OPENCV
// Don't include full library, only needed modules
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#endif

// Parameters for findMarkerCandidates
#define MAX_SIZE_DIFF 4 // Max relative blob size difference
#define MAX_LINE_DIFF 0.2 // Max relative marker length difference
#define MAX_LENGTH_DIFF 0.3 // Max relative marker length difference
#define MAX_ANGLE_DIFF 15.0f/180*PI // Max angle difference in radians

/**
 * Calibration
 */


/* Structures */

struct MarkerCandidate
{
	int id;
	std::vector<int> points;
	float error;
};


/* Functions */

/**
 * Fill with built-in markers that findMarkerCandidate can detect
 */
void getBuiltInMarkers(std::vector<DefMarker> &markers2D)
{
	markers2D.push_back({
		0, "Default-6Pt-PadBoard",
		{ 
			{ Eigen::Vector3f(-4.572f, -4.572f, 0.0f), Eigen::Vector3f(0,0,1), 160 }, // 3-Base left
			{ Eigen::Vector3f(+4.572f, -4.572f, 0.0f), Eigen::Vector3f(0,0,1), 160 }, // 3-Base right
			{ Eigen::Vector3f(   0.0f,    0.0f, 0.0f), Eigen::Vector3f(0,0,1), 160 }, // Center
			{ Eigen::Vector3f(-2.286f, +4.572f, 0.0f), Eigen::Vector3f(0,0,1), 160 }, // Head left
			{ Eigen::Vector3f(+2.286f, +4.572f, 0.0f), Eigen::Vector3f(0,0,1), 160 }, // Head right
			{ Eigen::Vector3f(   0.0f, -4.572f, 0.0f), Eigen::Vector3f(0,0,1), 160 }  // 3-Base center
		}
	});
}

/**
 * Returns if marker ID can be detected by findMarkerCandidates
 */
bool isMarkerBuiltIn(int id)
{
	return id == 0;
}

/**
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(const std::vector<Eigen::Vector2f> &points2D, const std::vector<float> &pointSizes, std::vector<Marker2D> &markers2D, std::vector<int> &freePoints2D)
{
	static std::vector<MarkerCandidate> markerCandidates;
	static std::bitset<512> markerTag;

	// Find and extract all markers candidates (base line of 3 dots plus header)
	// Step 1: Find all base lines (3 points in a straight line with equal distance)
	// Step 2: For each line, find the leading point on one side
	markerCandidates.clear();
	for (int c = 0; c < points2D.size(); c++)
	{
		for (int a = 0; a < points2D.size(); a++)
		{
			if (a == c) continue;

			// Check size difference
			float sizeMinC = std::min(pointSizes[c], pointSizes[a]);
			float sizeMaxC = std::max(pointSizes[c], pointSizes[a]);
			if (sizeMinC * MAX_SIZE_DIFF < sizeMaxC) continue;

			for (int b = a+1; b < points2D.size(); b++)
			{
				if (b == c) continue;

				// Check size difference
				float sizeMinB = std::min(sizeMinC, pointSizes[b]);
				float sizeMaxB = std::max(sizeMaxC, pointSizes[b]);
				if (sizeMinB * MAX_SIZE_DIFF < sizeMaxB) continue;

				// Get point positions
				Eigen::Vector2f ptC = points2D[c];
				Eigen::Vector2f ptA = points2D[a];
				Eigen::Vector2f ptB = points2D[b];

				// Get base parameters
				Eigen::Vector2f base = ptA - ptB;
				float baseLen = base.norm();

				// Check if c is at the center of a and b, building a base line
				float baseCenterDiff = (ptC*2 - ptA - ptB).norm();
				float baseCenterError = baseCenterDiff / (baseLen/2);
				if (baseCenterError > MAX_LINE_DIFF) continue;
				
				// Found a base
				//float baseAngle = std::atan2(base.y(), base.x()) / PI * 180;

				// Search for parallel header
				for (int e = 0; e < points2D.size(); e++)
				{
					if (e == c || e == a || e == b) continue;

					// Check size difference
					float sizeMinE = std::min(sizeMinB, pointSizes[e]);
					float sizeMaxE = std::max(sizeMaxB, pointSizes[e]);
					if (sizeMinE * MAX_SIZE_DIFF < sizeMaxE) continue;

					for (int f = e+1; f < points2D.size(); f++)
					{
						if (f == c || f == a || f == b) continue;

						// Check size difference
						float sizeMinF = std::min(sizeMinE, pointSizes[f]);
						float sizeMaxF = std::max(sizeMaxE, pointSizes[f]);
						if (sizeMinF * MAX_SIZE_DIFF < sizeMaxF) continue;

						// Get point positions
						Eigen::Vector2f ptE = points2D[e];
						Eigen::Vector2f ptF = points2D[f];

						// Get head parameters
						Eigen::Vector2f head = ptE - ptF;
						float headLen = head.norm();

						// Check if head length is approximately half the size of the base
						float headLenError = headLen * 2 / baseLen;
						if (headLenError > 1.0f+MAX_LENGTH_DIFF || headLenError < 1.0f-MAX_LENGTH_DIFF)
							continue;

						// Check if head is aligned with base
						float parallelsAngleDiff = std::acos(std::abs(base.normalized().dot(head.normalized())));
						if (parallelsAngleDiff > MAX_ANGLE_DIFF)
							continue;

						// Found head, now find center
						Eigen::Vector2f headCenter = (ptE+ptF) / 2;
						Eigen::Vector2f centerPos = (headCenter + ptC) / 2;
						Eigen::Vector2f vertical = headCenter - ptC;
						float verticalLength = vertical.norm();
						
						for (int d = 0; d < points2D.size(); d++)
						{
							if (d == c || d == a || d == b || d == e || d == f) continue;

							// Check size difference
							float sizeMinD = std::min(sizeMinF, pointSizes[d]);
							float sizeMaxD = std::max(sizeMaxF, pointSizes[d]);
							if (sizeMinD * MAX_SIZE_DIFF < sizeMaxD) continue;

							// Check if point is close to center
							Eigen::Vector2f ptD = points2D[d];
							float centerDiff = (ptD-centerPos).norm();
							float centerError = centerDiff / verticalLength * 2;
							if (centerError > MAX_LENGTH_DIFF)
								continue;

							// Now order base and head points correctly
							int aR = a, bR = b, eR = e, fR = f;
							if (vertical.x() * base.y() - vertical.y()*base.x() < 0)
							{ // Wrong side, flip base
								aR = b;
								bR = a;

								// Check if head needs to be flipped, too
								if (head.dot(base) > 0)
								{
									eR = f;
									fR = e;
								}
							}
							else
							{ // Base is correctly oriented
								// Check if head needs to be flipped
								if (head.dot(base) < 0)
								{
									eR = f;
									fR = e;
								}
							}

							// Register marker candidate with all error terms
							markerCandidates.push_back({
								0, { aR, bR, d, eR, fR, c }, 
								baseCenterError + centerError + headLenError + parallelsAngleDiff
							});
						}
					}
				}
			}
		}
	}

	// Sort marker candidates by accuracy
	std::sort(markerCandidates.begin(), markerCandidates.end(), [](MarkerCandidate m1, MarkerCandidate m2) {
		return m1.error < m2.error;
	});

	// Find best marker candidates and associate them
	markerTag.reset();
	markers2D.reserve(markers2D.size() + markerCandidates.size());
	for (int m = 0; m < markerCandidates.size(); m++)
	{
		MarkerCandidate *mc = &markerCandidates[m];
		
		// Make sure all points of the candidates are unoccupied
		bool occupied = false;
		for (int i = 0; i < mc->points.size(); i++)
			if (occupied = markerTag[mc->points[i]]) break;
		if (occupied) continue;

		// Set blobs as occupied
		for (int i = 0; i < mc->points.size(); i++)
			markerTag.set(mc->points[i]);

		// Register marker
		markers2D.push_back(Marker2D(mc->id, mc->points.size()));
		Marker2D *marker = &markers2D[markers2D.size()-1];
		for (int i = 0; i < mc->points.size(); i++)
			marker->points[i] = points2D[mc->points[i]];

#ifdef MARKER_DEBUG
		std::cout << "Chose marker with error " << mc->error << "!" << std::endl;
#endif
	}

	// Find blobs not part of any marker
	if (points2D.size() > markers2D.size()*4)
	{ // There are blobs not part of any marker
		for (int b = 0; b < points2D.size(); b++)
		{
			if (!markerTag.test(b))
			{ // Blob is not part of any marker
				freePoints2D.push_back(b);
			}
		}
	}
}

#ifdef USE_OPENCV
/**
 * Interprets OpenCV position and rotation in camera spaces and transforms it into correct scene transformations
 */
Eigen::Isometry3f interpretCVSpace(const cv::Matx31d &cvPos, const cv::Matx31d &cvRot)
{
	Eigen::Isometry3f pose;

	// Z-Axis of OpenGL/Blender coordinate frame is opposite of OpenCV
	// OpenGL/Blender is -z foward, +z towards viewer
	pose.translation() = Eigen::Vector3f(cvPos(0), cvPos(1), -cvPos(2));

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
	pose.linear() = getRotationXYZ(rotAngles);

	return pose;
}
#endif

/**
 * Infer the pose of a marker in camera space given its image points in pixel space and the intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPose(const Camera &camera, const Marker2D &marker2D, const DefMarker &markerTemplate2D)
{
#ifdef USE_OPENCV
	// Create CV 3D array of marker template points
	std::vector<cv::Point3f> cv_pointsTemplate3D;
	for (int i = 0; i < markerTemplate2D.points.size(); i++)
		cv_pointsTemplate3D.push_back(cv::Point3f(markerTemplate2D.points[i].pos.x(), markerTemplate2D.points[i].pos.y(), markerTemplate2D.points[i].pos.z()));

	// Create CV 2D array of marker points
	std::vector<cv::Point2f> cv_points2D;
	for (int i = 0; i < marker2D.points.size(); i++)
		cv_points2D.push_back(cv::Point2f(marker2D.points[i].x(), marker2D.points[i].y()));

	// Compose camera matrix
	float w = (float)camera.width, h = (float)camera.height;
	float fx = w/std::tan(camera.fovH/180.0f*PI/2)/2, fy = h/std::tan(camera.fovV/180.0f*PI/2)/2;
	cv::Matx33f cv_camMat = cv::Matx33f(
		fx, 0, w/2,
		0, fy, h/2,
		0, 0, 1);

	// Set distortion values -- already undistorted marker points
/*	cv::Matx<double,5,1> cv_distort;
	cv_distort(0) = camera.distortion.k1;
	cv_distort(1) = camera.distortion.k2;
	cv_distort(2) = camera.distortion.p1;
	cv_distort(3) = camera.distortion.p2;
	cv_distort(4) = camera.distortion.k3;*/

	// use solvePnP to recreate rotation and translation
	cv::Matx31d cv_rotation;
	cv::Matx31d cv_translation;
	cv::solvePnP(cv_pointsTemplate3D, cv_points2D, cv_camMat, cv::noArray(), cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

	// Convert from OpenCV to Blender/OpenGL space
	return interpretCVSpace(cv_translation, cv_rotation);
#else
	return Eigen::Isometry3f::Identity();
#endif
}

/**
 * Calculate mean squared error in image space of detected pose
 */
float calculateMSE(const Eigen::Isometry3f &pose3D, const Camera &camera, const Marker2D &marker2D, const DefMarker &markerTemplate2D)
{
	Eigen::Projective3f p = createProjectionMatrix(camera.fovH, camera.fovV);
	float mse = 0.0f;
	for (int i = 0; i < markerTemplate2D.points.size(); i++)
	{
		Eigen::Vector2f pt = projectPoint(p, pose3D * markerTemplate2D.points[i].pos).head<2>();
		pt = Eigen::Vector2f((pt.x()+1)/2*camera.width, (pt.y()+1)/2*camera.height);
		Eigen::Vector2f diffPx = marker2D.points[i] - pt; // Convert difference to pixels
		mse += diffPx.squaredNorm();
	}
	return mse / markerTemplate2D.points.size();
}

/**
 * Adds transform to a candidate within error margin or creates a new one
 */
int AddTransformToCandidates(std::vector<TransformCandidate> &candidates, Eigen::Isometry3f transform, float weight, float maxTError, float maxRError)
{
	for (int c = 0; c < candidates.size(); c++)
	{
		TransformCandidate *candidate = &candidates[c];
		// Calculate error between candidate and data point
		Eigen::Vector3f tDiff = transform.translation() - candidate->transform.translation();
		Eigen::Matrix3f rDiff = transform.rotation() * candidate->transform.rotation().transpose();
		Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
		float tError = tDiff.norm(), rError = rDiffAx.angle()/PI*180;
		if (tError < maxTError && rError < maxRError)
		{
			// Adjust weight
			weight = weight / (1 + tError/maxTError) / (1 + rError/maxRError);
			// Accept to candidate
			candidate->datapoints.push_back({ transform, weight });
			candidate->weight = candidate->weight + weight;
			// Interpolate position and rotation
			candidate->transform.translation() += tDiff * weight/candidate->weight;
			rDiffAx.angle() *= weight/candidate->weight;
			candidate->transform.linear() = rDiffAx * candidate->transform.linear();
			return c;
		}
	}

	// Create new candidate
	candidates.push_back({ transform, weight, { { transform, weight } } });
	return (int)candidates.size()-1;
}

/**
 * Choose transform canidate among the given candidates with the most weight
 */
TransformSample chooseBestTransformCandidate(const std::vector<TransformCandidate> &candidates)
{
	// Choose best candidate for this cameras relation to the origin
	int candidateInd = -1;
	float weight = 0;
	for (int c = 0; c < candidates.size(); c++)
	{
		if (weight < candidates[c].weight)
		{
			candidateInd = c;
			weight = candidates[c].weight;
		}
	}
	// Output sample
	TransformSample sample;
	if (candidateInd != -1)
	{ // Apply candidate and analyze
		const TransformCandidate *candidate = &candidates[candidateInd];
		sample.transform = candidate->transform;
		sample.weight = candidate->weight;
		sample.sampleCount = candidate->datapoints.size();
		
		// Calculate standard deviations
		sample.stdDevT = 0;
		sample.stdDevR = 0;
		for (int i = 0; i < candidate->datapoints.size(); i++)
		{
			Eigen::Vector3f tDiff = candidate->datapoints[i].first.translation() - candidate->transform.translation();
			Eigen::Matrix3f rDiff = candidate->datapoints[i].first.rotation() * candidate->transform.rotation().transpose();
			Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
			float tError = tDiff.norm(), rError = rDiffAx.angle()/PI*180;
			sample.stdDevT += tError*tError * candidate->datapoints[i].second;
			sample.stdDevR += rError*rError * candidate->datapoints[i].second;
		}
		sample.stdDevT = std::sqrt(sample.stdDevT / candidate->weight);
		sample.stdDevR = std::sqrt(sample.stdDevR / candidate->weight);

		// TODO: Filter out outliers out of datapoints to improve reliability
	}
	else
		sample.transform = Eigen::Isometry3f::Identity();
	return sample;
}

/**
 * Calculate absolute camera transforms with weighted samples of origin and relations between cameras, with focus on keeping the relations intact
 */
void calculateCameraTransforms(std::vector<TransformSample*> &origins, std::vector<std::vector<TransformSample*>> &relations, std::vector<Eigen::Isometry3f> &cameraTransforms)
{
	// Determine absolute position of cameras
	// Instead of simply taking the origin pose from each camera (which can be noisy or non existant based on camera position),
	// the relation to other cameras is favoured more and included,
	// since they are (1) more important and (2) potentially more reliable (pose can be closer to each camera).
	// So for each camera, all paths to the origin through all cameras are considered.
	// These paths are weighted based on it's weakest link by using the parallel resistor formula.
	// This also naturally supports multi-room setups where not each camera can see the origin,
	// instead only some overlapping space with at least one other camera

	// Complexity: cnt!/(cnt-maxP)! for cnt cameras and maximum path length maxP
	// CAN be drastically improved with a different implementation of the same algorithm but this simple one suffices for now

	const int cnt = origins.size();
	const int maxP = 4;
	const int len = std::min(cnt, maxP);
	std::vector<bool> used (cnt);
	std::vector<int> path (len);
	std::vector<std::vector<std::pair<Eigen::Isometry3f, float>>> transforms (cnt);
	int pos = 0;

	while (true)
	{
		while (path[pos] < cnt && used[path[pos]])
			path[pos]++;
		if (path[pos] >= cnt)
		{ // Finished this branch, backtrack and start next
			pos--;
			if (pos < 0) break;
			used[path[pos]] = false;
			path[pos]++;
			continue;
		}

		// Get weight of path (calculated so that the weakest link limits total path weight, just like resistors)
		float pathWeight = 1/origins[path[pos]]->weight;
		bool pathInvalid = false;
		for (int i = pos; i > 0; i--)
		{
			float relWeight = relations[path[i]][path[i-1]]->weight;
			if (relWeight == 0)
			{ // No candidate at all
				pathInvalid = true;
				break;
			}
			else
				pathWeight += 1/relWeight;

		}
		pathWeight = 1/pathWeight;

		if (!pathInvalid)
		{
			// Calculate path transform
			Eigen::Isometry3f pathTransform = origins[path[pos]]->transform.inverse();
			for (int i = pos; i > 0; i--)
			{
				Eigen::Isometry3f transform = relations[path[i]][path[i-1]]->transform;
				if (path[i-1] < path[i]) transform = transform.inverse();
				pathTransform = pathTransform * transform;
			}

			// Register as weighted path transform
			transforms[path[0]].push_back({ pathTransform, pathWeight });

			{ // Debug
				std::stringstream ss;
				for (int i = 0; i <= pos; i++)
					ss << path[i] << ":";
				wxLogMessage("Path weight %.4f: %s!", pathWeight, ss.str());
			}
		}

		if (pos+1 < len)
		{ // Advance branch
			used[path[pos]] = true;
			pos++;
			path[pos] = 0;
			continue;
		}
		else
		{ // Reached end, cycle through unused ones
			path[pos]++;
		}
	}

	// Average out weighted path transforms
	cameraTransforms.clear();
	cameraTransforms.reserve(cnt);
	for (int m = 0; m < cnt; m++)
	{
		float totalWeight = 0.0f;
		Eigen::Isometry3f cameraTransform = Eigen::Isometry3f::Identity();
		for (int t = 0; t < transforms[m].size(); t++)
		{
			float weight = transforms[m][t].second;
			Eigen::Isometry3f transform = transforms[m][t].first;
			// Calculate differences
			Eigen::Vector3f tDiff = transform.translation() - cameraTransform.translation();
			Eigen::Matrix3f rDiff = transform.rotation() * cameraTransform.rotation().transpose();
			Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
			// Interpolate position and rotation
			totalWeight += weight;
			cameraTransform.translation() += tDiff * weight/totalWeight;
			rDiffAx.angle() *= weight/totalWeight;
			cameraTransform.linear() = rDiffAx * cameraTransform.linear();
		}
		cameraTransforms.push_back(cameraTransform);

		// Debug
		wxLogMessage("Cam %d: %d paths, total weight %.4f!", m, (int)transforms[m].size(), totalWeight);
	}
}

/**
 * Adds markers that add value to the calibration to the calibration selection and adjusts selection parameters
 */
bool selectMarkersForCalibration(const Camera &camera, const DefMarker &markerTemplate2D, const std::vector<Marker2D> &markers, std::vector<Marker2D> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, float radialGranularity, float radialTarget, std::vector<std::vector<uint16_t>> &gridBuckets, int gridSize, int gridTarget, float threshold, int m)
{
	// granularity: Range of radial density check, specifies granularity at which markers are required and checked
	// target: Local target density in the radial lookup table
	// threshold: The minimum value of a marker (in terms of reaching target density) before it's added to the selection 
	const float cornerCut = 0.7f;
	// CornerCut 1.0f: strict coverage until the farthest corner needed
	// CornerCut 0.0f: only coverage of central rhombus reqired (corners cut to screen edge centers)
	float w = camera.width, h = camera.height, a = h/w;

	auto calcMarkerDensity = [&radialLookup](float r, float range) {
		auto a = radialLookup.lower_bound(r-range);
		auto b = radialLookup.upper_bound(r+range);
		float accum = 0;
		for (auto i = a; i != b; i++)
			accum += 1-std::abs((*i).first - r)/range; // 1 at center, 0 at the edge
		return accum;
	};
	auto getBucket = [&gridBuckets, w, h, gridSize](Eigen::Vector2f pos) {
		int x = std::min((int)(pos.x()/w * gridSize), gridSize-1);
		int y = std::min((int)(pos.y()/h * gridSize), gridSize-1);
		return &gridBuckets[y * gridSize + x];
	};

	// Add detected markers if they add value to the calibration selection
	bool addedMarker = false;
	for (int p = 0; p < markers.size(); p++)
	{
		const Marker2D *marker = &markers[p];

		// Only select markers of the desired type
		if (marker->id != markerTemplate2D.id || marker->points.size() != markerTemplate2D.points.size()) continue;

		// Determine value of adding marker to selection
		float radialValue = 0, gridValue = 0;
		for (int i = 0; i < marker->points.size(); i++)
		{
			float r = Eigen::Vector2f((marker->points[i].x()-w/2)/w*2, (marker->points[i].y()-h/2)/w*2).norm();
			float density = calcMarkerDensity(r, radialGranularity);
			float scale = 10.0f/marker->points.size();
			radialValue += std::max(0.0f, radialTarget-density) * scale;
			gridValue += std::max(0.0f, (float)gridTarget-getBucket(marker->points[i])->size()) * scale;
		}

		// Determine if marker should be added
		if (radialValue < threshold*radialTarget && gridValue < threshold*gridTarget)
			continue;

		// Add marker
		int index = calibrationSelection.size();
		calibrationSelection.push_back(*marker);
		float rmin = 2, rmax = 0;
		for (int i = 0; i < marker->points.size(); i++)
		{
			float r = Eigen::Vector2f((marker->points[i].x()-w/2)/w*2, (marker->points[i].y()-h/2)/w*2).norm();
			rmin = std::min(rmin, r);
			rmax = std::max(rmax, r);
			radialLookup.insert({ r, (uint16_t)index });
			getBucket(marker->points[i])->push_back((uint16_t)index);
		}
		wxLogMessage("Cam %d calibration: Marker (%.4f-%.4f); Value radial: %.4f, grid: %.4f", m, rmin, rmax, radialValue, gridValue);
		addedMarker = true;
	}

	// Check if calibration selection is broad enough to satisfy target
	int cnt = calibrationSelection.size();
	float aa = a*a, cc = cornerCut*cornerCut;
	float maxRange = std::max(std::sqrt(1*cc + aa), std::sqrt(1 + aa*cc));
	float step = radialGranularity*2;
	bool reachedTarget = true;
	if (reachedTarget)
	{
		for (float r = step; r < maxRange; r += step)
		{
			float density = calcMarkerDensity(r, step);
			if (density < radialTarget)
			{
				reachedTarget = false;
				if (addedMarker)
					wxLogMessage("Cam %d calibration: Lacking density at r %.4f +-%.4f with density %.4f < %.4f",
						m, r, step, density, radialTarget);
				break;
			}
		}
	}
	if (reachedTarget)
	{
		for (int i = 0; i < gridBuckets.size(); i++)
		{
			int count = gridBuckets[i].size();
			if (count < gridTarget/2)
			{
				reachedTarget = false;
				if (addedMarker)
					wxLogMessage("Cam %d calibration: Lacking bucket target at %d with count %d < %d",
						m, i, count, gridTarget);
				break;
			}
		}
	}

	return reachedTarget;
}

/**
 * Calibrate using a range of detected markers (each detected marker has to have the same point count as the template and the same ID)
 */
float calculateIntrinsicCalibration(Camera &camera, const std::vector<Marker2D> &markers, const DefMarker &markerTemplate2D, std::vector<double> &errors)
{
#ifdef USE_OPENCV
	// Create CV 2D array of marker points
	int ptCnt = markerTemplate2D.points.size();
	int mkCnt = markers.size();
	std::vector<std::vector<cv::Point2f>> cv_calibPoints2D(mkCnt);
	std::vector<std::vector<cv::Point3f>> cv_markerPoints3D(mkCnt);
	for (int m = 0; m < markers.size(); m++)
	{
		cv_calibPoints2D[m].reserve(ptCnt);
		cv_markerPoints3D[m].reserve(ptCnt);
		for (int i = 0; i < ptCnt; i++)
		{
			cv_calibPoints2D[m].push_back(cv::Point2f(markers[m].points[i].x(), markers[m].points[i].y()));
			cv_markerPoints3D[m].push_back(cv::Point3f(markerTemplate2D.points[i].pos.x(), markerTemplate2D.points[i].pos.y(), markerTemplate2D.points[i].pos.z()));
		}
	}

	float w = (float)camera.width, h = (float)camera.height;
	int calibFlags = cv::CALIB_FIX_PRINCIPAL_POINT;
	if (camera.fovH == 0 || camera.fovV == 0) //  || camera.fovH/camera.fovV-camera.width/camera.height > 0.01f)
	{ // Init with default values, just the aspect ration being correct
/*		camera.fovH = 45.0f;
		camera.fovV = std::atan(h/w * std::tan(camera.fovH/180.0f*PI/2))*2 * 180.0f/PI;
		calibFlags |= cv::CALIB_FIX_ASPECT_RATIO;*/
	}
	else
	{ // Already got some kind of calibration
		calibFlags |= cv::CALIB_USE_INTRINSIC_GUESS;
	}

	// Create camera matrix from existing guess or default values
	float fx = w/std::tan(camera.fovH/180.0f*PI/2)/2, fy = h/std::tan(camera.fovV/180.0f*PI/2)/2;
	cv::Matx33f cv_camMat = cv::Matx33f(
		fx, 0, w/2,
		0, fy, h/2,
		0, 0, 1);

	// Use distortion values if already existing
	cv::Matx<double,5,1> cv_distort;
	cv_distort(0) = camera.distortion.k1;
	cv_distort(1) = camera.distortion.k2;
	cv_distort(2) = camera.distortion.p1;
	cv_distort(3) = camera.distortion.p2;
	cv_distort(4) = camera.distortion.k3;

	// Output values
	std::vector<cv::Matx31d> cv_rotations;
	std::vector<cv::Matx31d> cv_translations;
	std::vector<double> stdDev_Intrinsic;
	std::vector<double> stdDev_Extrinsic;

	// Release Object method toggle
	int fixedPoint = -1;
//	fixedPoint = 1;
	std::vector<cv::Point3f> correctedPoints;
	std::vector<double> stdDev_ObjPts;

	// Calibration
	double stdDev = cv::calibrateCameraRO(
		cv_markerPoints3D, cv_calibPoints2D,																// Calibration data
		cv::Size2i(camera.width, camera.height), fixedPoint, cv_camMat, cv_distort,							// Camera model
		cv_rotations, cv_translations, correctedPoints,														// Output poses
		stdDev_Intrinsic, stdDev_Extrinsic, stdDev_ObjPts, errors,											// Output diagnostics (std dev, errors)
		calibFlags, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, FLT_EPSILON));	// Calibration model

	// Read out results
	camera.distortion.k1 = cv_distort(0);
	camera.distortion.k2 = cv_distort(1);
	camera.distortion.p1 = cv_distort(2);
	camera.distortion.p2 = cv_distort(3);
	camera.distortion.k3 = cv_distort(4);

	// Calculate field of view
	fx = cv_camMat(0,0);
	fy = cv_camMat(1,1);
	camera.fovH = std::atan(w/fx/2)*2 * 180.0f/PI;
	camera.fovV = std::atan(h/fy/2)*2 * 180.0f/PI;

	// Propagate standard deviation from focal length to field of view
	float sdfx = (float)stdDev_Intrinsic[0], sdfy = (float)stdDev_Intrinsic[1];
	float sdFovH = std::sqrt(sdfx*sdfx/fx/fx) / fx * w/2 / (1 + w*w/fx/fx/4) * 2 * 180.0f / PI;
	float sdFovV = std::sqrt(sdfy*sdfy/fy/fy) / fy * h/2 / (1 + h*h/fy/fy/4) * 2 * 180.0f / PI;

	wxLogMessage("StdDev FoV: (%f, %f), Dist: (%f, %f, %f, %f, %f)",
		sdFovH, sdFovV,
		stdDev_Intrinsic[4], stdDev_Intrinsic[5], stdDev_Intrinsic[6], stdDev_Intrinsic[7], stdDev_Intrinsic[8]);
	if (fixedPoint > 0)
		wxLogMessage("StdDev obj: %f %f %f %f %f %f", stdDev_ObjPts[0], stdDev_ObjPts[1], stdDev_ObjPts[2],
			stdDev_ObjPts[3], stdDev_ObjPts[4], stdDev_ObjPts[5]);

	return (float)stdDev;

#endif
	return 0;
}

/**
 * Takes the calibration selection and filters them using the errors from the last calibration round
 */
void filterMarkersForCalibration(const Camera &camera, const std::vector<double> &errors, std::vector<Marker2D> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, std::vector<std::vector<uint16_t>> &gridBuckets)
{
	// Calculate std deviation of marker errors
	float errorAvg = 0, errorStdDev = 0;
	for (int i = 0; i < errors.size(); i++)
		errorAvg += errors[i];
	errorAvg /= errors.size();
	for (int i = 0; i < errors.size(); i++)
		errorStdDev += (errors[i] - errorAvg) * (errors[i] - errorAvg);
	errorStdDev = std::sqrt(errorStdDev/errors.size());

	// Determine upper bounds for error
	// Sigma bounds for 80%, so worst 10% are out (and also best 10%, but those are kept)
	float errorBounds = errorStdDev*1.281552f;

	for (int i = 0; i < errors.size(); i++)
	{ // Remove the worst 10%
		if ((errors[i]-errorAvg) < errorBounds) continue;
		int index = i;

/*
	// Find the x worst fitting markers
	int cnt = calibrationSelection.size();
	int worstCnt = std::max(2, cnt/6);

	// Setup index arrays for sorting
	std::vector<int> idx;
	idx.reserve(cnt);
	for (int i = 0; i < cnt; i++)
		idx.push_back(i);

	// Sort by error
	std::partial_sort(idx.begin(), idx.begin()+worstCnt+5, idx.end(),
		[&errors](int i1, int i2) {return errors[i1] > errors[i2];});

	// Then again by index, to have highest index to remove in the front
	std::sort(idx.begin(), idx.begin()+worstCnt,
		[&errors](int i1, int i2) {return i1 > i2;});

	// Remove worst ones
	for (int i = 0; i < worstCnt; i++)
	{
		int index = idx[i];
*/

		// Switch with last element and remove it, keeps all relevant indices because we start deleting from the highest index
		calibrationSelection[index] = calibrationSelection.back();
		calibrationSelection.pop_back();
		int chIndex = calibrationSelection.size();
		// Erase all points belonging to removed marker
		auto it = radialLookup.begin();
		while (it != radialLookup.end())
		{
			if (it->second == index) radialLookup.erase(it++);
			else {
				if (it->second == chIndex) it->second = index; // Fix indices changed by switch
				it++;
			}
		}
		// Erase all points belonging to removed marker
		for (int b = 0; b < gridBuckets.size(); b++)
		{
			std::vector<uint16_t> *bucket = &gridBuckets[b];
			for (int m = 0; m < bucket->size(); m++)
			{
				if (bucket->at(m) == index)
				{
					bucket->at(m) = bucket->back();
					bucket->pop_back();
					m--;
				}
				else if (bucket->at(m) == chIndex)
					bucket->at(m) = index;
			}
		}
		wxLogMessage("  -> Removed (%d, %.04f)", index, errors[index]);
	}

//	wxLogMessage("Worst 5 markers left: (%d, %.04f), (%d, %.04f), (%d, %.04f), (%d, %.04f), (%d, %.04f)",
//	idx[worstCnt+0], errors[idx[worstCnt+0]], idx[worstCnt+1], errors[idx[worstCnt+1]], idx[worstCnt+2], errors[idx[worstCnt+2]], idx[worstCnt+3], errors[idx[worstCnt+3]], idx[worstCnt+4], errors[idx[worstCnt+4]]);

	wxLogMessage("Removed all markers with error higher than %.04f + %.04f = %.04f",
		errorAvg, errorBounds, errorAvg+errorBounds);
}