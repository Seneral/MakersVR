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

DefMarker calibMarker3D;

#ifdef USE_CV
std::vector<cv::Point3f> cv_marker3DTemplate;
#endif


/* Functions */

/**
 * Initialize resources for calibration
 */
void initCalibration()
{
}

/**
 * Cleanup of resources
 */
void cleanCalibration()
{

}

/**
 * Finds all (potential) marker and also returns all free (unassociated) blobs left (Deprecated)
 */
void findMarkerCandidates(const std::vector<Eigen::Vector2f> &points2D, const std::vector<float> &pointSizes, std::vector<Marker> &markers2D, std::vector<int> &freePoints2D)
{
	static std::vector<MarkerCandidate> markerCandidates;
	static std::bitset<128> markerTag;

	// Find and extract all markers candidates (base line of 3 dots plus header)
	// Step 1: Find all base lines (3 points in a straight line with equal distance)
	// Step 2: For each line, find the leading point on one side
	markerCandidates.clear();
	for (uint8_t c = 0; c < points2D.size(); c++)
	{

		for (uint8_t a = 0; a < points2D.size(); a++)
		{
			if (a == c) continue;

			// Check size difference
			float sizeMin = std::min(pointSizes[c], pointSizes[a]);
			float sizeMax = std::max(pointSizes[c], pointSizes[a]);
			if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

			for (uint8_t b = a+1; b < points2D.size(); b++)
			{
				if (b == c) continue;

				// Check size difference
				sizeMin = std::min(sizeMin, pointSizes[b]);
				sizeMax = std::max(sizeMax, pointSizes[b]);
				if (sizeMin * MAX_SIZE_DIFF < sizeMax) continue;

				// Get point positions
				Eigen::Vector2f ptC = points2D[c];
				Eigen::Vector2f ptA = points2D[a];
				Eigen::Vector2f ptB = points2D[b];

				// Get base parameters
				Eigen::Vector2f base = ptA-ptB;
				float baseLen = base.norm();

				// Check if c is at the center of a and b, building a base line
				float centerDiff = (ptC*2 - ptA - ptB).norm();
				float centerError = centerDiff / baseLen;

				// if (std::abs(diffX) + std::abs(diffY) <= MAX_BASE_PX_DIST)
				if (centerError <= MAX_BASE_DIFF)
				{ // Found a base
					float baseAngle = std::atan2(base.y(), base.x()) / PI * 180;

					// Find closest blob as heading
					uint_fast8_t h = -1;
					float headLen = (float)1000000.0f;
					for (int i = 0; i < points2D.size(); i++)
					{
						if (i == c || i == a || i == b) continue;

						// Check size difference
						if (std::min(sizeMin, pointSizes[i]) * MAX_SIZE_DIFF < std::max(sizeMax, pointSizes[i])) continue;

						// Get Head parameters
						float hLen = (ptC-points2D[i]).norm();
						if (hLen < baseLen*10.0 && hLen < headLen)
						{ // Best head candidate so far
							headLen = hLen;
							h = i;
						}
					}
					if (h != -1)
					{
						Eigen::Vector2f ptH = points2D[h];

						// Get inner angle between base and head
						Eigen::Vector2f head = ptC-ptH;
						float innerDot = base.dot(head);
						float innerAngle = std::acos(innerDot / (baseLen * headLen)) / PI * 180 - 90;

						// Register marker
						// TODO: order a,b depending on which side h is using innerAngle
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
	markerTag.reset();
	markers2D.reserve(markers2D.size() + markerCandidates.size());
	for (int m = 0; m < markerCandidates.size(); m++)
	{
		MarkerCandidate *mc = &markerCandidates[m];
		if (markerTag[mc->c] || markerTag[mc->a] || markerTag[mc->b]) continue;

		// Set blobs as used
		markerTag.set(mc->a);
		markerTag.set(mc->b);
		markerTag.set(mc->c);
		markerTag.set(mc->h);

		markers2D.push_back(Marker({
			points2D[mc->c],
			points2D[mc->a],
			points2D[mc->b],
			points2D[mc->h],
		}));

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

/**
 * Infer the pose of a marker in camera space given its image points in pixel space and the intrinsically calibrated camera
 */
Eigen::Isometry3f inferMarkerPose(const Camera &camera, const Marker &marker2D)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	std::vector<cv::Point2f> cv_points2D;
	for (int i = 0; i < marker2D.pts.size(); i++)
		cv_points2D.push_back(cv::Point2f(marker2D.pts[i].x(), marker2D.pts[i].y()));

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
	cv::solvePnP(cv_marker3DTemplate, cv_points2D, cv_camMat, cv::noArray(), cv_rotation, cv_translation, false, cv::SOLVEPNP_ITERATIVE);

	// Convert from OpenCV to Blender/OpenGL space
	return interpretCVSpace(cv_translation, cv_rotation);
#endif
}

/**
 * Calculate mean squared error in image space of detected pose
 */
float calculateMSE(const Eigen::Isometry3f &pose3D, const Camera &camera, const Marker &marker2D)
{
	Eigen::Projective3f p = createProjectionMatrix(camera.fovH, camera.fovV);
	float mse = 0.0f;
	for (int i = 0; i < calibMarker3D.pts.size(); i++)
	{
		Eigen::Vector2f pt = projectPoint(p, pose3D * calibMarker3D.pts[i].pos).head<2>();
		pt = Eigen::Vector2f((pt.x()+1)/2*camera.width, (pt.y()+1)/2*camera.height);
		Eigen::Vector2f diffPx = marker2D.pts[i] - pt; // Convert difference to pixels
		mse += diffPx.squaredNorm();
	}
	return mse / calibMarker3D.pts.size();
}

/**
 * Gets the currently expected blob count of the current target
 */
int getExpectedBlobCount()
{
#ifdef USE_CV
	return cv_marker3DTemplate.size() < 4? 4 : cv_marker3DTemplate.size();
#else
	return 0;
#endif
}

/**
 * Adds transform to a candidate within error margin or creates a new one
 */
int AddTransformToCandidates (std::vector<TransformCandidate> &candidates, Eigen::Isometry3f transform, float weight, float maxTError, float maxRError)
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
	int candidate = -1;
	float weight = 0;
	for (int c = 0; c < candidates.size(); c++)
	{
		if (weight < candidates[c].weight)
		{
			candidate = c;
			weight = candidates[c].weight;
		}
	}
	// Output sample
	TransformSample sample;
	if (candidate != -1)
	{ // Apply candidate and analyze
		sample.transform = candidates[candidate].transform;
		sample.weight = candidates[candidate].weight;
		// TODO: Filter out outliers out of datapoints to improve reliability
		sample.stdDeviation = 0;
	}
	else
		sample.weight = 0;
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

	const int cnt = origins.size();
	const int len = cnt; // Can be set to limit path length for large camera setups
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
bool selectMarkersForCalibration(const Camera &camera, const std::vector<Marker> &markers, std::vector<Marker> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, float radialGranularity, float radialTarget, std::vector<std::vector<uint16_t>> &gridBuckets, int gridSize, int gridTarget, float *threshold, int m)
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
		const Marker *marker = &markers[p];

		// Determine value of adding marker to selection
		float radialValue = 0, gridValue = 0;
		for (int i = 0; i < marker->pts.size(); i++)
		{
			float r = Eigen::Vector2f((marker->pts[i].x()-w/2)/w*2, (marker->pts[i].y()-h/2)/w*2).norm();
			float density = calcMarkerDensity(r, radialGranularity);
			float scale = 10.0f/marker->pts.size();
			radialValue += std::max(0.0f, radialTarget-density) * scale;
			gridValue += std::max(0.0f, (float)gridTarget-getBucket(marker->pts[i])->size()) * scale;
		}

		// Determine if marker should be added
		if (radialValue < *threshold*radialTarget && gridValue < *threshold*gridTarget)
		{
			*threshold = 0.99f * *threshold;
			continue;
		}

		// Add marker
		int index = calibrationSelection.size();
		calibrationSelection.push_back(*marker);
		float rmin = 2, rmax = 0;
		for (int i = 0; i < marker->pts.size(); i++)
		{
			float r = Eigen::Vector2f((marker->pts[i].x()-w/2)/w*2, (marker->pts[i].y()-h/2)/w*2).norm();
			rmin = std::min(rmin, r);
			rmax = std::max(rmax, r);
			radialLookup.insert({ r, (uint16_t)index });
			getBucket(marker->pts[i])->push_back((uint16_t)index);
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
 * Calibrate using a range of detected markers
 */
float calculateIntrinsicCalibration(Camera &camera, const std::vector<Marker> &markers, std::vector<double> &errors)
{
#ifdef USE_CV
	// Create CV 2D array of marker points
	int ptCnt = cv_marker3DTemplate.size();
	int mkCnt = markers.size();
	std::vector<std::vector<cv::Point2f>> cv_calibPoints2D(mkCnt);
	std::vector<std::vector<cv::Point3f>> cv_markerPoints3D(mkCnt);
	for (int m = 0; m < markers.size(); m++)
	{
		if (markers[m].pts.size() != ptCnt) continue;
		cv_calibPoints2D[m].reserve(ptCnt);
		cv_markerPoints3D[m].reserve(ptCnt);
		for (int i = 0; i < ptCnt; i++)
		{
			cv_calibPoints2D[m].push_back(cv::Point2f(markers[m].pts[i].x(), markers[m].pts[i].y()));
			cv_markerPoints3D[m].push_back(cv_marker3DTemplate[i]);
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

	// Calibration
	double stdDev = cv::calibrateCameraRO(
		cv_markerPoints3D, cv_calibPoints2D,																// Calibration data
		cv::Size2i(camera.width, camera.height), -1, cv_camMat, cv_distort,									// Camera model
		cv_rotations, cv_translations, cv::noArray(),														// Output poses
		stdDev_Intrinsic, stdDev_Extrinsic, cv::noArray(), errors,											// Output diagnostics (std dev, errors)
		calibFlags, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON));	// Calibration model

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

	wxLogMessage("StdDev fx: %f, fy: %f, fovH: %f, fovV: %f, k1: %f, k2: %f, p1: %f, p2: %f, k3: %f",
		stdDev_Intrinsic[0], stdDev_Intrinsic[1], sdFovH, sdFovV,
		stdDev_Intrinsic[4], stdDev_Intrinsic[5], stdDev_Intrinsic[6], stdDev_Intrinsic[7], stdDev_Intrinsic[8]);

	return (float)stdDev;

#endif
	return 0;
}

/**
 * Takes the calibration selection and filters them using the errors from the last calibration round
 */
void filterMarkersForCalibration(const Camera &camera, const std::vector<double> &errors, std::vector<Marker> &calibrationSelection, std::multimap<float, uint16_t> &radialLookup, std::vector<std::vector<uint16_t>> &gridBuckets, int m)
{
	// Find the x worst fitting markers
	int cnt = calibrationSelection.size();
	int worstCnt = std::max(2, cnt/6);
	wxLogMessage("Cam %d worst %d markers removed:", m, worstCnt);

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
		// Switch with last element and remove it, keeps all relevant indices because we start deleting from the highest index
		calibrationSelection[idx[i]] = calibrationSelection.back();
		calibrationSelection.pop_back();
		int chIndex = calibrationSelection.size();
		// Erase all points belonging to removed marker
		auto it = radialLookup.begin();
		while (it != radialLookup.end())
		{
			if (it->second == idx[i]) radialLookup.erase(it++);
			else {
				if (it->second == chIndex) it->second = idx[i]; // Fix indices changed by switch
				it++;
			}
		}
		// Erase all points belonging to removed marker
		for (int b = 0; b < gridBuckets.size(); b++)
		{
			std::vector<uint16_t> *bucket = &gridBuckets[b];
			for (int m = 0; m < bucket->size(); m++)
			{
				if (bucket->at(m) == idx[i])
				{
					bucket->at(m) = bucket->back();
					bucket->pop_back();
					m--;
				}
				else if (bucket->at(m) == chIndex)
					bucket->at(m) = idx[i];
			}
		}
		wxLogMessage("  -> Removed (%d, %.04f)", idx[i], errors[idx[i]]);
	}

	wxLogMessage("Cam %d worst 5 markers left: (%d, %.04f), (%d, %.04f), (%d, %.04f), (%d, %.04f), (%d, %.04f)", m,
	idx[worstCnt+0], errors[idx[worstCnt+0]], idx[worstCnt+1], errors[idx[worstCnt+1]], idx[worstCnt+2], errors[idx[worstCnt+2]], idx[worstCnt+3], errors[idx[worstCnt+3]], idx[worstCnt+4], errors[idx[worstCnt+4]]);
}