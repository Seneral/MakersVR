/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "control.hpp"
#include "config.hpp"

#include "wxbase.hpp" // wxLog*

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

// Intrinsic
static void InitCalibrationIntrinsic(ControlState *state);
static void FinalizeCalibrationIntrinsic(ControlState *state);
static void DiscardCalibrationIntrinsic(ControlState *state);
static void UpdateCalibrationIntrinsic(ControlState *state);
static void ThreadCalibrationIntrinsic(ControlState *state, CameraState *cam);

// Extrinsic
static void InitCalibrationExtrinsic(ControlState *state);
static void FinalizeCalibrationExtrinsic(ControlState *state);
static void DiscardCalibrationExtrinsic(ControlState *state);
static void UpdateCalibrationExtrinsic(ControlState *state);

// Room
static void InitCalibrationRoom(ControlState *state);
static void FinalizeCalibrationRoom(ControlState *state);
static void DiscardCalibrationRoom(ControlState *state);
static void UpdateCalibrationRoom(ControlState *state);

// Marker
static void InitCalibrationMarker(ControlState *state);
static void FinalizeCalibrationMarker(ControlState *state);
static void DiscardCalibrationMarker(ControlState *state);
static void UpdateCalibrationMarker(ControlState *state);

// General 3D and 2D Tracking
static void TrackMarkers3D(ControlState *state);
static void TrackMarkers2D(ControlState *state);
static void UndistortPoints(ControlState *state);

// + Exposed Phase control


// ----------------------------------------------------------------------------
// Intrinsic Calibration
// ----------------------------------------------------------------------------

static void InitCalibrationIntrinsic(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->intrinsic.calibrationStopped = false;
		cam->intrinsic.calibrationFinished = false;
		cam->intrinsic.calibrationRunning = false;
		cam->intrinsic.calibrationMarkerCount = 0;
		cam->intrinsic.selectionThreshold.reset();
		cam->intrinsic.maxMarkerCount.reset();
		// Setup radial lookup for radial control of marker selection
		cam->intrinsic.radialLookup.clear();
		cam->intrinsic.radialDensityGranularity.reset();
		cam->intrinsic.radialDensityTarget.reset();
		// Setup grid bucket for spacial control of marker selection
		cam->intrinsic.gridCountTarget.reset();
		cam->intrinsic.gridBuckets.resize(cam->intrinsic.gridSize*cam->intrinsic.gridSize);
		for (int i = 0; i < cam->intrinsic.gridBuckets.size(); i++)
		{
			cam->intrinsic.gridBuckets[i].clear();
			cam->intrinsic.gridBuckets[i].reserve(100);
		}
	}
}

static void FinalizeCalibrationIntrinsic(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->intrinsic.calibrationStopped = true;
	}

	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

//		if (cam->intrinsic.calibrationRunning && cam->intrinsic.calibrationThread->joinable())
//			cam->intrinsic.calibrationThread->join();
//		if (cam->intrinsic.calibrationRunning && cam->intrinsic.calibrationThread->joinable())
//			cam->intrinsic.calibrationThread->std::thread::~thread();
//		if (cam->intrinsic.calibrationThread != nullptr)
//			delete cam->intrinsic.calibrationThread;
		if (cam->intrinsic.calibrationRunning && cam->intrinsic.calibrationThread->joinable())
			cam->intrinsic.calibrationThread->detach();
		cam->intrinsic.calibrationThread = nullptr;
		cam->intrinsic.calibrationRunning = false;
		cam->intrinsic.calibrationFinished = false;

		wxLogMessage("Cam %d calibration: %d points, FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", m, (int)cam->intrinsic.calibrationSelection.size(), cam->camera.fovH, cam->camera.fovV, cam->camera.distortion.k1, cam->camera.distortion.k2, cam->camera.distortion.p1, cam->camera.distortion.p2, cam->camera.distortion.k3);
	}
}

static void DiscardCalibrationIntrinsic(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->intrinsic.radialLookup.clear();
		cam->intrinsic.gridBuckets.clear();
	}
}

static void UpdateCalibrationIntrinsic(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

		cam->intrinsic.markers.clear();
		cam->intrinsic.freeBlobs.clear();

		if (!isMarkerBuiltIn(state->calibration.markerTemplate2D->id))
		{ // Skip detection for testing markers (there's no detection algorithm for them)
			if (cam->points2D.size() >= state->calibration.markerTemplate2D->points.size())
			{ // Assume all points visible belong to singular marker and are kept ordered (not shuffled)
				cam->intrinsic.markers.push_back(Marker2D(state->calibration.markerTemplate2D->id, cam->points2D.size()));
				std::copy(cam->points2D.begin(), cam->points2D.end(), cam->intrinsic.markers[0].points.begin());
			}
		}
		else // Detect built-in markers
			findMarkerCandidates(cam->points2D, cam->pointSizes, cam->intrinsic.markers, cam->intrinsic.freeBlobs);
	}

	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

		// Calibration phase finalized, don't start new calibration rounds
		if (cam->intrinsic.calibrationStopped) continue;

		if (!cam->intrinsic.calibrationRunning)
		{ // No updates while calibration is running

			// Select markers for next calibration round
			int prevCnt = cam->intrinsic.calibrationSelection.size();
			bool calibReady = selectMarkersForCalibration(cam->camera, *state->calibration.markerTemplate2D, cam->intrinsic.markers, cam->intrinsic.calibrationSelection,
				cam->intrinsic.radialLookup, cam->intrinsic.radialDensityGranularity.get(), cam->intrinsic.radialDensityTarget.get(),
				cam->intrinsic.gridBuckets, cam->intrinsic.gridSize, cam->intrinsic.gridCountTarget.get(),
				cam->intrinsic.selectionThreshold.get(), m);
			int cnt = cam->intrinsic.calibrationSelection.size();
			//if (prevCnt == cnt) cam->intrinsic.selectionThreshold.current *= 0.8f;

			// Start calibration round
			if (cnt > 10 /*&& cnt >= cam->intrinsic.calibrationMarkerCount+10*/ && (calibReady || cnt >= cam->intrinsic.maxMarkerCount.get()))
			{
				wxLogMessage("Cam %d starting calibration with %d points!", m, cnt);

				// Start calibration thread
				cam->intrinsic.calibrationRunning = true;
				cam->intrinsic.calibrationFinished = false;
				cam->intrinsic.calibrationMarkerCount = cnt;
				cam->intrinsic.calibrationThread = new std::thread(ThreadCalibrationIntrinsic, state, cam);
			}
		}
		else if (cam->intrinsic.calibrationFinished)
		{ // Finished last calibration round

			// Stop thread
			if (cam->intrinsic.calibrationThread->joinable())
				cam->intrinsic.calibrationThread->join();
			delete cam->intrinsic.calibrationThread;
			cam->intrinsic.calibrationThread = nullptr;
			cam->intrinsic.calibrationRunning = false;
			cam->intrinsic.calibrationFinished = false;

			wxLogMessage("Cam %d calibration: %d points, FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", m, (int)cam->intrinsic.calibrationSelection.size(), cam->camera.fovH, cam->camera.fovV, cam->camera.distortion.k1, cam->camera.distortion.k2, cam->camera.distortion.p1, cam->camera.distortion.p2, cam->camera.distortion.k3);

			// Filter out markers that had a high error
			int prevCnt = cam->intrinsic.calibrationSelection.size();
			filterMarkersForCalibration(cam->camera, cam->intrinsic.calibrationMarkerErrors, cam->intrinsic.calibrationSelection, cam->intrinsic.radialLookup, cam->intrinsic.gridBuckets);
			int worstCnt = prevCnt - cam->intrinsic.calibrationSelection.size();
			wxLogMessage("Cam %d worst %d markers removed:", m, worstCnt);

			// Adapt parameters for next round
			cam->intrinsic.selectionThreshold.iterate();
			cam->intrinsic.maxMarkerCount.iterate();
			cam->intrinsic.radialDensityGranularity.iterate();
			cam->intrinsic.radialDensityTarget.iterate();
			cam->intrinsic.gridCountTarget.iterate();
		}
	}
}

static void ThreadCalibrationIntrinsic(ControlState *state, CameraState *cam)
{
	cam->intrinsic.calibrationRunning = true;
	cam->intrinsic.calibrationFinished = false;
	calculateIntrinsicCalibration(cam->camera, cam->intrinsic.calibrationSelection, *state->calibration.markerTemplate2D, cam->intrinsic.calibrationMarkerErrors);
	cam->intrinsic.calibrationFinished = true;
}


// ----------------------------------------------------------------------------
// Extrinsic Calibration
// ----------------------------------------------------------------------------

static void InitCalibrationExtrinsic(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
		state->cameras[m].extrinsic.relations.resize(state->cameras.size());
	
	// Reserve for maximum amount of possible relations, to preserve pointers
	// This estimate of the binominal coefficient (n^k / k!) is excellent for small k (here k=2)
	state->calibration.relations.reserve(state->cameras.size() * state->cameras.size() / 2);
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *camA = &state->cameras[m];
		for (int n = m+1; n < state->cameras.size(); n++)
		{
			CameraState *camB = &state->cameras[n];
			// Register relation
			state->calibration.relations.push_back({});
			CameraRelation *rel = &state->calibration.relations[state->calibration.relations.size()-1];
			// Set camera indices
			rel->camA = m;
			rel->camB = n;
			// Enter as relations
			camA->extrinsic.relations[n] = rel;
			camB->extrinsic.relations[m] = rel;
		}
	}
}

static void FinalizeCalibrationExtrinsic(ControlState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		rel->sample = chooseBestTransformCandidate(rel->candidates);
		rel->candidates.clear();
		wxLogMessage("Cam Rel %d-%d weight: %.4f", rel->camA, rel->camB, rel->sample.weight);
	}
}

static void DiscardCalibrationExtrinsic(ControlState *state)
{
	state->calibration.relations.clear();
	for (int m = 0; m < state->cameras.size(); m++)
		state->cameras[m].extrinsic.relations.clear();
}

static void UpdateCalibrationExtrinsic(ControlState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		CameraState *camA = &state->cameras[rel->camA];
		CameraState *camB = &state->cameras[rel->camB];

		for (int a = 0; a < camA->extrinsic.poses.size(); a++)
		{
			if (camA->extrinsic.posesMSE[a] > MAX_POSE_MSE) continue;
			const Eigen::Isometry3f poseA = camA->extrinsic.poses[a];
			for (int b = 0; b < camB->extrinsic.poses.size(); b++)
			{
				if (camB->extrinsic.posesMSE[b] > MAX_POSE_MSE) continue;
				// Datapoint (Transformation from camera a to camera b)
				const Eigen::Isometry3f poseB = camB->extrinsic.poses[b];
				Eigen::Isometry3f transAB = poseA * poseB.inverse();
				// Weight of datapoint
				float distA = poseA.translation().norm(), rotA = std::abs(Eigen::AngleAxisf(poseA.rotation()).angle())/PI*2;
				float distB = poseB.translation().norm(), rotB = std::abs(Eigen::AngleAxisf(poseB.rotation()).angle())/PI*2;
				float weight = distA * (1+rotA) + distB * (1+rotB);
				weight = 1000.0f/(weight*weight);
				// Add to suitable candidate
				int candidateInd = AddTransformToCandidates(rel->candidates, transAB, weight, 20, 2);

				{ // Debug
					// Calculate error between data point / candidate and ground truth
					Eigen::Isometry3f transABGT = camA->camera.transform.inverse() * camB->camera.transform;
					TransformCandidate *candidate = &rel->candidates[candidateInd];
					std::pair<float,float> poseError = calculatePoseError(transABGT, transAB);
					std::pair<float,float> candError = calculatePoseError(transABGT, candidate->transform);
					// Debug
					wxLogMessage("Cam Rel %d-%d: (%.4fmm, %.4f\u00B0) => (%.4fmm, %.4f\u00B0, %d, %d, %.4f)", rel->camA, rel->camB, poseError.first*10, poseError.second, candError.first*10, candError.second, candidateInd, (int)candidate->datapoints.size(), candidate->weight);
				}


			}	
		}
	}
}


// ----------------------------------------------------------------------------
// Room Calibration
// ----------------------------------------------------------------------------

static void InitCalibrationRoom(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
		state->cameras[m].extrinsic.originCandidates.clear();
}

static void FinalizeCalibrationRoom(ControlState *state)
{
	// Pick best origin candidates and compile relations and origin samples
	std::vector<TransformSample*> origins (state->cameras.size());
	std::vector<std::vector<TransformSample*>> relations(state->cameras.size());
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->extrinsic.origin = chooseBestTransformCandidate(cam->extrinsic.originCandidates);
		cam->extrinsic.originCandidates.clear();
		origins[m] = &cam->extrinsic.origin;
		relations[m].resize(cam->extrinsic.relations.size());
		wxLogMessage("Cam %d origin weight: %.4f", m, cam->extrinsic.origin.weight);
	}
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		relations[rel->camA][rel->camB] = &rel->sample;
		relations[rel->camB][rel->camA] = &rel->sample;
		wxLogMessage("Cam Rel %d-%d weight: %.4f", rel->camA, rel->camB, rel->sample.weight);
	}

	// Calculate camera transforms using origin and relation samples
	std::vector<Eigen::Isometry3f> cameraTransforms;
	calculateCameraTransforms(origins, relations, cameraTransforms);
	for (int m = 0; m < state->cameras.size(); m++)
		state->cameras[m].camera.transform = cameraTransforms[m];

	// Clear
	state->calibration.relations.clear();
	for (int m = 0; m < state->cameras.size(); m++)
		state->cameras[m].extrinsic.relations.clear();

	for (int m = 0; m < state->cameras.size(); m++)
	{ // Debug
		Eigen::Isometry3f cameraTransform = cameraTransforms[m];
		Eigen::Isometry3f trueTransform = state->cameras[m].testing.camera.transform;
		Eigen::Vector3f tDiff = cameraTransform.translation() - trueTransform.translation();
		Eigen::Matrix3f rDiff = trueTransform.rotation() * cameraTransform.rotation().transpose();
		float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
		wxLogMessage("Camera %d error: (%.4fmm, %.4f\u00B0)", m, tError*10, rError);
	}
}

static void DiscardCalibrationRoom(ControlState *state)
{
	state->calibration.relations.clear();
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->extrinsic.relations.clear();
		cam->extrinsic.originCandidates.clear();
	}
}

static void UpdateCalibrationRoom(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		for (int p = 0; p < cam->extrinsic.poses.size(); p++)
		{
			if (cam->extrinsic.posesMSE[p] > MAX_POSE_MSE) continue;
			// Datapoint (Transformation from camera to origin)
			const Eigen::Isometry3f pose = cam->extrinsic.poses[p];
			// Weight of datapoint
			float dist = pose.translation().norm(), rot = std::abs(Eigen::AngleAxisf(pose.rotation()).angle())/PI*2;
			float weight = dist * (1+rot);
			weight = 1000.0f/(weight*weight);
			// Add to suitable candidate
			int candidateInd = AddTransformToCandidates(cam->extrinsic.originCandidates, pose, weight, 5, 1);
			
			{ // Debug
				// Calculate error between data point / candidate and ground truth
				TransformCandidate *candidate = &cam->extrinsic.originCandidates[candidateInd];
				std::pair<float,float> poseError = calculatePoseError(cam->camera.transform.inverse(), cam->extrinsic.poses[p]);
				std::pair<float,float> candError = calculatePoseError(cam->camera.transform.inverse(), candidate->transform);
				// Debug
				wxLogMessage("Cam %d origin: (%.4fmm, %.4f\u00B0) => (%.4fmm, %.4f\u00B0, %d, %d, %.4f)", m, poseError.first*10, poseError.second, candError.first*10, candError.second, candidateInd, (int)candidate->datapoints.size(), candidate->weight);
			}
		}
	}
}


// ----------------------------------------------------------------------------
// Marker Calibration
// ----------------------------------------------------------------------------

static void InitCalibrationMarker(ControlState *state)
{
	state->markerCalib.iteration = 0;
	state->markerCalib.iterativeMarker.points.clear();
	state->markerCalib.iterativeMarker.pointRelation.clear();
	state->markerCalib.iterativeMarker.relationDist.clear();
	state->markerCalib.markerPoints.clear();
	state->markerCalib.markerPointRating.clear();
}

static void FinalizeCalibrationMarker(ControlState *state)
{
	// Remove points with lower confidence
	float maxConfidence = std::max_element(state->markerCalib.markerPointRating.begin(), state->markerCalib.markerPointRating.end(), [](auto &a, auto &b){ return a.first < b.first; })->first;
	for (int i = 0; i < state->markerCalib.markerPoints.size(); i++)
	{
		auto &ptRating = state->markerCalib.markerPointRating[i];
		if (ptRating.first < maxConfidence/10.0f)
		{ // Confidence lower than 1/10 of best point
			wxLogMessage("Removed old point %d with confidence %.4f < %.4f!", i, ptRating.first, maxConfidence/10.0f);
			state->markerCalib.markerPoints[i] = state->markerCalib.markerPoints.back();
			state->markerCalib.markerPoints.pop_back();
			state->markerCalib.markerPointRating[i] = state->markerCalib.markerPointRating.back();
			state->markerCalib.markerPointRating.pop_back();
		}
	}

	// Center marker around center of mass
	Eigen::Vector3f COM = Eigen::Vector3f::Zero();
	for (int i = 0; i < state->markerCalib.markerPoints.size(); i++)
		COM += state->markerCalib.markerPoints[i] / state->markerCalib.markerPoints.size();
	for (int i = 0; i < state->markerCalib.markerPoints.size(); i++)
		state->markerCalib.markerPoints[i] -= COM;

	// TODO: Proper identity transform calibration step

	// Find max occupied marker id (excluding testing markers because they are negative)
	int maxID = -1;
	for (int i = 0; i < state->tracking.markerTemplates3D.size(); i++)
		maxID = std::max(maxID, state->tracking.markerTemplates3D[i].id);

	// Add marker with new ID to templates
	state->tracking.markerTemplates3D.push_back({});
	MarkerTemplate3D *marker = &state->tracking.markerTemplates3D[state->tracking.markerTemplates3D.size()-1];
	marker->id = maxID+1;
	marker->label = std::string("Marker ID ") + (char)((int)'0' + marker->id);
	marker->points.resize(state->markerCalib.markerPoints.size());
	std::copy(state->markerCalib.markerPoints.begin(), state->markerCalib.markerPoints.end(), marker->points.begin());
	generateLookupTables(marker);
}

static void DiscardCalibrationMarker(ControlState *state)
{
	state->markerCalib.iterativeMarker.points.clear();
	state->markerCalib.iterativeMarker.pointRelation.clear();
	state->markerCalib.iterativeMarker.relationDist.clear();
	state->markerCalib.markerPoints.clear();
	state->markerCalib.markerPointRating.clear();
}

static void UpdateCalibrationMarker(ControlState *state)
{
	// Create 3D Rays for triangulation assuming calibrated camera
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->tracking.rays3D.clear();
		castRays(cam->undistorted2D, cam->camera, cam->tracking.rays3D);
	}

	// Perform triangulation using 3D Rays
	std::vector<std::vector<Ray>*> rayGroups;
	for (int m = 0; m < state->cameras.size(); m++)
		rayGroups.push_back(&state->cameras[m].tracking.rays3D);
	state->tracking.points3D.clear();
	state->tracking.conflicts.clear();
	state->tracking.nonconflictedCount = triangulateRayIntersections(rayGroups, state->tracking.points3D, state->tracking.conflicts, state->tracking.maxIntersectError, state->tracking.minIntersectError);

	static Eigen::Isometry3f originalGT;

	auto updateIterativeMarkerTemplate = [&state]()
	{
		state->markerCalib.iterativeMarker.id = -1;
		state->markerCalib.iterativeMarker.points.resize(state->markerCalib.markerPoints.size());
		std::copy(state->markerCalib.markerPoints.begin(), state->markerCalib.markerPoints.end(), state->markerCalib.iterativeMarker.points.begin());
		generateLookupTables(&state->markerCalib.iterativeMarker);
	};

	if (state->markerCalib.markerPoints.size() == 0)
	{ // Add initial marker points
		if (state->tracking.nonconflictedCount > 4)
		{
			state->markerCalib.markerPoints.reserve(state->tracking.points3D.size());
			state->markerCalib.markerPointRating.reserve(state->tracking.points3D.size());
			for (int i = 0; i < state->tracking.points3D.size(); i++)
			{
				state->markerCalib.markerPoints.push_back(state->tracking.points3D[i].pos);
				state->markerCalib.markerPointRating.push_back({ state->tracking.points3D[i].confidence/state->tracking.points3D[i].error, 0 });
			}
			updateIterativeMarkerTemplate();
			originalGT = state->testing.GT;
		}
	}
	else
	{ // Iteratively add and improve marker points

		// Detect match using iterative marker template
		std::vector<MarkerCandidate3D> candidates;
		detectMarker3D(&state->markerCalib.iterativeMarker, state->tracking.points3D, state->tracking.conflicts, state->tracking.nonconflictedCount, candidates, state->tracking.sigmaError*2); // Double error because both point clouds introduce uncertainty

		// Get the best candidate and make sure it's a good match
		std::tuple<int,float,int> bestCand = getBestMarkerCandidate(&state->markerCalib.iterativeMarker, state->tracking.points3D, state->tracking.conflicts, state->tracking.nonconflictedCount, candidates);
		int bestCandIndex = std::get<0>(bestCand);
		MarkerCandidate3D *candidate = &candidates[bestCandIndex];
		int bestCandCount = std::get<2>(bestCand);
		int bestCandPtCnt = candidate->points.size();
		float bestCandMSE = std::get<1>(bestCand);
		if (bestCandPtCnt < 4)
		{
			wxLogMessage("Best candidate has only %d points!", bestCandPtCnt);
			return;
		}
		if (bestCandCount > 1)
		{
			wxLogMessage("Found %d best markers with %d points!", bestCandCount, bestCandPtCnt);
			return;
		}

		// Log
		wxLogMessage("Iteration %d: %d marker points; %d matches; Best: %d points, MSE %.4fmpx", state->markerCalib.iteration, (int)state->markerCalib.markerPoints.size(), (int)candidates.size(), (int)candidates[bestCandIndex].points.size(), bestCandMSE*1000);
		if (state->testing.isTesting)
		{ // Debug
			auto poseError = calculatePoseError(candidate->pose * originalGT, state->testing.GT);
			wxLogMessage("Ground truth error (%.4fmm, %.4f\u00B0)", poseError.first*10, poseError.second);
		}

		auto mergeMarkerPoints = [&state](MarkerCandidate3D *candidate, int mkPt, const TriangulatedPoint &matchPt)
		{
			float matchConfidence = matchPt.confidence / matchPt.error;
			state->markerCalib.markerPointRating[mkPt].first += matchConfidence;
			float lerp = matchConfidence / state->markerCalib.markerPointRating[mkPt].first;
			Eigen::Vector3f trPos = candidate->pose.inverse() * matchPt.pos;
			Eigen::Vector3f diff = trPos - state->markerCalib.markerPoints[mkPt];
			state->markerCalib.markerPoints[mkPt] += diff * lerp;
		};

		// Add new points and improve existing matched marker points
		state->markerCalib.iteration++;
		int mergeCnt = 0;
		for (int i = 0; i < candidate->pointMap.size(); i++)
		{
			int mkPt = candidate->pointMap[i];
			if (mkPt == -1)
			{ // No match for point
				if (i < state->tracking.nonconflictedCount)
				{ // Add only unconflicted points to marker
					// Check if any points are closeby which were missed
					float closestPoint = 9999.9f;
					int closestIndex = -1;
					for (int j = 0; j < state->markerCalib.markerPoints.size(); j++)
					{
						float dist = (state->markerCalib.markerPoints[j] - candidate->pose.inverse() * state->tracking.points3D[i].pos).norm();
						if (dist < closestPoint)
						{
							closestPoint = dist;
							closestIndex = j;
						}
					}
					if (closestPoint < 0.5f) // 5mm
						wxLogMessage("Found unconflicted point, but closest marker point was only %.4fcm away!", closestPoint);
					else
					{
						state->markerCalib.markerPoints.push_back(candidate->pose.inverse() * state->tracking.points3D[i].pos);
						state->markerCalib.markerPointRating.push_back({ state->tracking.points3D[i].confidence / state->tracking.points3D[i].error, state->markerCalib.iteration });
						wxLogMessage("Found new unconflicted point, total of %d points now - closest point was %.4fcm!", (int)state->markerCalib.markerPoints.size(), closestPoint);
					}
				}
			}
			else
			{ // Reinforce matched points
				mergeMarkerPoints(candidate, mkPt, state->tracking.points3D[i]);
				mergeCnt++;
			}
		}
		wxLogMessage("Reinforced %d points!", mergeCnt);

		// Remove points with lower confidence while giving new points that are consistent a chance to grow confidence
		float maxConfidence = std::max_element(state->markerCalib.markerPointRating.begin(), state->markerCalib.markerPointRating.end(), [](auto &a, auto &b){ return a.first < b.first; })->first;
		for (int i = 0; i < state->markerCalib.markerPoints.size(); i++)
		{
			auto &ptRating = state->markerCalib.markerPointRating[i];
			if (ptRating.first < maxConfidence/10.0f)
			{ // Confidence lower than 1/10 of best point
				float avg = ptRating.first / (state->markerCalib.iteration - ptRating.second);
				float bestAvg = maxConfidence/state->markerCalib.iteration;
				if (avg < bestAvg/2)
				{ // Average confidence since first appearance not significant compared to (approx) average confidence of best point 
					wxLogMessage("Removed old point %d with confidence %.4f < %.4f and average %.4f < %.4f!", i, ptRating.first, maxConfidence/10.0f, avg, bestAvg/2);
					state->markerCalib.markerPoints[i] = state->markerCalib.markerPoints.back();
					state->markerCalib.markerPoints.pop_back();
					state->markerCalib.markerPointRating[i] = state->markerCalib.markerPointRating.back();
					state->markerCalib.markerPointRating.pop_back();
				}
			}
		}

		updateIterativeMarkerTemplate();

		if (state->testing.isTesting)
		{ // Debug

			// Set ground truth marker as point cloud
			std::vector<TriangulatedPoint> pointCloud;
			pointCloud.reserve(state->testing.markerTemplate3D->points.size());
			for (int i = 0; i < state->testing.markerTemplate3D->points.size(); i++)
				pointCloud.push_back({  state->testing.markerTemplate3D->points[i].pos, state->tracking.maxIntersectError, 1 });

			// Detect match using iterative marker template
			std::vector<MarkerCandidate3D> checkCandidates;
			detectMarker3D(&state->markerCalib.iterativeMarker, pointCloud, {}, pointCloud.size(), checkCandidates, state->tracking.sigmaError*2, false);

			// Pick best match
			std::tuple<int,float,int> checkCand = getBestMarkerCandidate(&state->markerCalib.iterativeMarker, pointCloud, {}, pointCloud.size(), checkCandidates);
			MarkerCandidate3D *checkCandidate = &checkCandidates[std::get<0>(checkCand)];
			int checkCandPtCnt = checkCandidate->points.size();

			// Log match with actual marker
			auto poseError = calculatePoseError(checkCandidate->pose, originalGT);
			wxLogMessage("Marker points: %d candidates; %d matched, %d missing, %d excess; MSE %.4fmpx, Error (%.4fmm, %.4f\u00B0)", (int)checkCandidates.size(), checkCandPtCnt, (int)(pointCloud.size() - checkCandPtCnt), (int)(state->markerCalib.markerPoints.size() - checkCandPtCnt), std::get<1>(checkCand)*1000, poseError.first*10, poseError.second);

			/*for (int i = 0; i < pointCloud.size(); i++)
			{
				if (checkCandidate->pointMap[i] < 0) continue;
				float error = (pointCloud[i].pos - checkCandidate->pose * markerTemplate->points[checkCandidate->pointMap[i]]).norm();
				wxLogMessage("Mapping %d -> %d with error %.2fmm", i, checkCandidate->pointMap[i], error*10);
			}*/
		}
	}
}


// ----------------------------------------------------------------------------
// General 3D- and 2D Marker Tracking
// ----------------------------------------------------------------------------

static void TrackMarkers3D(ControlState *state)
{
	// Create 3D Rays for triangulation assuming calibrated camera
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->tracking.rays3D.clear();
		castRays(cam->undistorted2D, cam->camera, cam->tracking.rays3D);
	}

	// Perform triangulation using 3D Rays
	std::vector<std::vector<Ray>*> rayGroups;
	for (int m = 0; m < state->cameras.size(); m++)
		rayGroups.push_back(&state->cameras[m].tracking.rays3D);
	state->tracking.points3D.clear();
	state->tracking.conflicts.clear();
	state->tracking.nonconflictedCount = triangulateRayIntersections(rayGroups, state->tracking.points3D, state->tracking.conflicts, state->tracking.maxIntersectError, state->tracking.minIntersectError);

	// TODO: Split point cloud into groups and assign marker templates expected in that point cloud

	// Detect markers in triangulated point cloud
	state->tracking.poses3D.clear();
	state->tracking.posesMSE.clear();
	auto markerTemplate = std::find_if(state->tracking.markerTemplates3D.begin(), state->tracking.markerTemplates3D.end(), [state](auto &m){ return m.id == state->tracking.trackID; });
	if (markerTemplate != state->tracking.markerTemplates3D.end())
	{ // TODO: Replace with expected marker after initial detection
		auto pose = detectMarker3D(markerTemplate._Ptr, state->tracking.points3D, state->tracking.conflicts, state->tracking.nonconflictedCount, state->tracking.sigmaError);
		if (std::get<2>(pose) > 0)
		{ // Found a pose
			state->tracking.poses3D.push_back(std::get<0>(pose));
			state->tracking.posesMSE.push_back({ std::get<1>(pose), std::get<2>(pose) });
		}
	}
}

static void TrackMarkers2D(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

		// Single camera pose estimation using OpenCV
		cam->extrinsic.lastPoses.swap(cam->extrinsic.poses);
		cam->extrinsic.poses.clear();
		cam->extrinsic.posesMSE.clear();
		cam->extrinsic.posesMatch.clear();
		cam->extrinsic.markers.clear();
		cam->extrinsic.freeBlobs.clear();

		if (!isMarkerBuiltIn(state->calibration.markerTemplate2D->id))
		{ // Skip detection for testing markers (there's no detection algorithm for them)
			if (cam->undistorted2D.size() >= state->calibration.markerTemplate2D->points.size())
			{ // Assume all points visible belong to singular marker and are kept ordered (not shuffled)
				cam->extrinsic.markers.push_back(Marker2D(state->calibration.markerTemplate2D->id, cam->undistorted2D.size()));
				std::copy(cam->undistorted2D.begin(), cam->undistorted2D.end(), cam->extrinsic.markers[0].points.begin());
			}
		}
		else // Detect built-in markers
			findMarkerCandidates(cam->undistorted2D, cam->pointSizes, cam->extrinsic.markers, cam->extrinsic.freeBlobs);
		cam->extrinsic.posesMSE.reserve(cam->extrinsic.markers.size());
		cam->extrinsic.poses.reserve(cam->extrinsic.markers.size());

		// Infer pose of detected markers
		for (int i = 0; i < cam->extrinsic.markers.size(); i++)
		{
			Eigen::Isometry3f pose = inferMarkerPose(cam->camera, cam->extrinsic.markers[i], *state->calibration.markerTemplate2D);
			float MSE = calculateMSE(pose, cam->camera, cam->extrinsic.markers[i], *state->calibration.markerTemplate2D);
			cam->extrinsic.poses.push_back(pose);
			cam->extrinsic.posesMSE.push_back(MSE);
		}

		// Match markers from last frame
		matchTrackedPoses(cam->extrinsic.poses, cam->extrinsic.lastPoses, cam->extrinsic.poseDiffPos, cam->extrinsic.poseDiffRot, cam->extrinsic.posesMatch);

		// Detect mirrored poses (due to inaccuracies) using last frames poses
		for (int p = 0; p < cam->extrinsic.poses.size(); p++)
		{
			if (cam->extrinsic.posesMatch[p] != -1)
			{
				// Last frames rotation
				int l = cam->extrinsic.posesMatch[p];
				float expectedAngleDiff = cam->extrinsic.poseDiffRot[l];
				if (expectedAngleDiff == 0) expectedAngleDiff = 10; // New pose in last frame
				Eigen::Matrix3f lastRot = cam->extrinsic.lastPoses[l].rotation();// + cam->extrinsic.poseAngularMovement[i];
				// Current frames rotation
				Eigen::Matrix3f curRot = cam->extrinsic.poses[p].rotation();
				float curAngleDiff = Eigen::AngleAxisf(curRot * lastRot.transpose()).angle();
				if (curAngleDiff <= 2*expectedAngleDiff) continue;
				// Test flip and compare
				Eigen::Vector3f euler = getEulerXYZ(curRot);
				Eigen::Matrix3f flippedRot = getRotationXYZ(Eigen::Vector3f(-euler.x(), -euler.y(), euler.z()));
				float flippedAngleDiff = Eigen::AngleAxisf(flippedRot * lastRot.transpose()).angle();
				if (flippedAngleDiff < 2.0f*expectedAngleDiff || flippedAngleDiff*1.5 < curAngleDiff)
				{ // Flip rotation, because it conforms significantly more with expected rotation
					Eigen::Isometry3f flippedPose = cam->extrinsic.poses[p];
					flippedPose.linear() = flippedRot;
					float flippedMSE = calculateMSE(flippedPose, cam->camera, cam->extrinsic.markers[p], *state->calibration.markerTemplate2D);
					cam->extrinsic.poses[p] = flippedPose;
					cam->extrinsic.posesMSE[p] = flippedMSE*10;
//					wxLogMessage("Cam %d, Pose %d (%d):  Flipped, new MSE: %f! Diffs exp: %f, act: %f, flp: %f", m, p, l, flippedMSE, expectedAngleDiff/PI*180, curAngleDiff/PI*180, flippedAngleDiff/PI*180);
				}
			}
		}

		// Ignore rotation of inaccurate poses
		for (int p = 0; p < cam->extrinsic.poses.size(); p++)
		{
			if (cam->extrinsic.posesMSE[p] > MAX_POSE_MSE)
			{
				wxLogMessage("Cam %d: Discarded pose, MSE %.4fmpx", m, cam->extrinsic.posesMSE[p]*1000);
				int l = cam->extrinsic.posesMatch[p];
				if (l != -1) cam->extrinsic.poses[p].linear() = cam->extrinsic.lastPoses[l].rotation();
			}
		}

		// Update temporal information using matching
		matchAccept(cam->extrinsic.poses, cam->extrinsic.lastPoses, cam->extrinsic.poseDiffPos, cam->extrinsic.poseDiffRot, cam->extrinsic.posesMatch);
	}
}

static void UndistortPoints(ControlState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->undistorted2D.clear();
		cam->undistorted2D.reserve(cam->points2D.size());
		for (int i = 0; i < cam->points2D.size(); i++)
			cam->undistorted2D.push_back(undistortPoint(cam->camera, cam->points2D[i]));
	}
}


// ----------------------------------------------------------------------------
// Phase Control
// ----------------------------------------------------------------------------

/**
 * Finalize the current phase and return to the idle phase
 */
void FinalizePhase(ControlState *state)
{
	if (state->phase == PHASE_None || state->phase == PHASE_Idle)
		return;

	// Finalize current phase
	bool writeCalibration = false;
	if (state->phase == PHASE_Calibration_Intrinsic)
	{
		FinalizeCalibrationIntrinsic(state);
		writeCalibration = true;
	}
	else if (state->phase == PHASE_Calibration_Extrinsic)
	{
		FinalizeCalibrationExtrinsic(state);
	}
	else if (state->phase == PHASE_Calibration_Room)
	{
		FinalizeCalibrationRoom(state);
		writeCalibration = true;
	}
	else if (state->phase == PHASE_Calibration_Marker)
	{
		FinalizeCalibrationMarker(state);
		writeCalibration = true;
	}

	// Write results of current phase
	if (writeCalibration)
	{
		std::vector<Camera> cameraCalibrations(state->cameras.size());
		for (int i = 0; i < state->cameras.size(); i++)
			cameraCalibrations[i] = state->cameras[i].camera;
		std::vector<MarkerTemplate3D> markerTemplates;
		markerTemplates.reserve(state->tracking.markerTemplates3D.size());
		for (int i = 0; i < state->tracking.markerTemplates3D.size(); i++)
			if (state->tracking.markerTemplates3D[i].id >= 0) // TODO: Differentiate Testing markers some better way that allows saving of calibrated testing markers
				markerTemplates.push_back(state->tracking.markerTemplates3D[i]);
		writeCalibrationFile("config/calib.json", cameraCalibrations, markerTemplates);
	}

	// Return to idle phase
	state->lastPhase = state->phase;
	state->phase = PHASE_Idle;
}

/**
 * Discard the current phase and return to the idle phase
 */
void DiscardPhase(ControlState *state)
{
	if (state->phase == PHASE_None || state->phase == PHASE_Idle)
		return;

	// Init next phase
	if (state->phase == PHASE_Calibration_Intrinsic)
		DiscardCalibrationIntrinsic(state);
	else if (state->phase == PHASE_Calibration_Extrinsic)
		DiscardCalibrationExtrinsic(state);
	else if (state->phase == PHASE_Calibration_Room)
		DiscardCalibrationRoom(state);
	else if (state->phase == PHASE_Calibration_Marker)
		DiscardCalibrationMarker(state);

	// Return to idle phase
	state->lastPhase = state->phase;
	state->phase = PHASE_Idle;
}

/**
 * Enters the specified phase from the idle phase
 */
void EnterPhase(ControlState *state, ControlPhase phase)
{
	if (state->phase != PHASE_Idle)
		return;

	// Enter phase
	state->lastPhase = state->phase;
	state->phase = phase;

	// Init phase
	if (phase == PHASE_Calibration_Intrinsic)
		InitCalibrationIntrinsic(state);
	else if (phase == PHASE_Calibration_Extrinsic)
		InitCalibrationExtrinsic(state);
	else if (phase == PHASE_Calibration_Room)
		InitCalibrationRoom(state);
	else if (phase == PHASE_Calibration_Marker)
		InitCalibrationMarker(state);
}

/**
 * Enters the next phase, discarding any unfinalized current phases
 */
void NextPhase(ControlState *state)
{
	if (state->lastPhase == PHASE_Tracking)
		return;
	if (state->phase != PHASE_Idle)
		return;
	ControlPhase targetPhase = (ControlPhase)((int)state->lastPhase + 1);
	// Skip room calibration phase if extrinsic phase was discarded (either here or explicitly)
	if (state->lastPhase == PHASE_Calibration_Extrinsic && state->calibration.relations.size() == 0)
		targetPhase = (ControlPhase)((int)state->lastPhase + 2);
	EnterPhase(state, targetPhase);
}

/**
 * Handles a new frame
 */
void HandleCameraState(ControlState *state)
{
	if (state->phase == PHASE_Tracking)
	{
		// Undistort points
		UndistortPoints(state);

		// Triangulate and detect markers
		TrackMarkers3D(state);
	}
	else if (state->phase == PHASE_Calibration_Intrinsic)
	{
		// Add to calibration list of good candidate, update calibration values
		UpdateCalibrationIntrinsic(state);
	}
	else if (state->phase == PHASE_Calibration_Extrinsic || state->phase == PHASE_Calibration_Room)
	{
		// Undistort points using calibrated camera
		UndistortPoints(state);

		// Find calibration marker, match to previous frame and evaluate accuracy/error
		TrackMarkers2D(state);

		if (state->phase == PHASE_Calibration_Extrinsic)
		{ // Use pose to establish relations between cameras to figure out relative transforms
			UpdateCalibrationExtrinsic(state);
		}
		else if (state->phase == PHASE_Calibration_Room)
		{ // Establish similar relations from each camera to the room origin (or later, floor)
			UpdateCalibrationRoom(state);
		}
	}
	else if (state->phase == PHASE_Calibration_Marker)
	{
		// Undistort points using calibrated camera
		UndistortPoints(state);

		// Triangulate point cloud and improve current marker template
		UpdateCalibrationMarker(state);
	}
}