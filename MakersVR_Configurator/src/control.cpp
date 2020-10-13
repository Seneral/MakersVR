/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "control.hpp"

#include "wxbase.hpp" // wxLog*

static void CalibrationThreadFunction(TrackingState *state, CameraState *cam);

static void InitCalibrationIntrinsic(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->intrinsic.calibrationStopped = false;
		cam->intrinsic.calibrationFinished = false;
		cam->intrinsic.calibrationRunning = false;
		cam->intrinsic.calibrationMarkerCount = 0;
		// Setup radial lookup for radial control of marker selection
		cam->intrinsic.markerRadialLookup.clear();
		cam->intrinsic.markerDensityTarget = 1.0f;
		cam->intrinsic.markerSelectionThreshold = 2.0f;
		cam->intrinsic.markerDensityGranularity = 0.10f;
		// Setup grid bucket for spacial control of marker selection
		cam->intrinsic.markerGridSize = 5;
		cam->intrinsic.markerGridCountTarget = 2;
		cam->intrinsic.markerGridBuckets.resize(cam->intrinsic.markerGridSize*cam->intrinsic.markerGridSize);
		for (int i = 0; i < cam->intrinsic.markerGridBuckets.size(); i++)
		{
			cam->intrinsic.markerGridBuckets[i].clear();
			cam->intrinsic.markerGridBuckets[i].reserve(100);
		}
	}
}

static void FinalizeCalibrationIntrinsic(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->intrinsic.calibrationStopped = true;
	}

	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

		if (cam->intrinsic.calibrationRunning && cam->intrinsic.calibrationThread->joinable())
			cam->intrinsic.calibrationThread->join();
		delete cam->intrinsic.calibrationThread;
		cam->intrinsic.calibrationThread = nullptr;
		cam->intrinsic.calibrationRunning = false;
		cam->intrinsic.calibrationFinished = false;

		wxLogMessage("Cam %d calibration: %d points, FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", m, (int)cam->intrinsic.calibrationSelection.size(), cam->camera.fovH, cam->camera.fovV, cam->camera.distortion.k1, cam->camera.distortion.k2, cam->camera.distortion.p1, cam->camera.distortion.p2, cam->camera.distortion.k3);
	}
}

static void InitCalibrationExtrinsic(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->extrinsic.relations.resize(state->cameras.size());
	}
	
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

static void FinalizeCalibrationExtrinsic(TrackingState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		rel->sample = chooseBestTransformCandidate(rel->candidates);
		rel->candidates.clear();
		wxLogMessage("Cam Rel %d-%d weight: %.4f", rel->camA, rel->camB, rel->sample.weight);
	}
}

static void InitCalibrationRoom(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->extrinsic.originCandidates.clear();
	}
}

static void FinalizeCalibrationRoom(TrackingState *state)
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

/**
 * Finalize the current phase and return to idle phase
 */
void FinalizePhase(TrackingState *state)
{
	if (state->phase != PHASE_None)
	{ // Finalize current phase
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
		// Write results of last phase
		if (writeCalibration)
		{
			std::vector<Camera> cameraCalibrations(state->cameras.size());
			for (int i = 0; i < state->cameras.size(); i++)
				cameraCalibrations[i] = state->cameras[i].camera;
			writeCalibrationFile("config/calib.json", cameraCalibrations);
		}
		// Advance phase
		state->lastPhase = state->phase;
		state->phase = PHASE_None;
	}
}

/**
 * Enters the next phase after last has been finalized
 */
void NextPhase(TrackingState *state)
{
	if (state->lastPhase == PHASE_Tracking)
	{
		wxLogError("Tracking is final phase!");
		return;
	}
	if (state->phase != PHASE_None)
	{
		wxLogError("Discarding unfinalized last phase!");
		EnterPhase(state, (ControlPhase)((int)state->phase + 1));
	}
	else
		EnterPhase(state, (ControlPhase)((int)state->lastPhase + 1));
}

/**
 * Enters the specified phase after last one has been finalized
 */
void EnterPhase(TrackingState *state, ControlPhase phase)
{
	if (state->phase == phase)
	{
		wxLogError("Already in desired phase!");
		return;
	}
	if (state->phase != PHASE_None)
	{
		wxLogError("Discarding unfinalized last phase!");
	}
	// Enter next phase
	state->lastPhase = state->phase;
	state->phase = phase;
	// Init next phase
	if (phase == PHASE_Calibration_Intrinsic)
		InitCalibrationIntrinsic(state);
	else if (phase == PHASE_Calibration_Extrinsic)
		InitCalibrationExtrinsic(state);
	else if (phase == PHASE_Calibration_Room)
		InitCalibrationRoom(state);

}

static void UpdateTrackedCalibrationMarker(TrackingState *state)
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

static void UpdateCameraRelationCandidates(TrackingState *state)
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

static void UpdateRoomOriginCandidates(TrackingState *state)
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

/**
 * Executes calibration and then exits
 */
static void CalibrationThreadFunction(TrackingState *state, CameraState *cam)
{
	cam->intrinsic.calibrationRunning = true;
	cam->intrinsic.calibrationFinished = false;
	calculateIntrinsicCalibration(cam->camera, cam->intrinsic.calibrationSelection, *state->calibration.markerTemplate2D, cam->intrinsic.calibrationMarkerErrors);
	cam->intrinsic.calibrationFinished = true;
}

static void UpdateIntrinsicCalibration(TrackingState *state)
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
			bool calibReady = selectMarkersForCalibration(cam->camera, *state->calibration.markerTemplate2D, cam->intrinsic.markers, cam->intrinsic.calibrationSelection,
				cam->intrinsic.markerRadialLookup, cam->intrinsic.markerDensityGranularity, cam->intrinsic.markerDensityTarget,
				cam->intrinsic.markerGridBuckets, cam->intrinsic.markerGridSize, cam->intrinsic.markerGridCountTarget,
				&cam->intrinsic.markerSelectionThreshold, m);

			// Start calibration round
			int cnt = cam->intrinsic.calibrationSelection.size();
			if (cnt > 10 && cnt >= cam->intrinsic.calibrationMarkerCount+10 && (calibReady || (cam->intrinsic.calibrationMarkerCount != 0 && cnt >= cam->intrinsic.calibrationMarkerCount*2)))
			{
				wxLogMessage("Cam %d starting calibration with %d points!", m, cnt);

				// Start calibration thread
				cam->intrinsic.calibrationRunning = true;
				cam->intrinsic.calibrationFinished = false;
				cam->intrinsic.calibrationMarkerCount = cnt;
				cam->intrinsic.calibrationThread = new std::thread(CalibrationThreadFunction, state, cam);
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
			filterMarkersForCalibration(cam->camera, cam->intrinsic.calibrationMarkerErrors, cam->intrinsic.calibrationSelection, cam->intrinsic.markerRadialLookup, cam->intrinsic.markerGridBuckets, m);

			// Adapt parameters for next round
//			cam->intrinsic.markerDensityTarget *= 2.0f;
			cam->intrinsic.markerDensityGranularity *= 0.5f;
			cam->intrinsic.markerGridCountTarget *= 2;
			cam->intrinsic.markerSelectionThreshold = 2.0f;
		}
	}
}

static void TrackMarkers(TrackingState *state)
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
	state->tracking.nonconflictedCount = triangulateRayIntersections(rayGroups, state->tracking.points3D, state->tracking.conflicts, state->tracking.intersectError);

	// Detect markers in triangulated point cloud
	state->tracking.poses3D.clear();
	state->tracking.posesMSE.clear();
	detectMarkers3D(state->tracking.markerTemplate3D, state->tracking.points3D, state->tracking.conflicts, state->tracking.nonconflictedCount, state->tracking.poses3D, state->tracking.posesMSE, state->tracking.sigmaError);
}

static void UndistortPoints(TrackingState *state)
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

void HandleCameraState(TrackingState *state)
{
	if (state->phase == PHASE_Tracking)
	{
		// Undistort points
		UndistortPoints(state);

		// Triangulate and detect markers
		TrackMarkers(state);
	}
	else if (state->phase == PHASE_Calibration_Intrinsic)
	{
		// Add to calibration list of good candidate, update calibration values
		UpdateIntrinsicCalibration(state);
	}
	else if (state->phase == PHASE_Calibration_Extrinsic || state->phase == PHASE_Calibration_Room)
	{
		// Undistort points using calibrated camera
		UndistortPoints(state);

		// Find calibration marker, match to previous frame and evaluate accuracy/error
		UpdateTrackedCalibrationMarker(state);

		if (state->phase == PHASE_Calibration_Extrinsic)
		{ // Use pose to establish relations between cameras to figure out relative transforms
			UpdateCameraRelationCandidates(state);
		}
		else if (state->phase == PHASE_Calibration_Room)
		{ // Establish similar relations from each camera to the room origin (or later, floor)
			UpdateRoomOriginCandidates(state);
		}
	}
}