/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "control.hpp"

#include "wxbase.hpp" // wxLog*

static void InitCalibrationIntrinsic(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
//		cam->camera.width = camWidth;
//		cam->camera.height = camHeight;
	}

	// TODO
}
static void FinalizeCalibrationIntrinsic(TrackingState *state)
{
	// TODO
	
	// Testing: Set true camera parameters
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->camera.fovH = cam->testing.camera.fovH;
		cam->camera.fovV = cam->testing.camera.fovV;
		// Distortions
	}
}

static void InitCalibrationExtrinsic(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->calibration.relations.resize(state->cameras.size());
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
			camA->calibration.relations[n] = rel;
			camB->calibration.relations[m] = rel;
		}
	}
}
static TransformSample ResolveTransformCandidate(TransformCandidate &candidate)
{
	TransformSample resolved;
	resolved.transform = candidate.transform;
	resolved.weight = candidate.weight;
	// TODO: Filter out outliers out of datapoints to improve reliability
	resolved.stdDeviation = 0;
	return resolved;
}
static void FinalizeCalibrationExtrinsic(TrackingState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		// Choose best candidate for this relation
		int candidate = -1;
		float weight = 0;
		for (int c = 0; c < rel->candidates.size(); c++)
		{
			if (weight < rel->candidates[c].weight)
			{
				candidate = c;
				weight = rel->candidates[c].weight;
			}
		}
		// Apply candidate and analyze
		if (candidate != -1)
			rel->sample = ResolveTransformCandidate(rel->candidates[candidate]);
		else
			rel->sample.weight = 0;
		// Clear intermediate candidates
		rel->candidates.clear();

		wxLogMessage("Cam Rel %d-%d has weight of %f", rel->camA, rel->camB, rel->sample.weight);
	}
}

static void InitCalibrationRoom(TrackingState *state)
{
	// TODO
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->calibration.originCandidates.clear();
	}
}
static void FinalizeCalibrationRoom(TrackingState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		wxLogMessage("Cam Rel %d-%d has weight of %f", rel->camA, rel->camB, rel->sample.weight);
	}
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		// Choose best candidate for this cameras relation to the origin
		int candidate = -1;
		float weight = 0;
		for (int c = 0; c < cam->calibration.originCandidates.size(); c++)
		{
			if (weight < cam->calibration.originCandidates[c].weight)
			{
				candidate = c;
				weight = cam->calibration.originCandidates[c].weight;
			}
		}
		// Apply candidate and analyze
		if (candidate != -1)
			cam->calibration.origin = ResolveTransformCandidate(cam->calibration.originCandidates[candidate]);
		else
			cam->calibration.origin.weight = 0;
		// Clear intermediate candidates
		cam->calibration.originCandidates.clear();
		
		wxLogMessage("Cam %d origin has weight of %f", m, cam->calibration.origin.weight);
	}

	// Determine absolute position of cameras
	// Instead of simply taking the origin pose from each camera (which can be noisy or non existant based on camera position),
	// the relation to other cameras is favoured more and included,
	// since they are (1) more important and (2) potentially more reliable (pose can be closer to each camera).
	// So for each camera, all paths to the origin through all cameras are considered.
	// These paths are weighted based on it's weakest link by using the parallel resistor formula.
	// This also naturally supports multi-room setups where not each camera can see the origin,
	// instead only some overlapping space with at least one other camera

	// Weighted transforms for each camera based on multiple paths
	std::vector<std::vector<std::pair<Eigen::Isometry3f, float>>> transforms (state->cameras.size());

	const int len = state->cameras.size(); // Can be set to limit path length for large camera setups
	const int cnt = state->cameras.size();
	std::vector<int> path (len);
	std::vector<bool> used (cnt);	
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
		float pathWeight = 1/state->cameras[path[0]].calibration.origin.weight;
		bool pathInvalid = false;
		for (int i = 1; i <= pos; i++)
		{
			float relWeight = state->cameras[path[i-1]].calibration.relations[path[i]]->sample.weight;
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
			Eigen::Isometry3f pathTransform = state->cameras[path[0]].calibration.origin.transform.inverse();
			for (int i = 1; i <= pos; i++)
			{
				CameraRelation *rel = state->cameras[path[i-1]].calibration.relations[path[i]];
				if (rel->camA == path[i-1])
					pathTransform = pathTransform * rel->sample.transform;
				else // Flip
					pathTransform = pathTransform * rel->sample.transform.inverse();
			}
			
			// Register as weighted path transform
			transforms[path[pos]].push_back({ pathTransform, pathWeight });

			{ // Debug
				std::stringstream ss;
				for (int i = 0; i <= pos; i++)
					ss << path[i] << ":";
				Eigen::Isometry3f trueTransform = state->cameras[path[pos]].testing.camera.transform;
				Eigen::Vector3f tDiff = pathTransform.translation() - trueTransform.translation();
				Eigen::Matrix3f rDiff = trueTransform.rotation() * pathTransform.rotation().transpose();
				float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
				wxLogMessage("Path %s, target %d, weight %f, tError %f, rError %f!", ss.str(), path[pos], pathWeight, tError, rError);
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
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

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
		cam->camera.transform = cameraTransform;
		
		{ // Debug
			Eigen::Isometry3f trueTransform = state->cameras[m].testing.camera.transform;
			Eigen::Vector3f tDiff = cameraTransform.translation() - trueTransform.translation();
			Eigen::Matrix3f rDiff = trueTransform.rotation() * cameraTransform.rotation().transpose();
			float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
			wxLogMessage("Camera %d, %d transforms, total weight %f, tError %f, rError %f!", m, (int)transforms[m].size(), totalWeight, tError, rError);
		}
	}

	// Testing: Set true camera relations
	/*for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];
		cam->camera.transform = cam->testing.camera.transform;
	}*/
}

/**
 * Finalizes the current phase and returns to idle phase
 */
void FinalizePhase(TrackingState *state)
{
	if (state->phase != PHASE_None)
	{ // Finalize current phase
		if (state->phase == PHASE_Calibration_Intrinsic)
			FinalizeCalibrationIntrinsic(state);
		else if (state->phase == PHASE_Calibration_Extrinsic)
			FinalizeCalibrationExtrinsic(state);
		else if (state->phase == PHASE_Calibration_Room)
			FinalizeCalibrationRoom(state);
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
		cam->calibration.lastPoses.swap(cam->calibration.poses);
		cam->calibration.poses.clear();
		cam->calibration.posesMSE.clear();
		cam->calibration.posesMatch.clear();

		// Infer pose of generic marker under above assumptions
		if (cam->points2D.size() >= getExpectedBlobCount())
		{
			Eigen::Isometry3f pose = inferMarkerPoseGeneric(cam->points2D, cam->camera);
			float MSE = calculateMSEGeneric(pose, cam->camera, cam->points2D);
			cam->calibration.poses.push_back(pose);
			cam->calibration.posesMSE.push_back(MSE);
		}

/*		// Assume all points visible belong to singular marker (TODO: Marker detection and point ordering)
		findMarkerCandidates(cam->points2D, cam->calibration.markers, cam->calibration.freeBlobs);
		cam->calibration.posesMSE.reserve(cam->calibration.markers.size());
		cam->calibration.poses.reserve(cam->calibration.markers.size());
		poseAccurate.reserve(cam->calibration.markers.size());

		// Infer pose of detected markers
		for (int i = 0; i < cam->calibration.markers.size(); i++)
		{
			Marker *marker = &cam->calibration.markers[i];
			Eigen::Isometry3f pose = inferMarkerPose(marker, cam->camera);
			// Calculate MSE
			float MSE = calculateMSE(pose, marker, cam->camera);
			float pxMSE = std::sqrt(MSE) * cam->camera.width;
			// Register pose
			cam->calibration.poses.push_back(pose);
			cam->calibration.posesMSE.push_back(pxMSE);
			poseAccurate.push_back(pxMSE <= 0.2f);
		}
*/

		// Match markers from last frame
		matchTrackedPoses(cam->calibration.poses, cam->calibration.lastPoses, cam->calibration.poseDiffPos, cam->calibration.poseDiffRot, cam->calibration.posesMatch);

		// Detect mirrored poses (due to inaccuracies) using last frames poses
		for (int p = 0; p < cam->calibration.poses.size(); p++)
		{
			if (cam->calibration.posesMatch[p] != -1)
			{
				// Last frames rotation
				int l = cam->calibration.posesMatch[p];
				float expectedAngleDiff = cam->calibration.poseDiffRot[l];
				if (expectedAngleDiff == 0) expectedAngleDiff = 10; // New pose in last frame
				Eigen::Matrix3f lastRot = cam->calibration.lastPoses[l].rotation();// + cam->calibration.poseAngularMovement[i];
				// Current frames rotation
				Eigen::Matrix3f curRot = cam->calibration.poses[p].rotation();
				float curAngleDiff = Eigen::AngleAxisf(curRot * lastRot.transpose()).angle();
				if (curAngleDiff <= 2*expectedAngleDiff) continue;
				// Test flip and compare
				Eigen::Vector3f euler = getEulerXYZ(curRot);
				Eigen::Matrix3f flippedRot = getRotationXYZ(Eigen::Vector3f(-euler.x(), -euler.y(), euler.z()));
				float flippedAngleDiff = Eigen::AngleAxisf(flippedRot * lastRot.transpose()).angle();
				if (flippedAngleDiff < 2.0f*expectedAngleDiff || flippedAngleDiff*1.5 < curAngleDiff)
				{ // Flip rotation, because it conforms significantly more with expected rotation
					Eigen::Isometry3f flippedPose = cam->calibration.poses[p];
					flippedPose.linear() = flippedRot;
					float flippedMSE = calculateMSEGeneric(flippedPose, cam->camera, cam->points2D);
					cam->calibration.poses[p] = flippedPose;
					cam->calibration.posesMSE[p] = flippedMSE*10;
//					wxLogMessage("Cam %d, Pose %d (%d):  Flipped, new MSE: %f! Diffs exp: %f, act: %f, flp: %f", m, p, l, flippedMSE, expectedAngleDiff/PI*180, curAngleDiff/PI*180, flippedAngleDiff/PI*180);
				}
			}
		}

		// Ignore rotation of inaccurate poses
		for (int p = 0; p < cam->calibration.poses.size(); p++)
		{
			if (cam->calibration.posesMSE[p] > MAX_POSE_MSE)
			{
				wxLogMessage("Cam %d: Discarded pose with MSE of %f ", m, cam->calibration.posesMSE[p]);
				int l = cam->calibration.posesMatch[p];
				if (l != -1) cam->calibration.poses[p].linear() = cam->calibration.lastPoses[l].rotation();
			}
		}

		// Update temporal information using matching
		matchAccept(cam->calibration.poses, cam->calibration.lastPoses, cam->calibration.poseDiffPos, cam->calibration.poseDiffRot, cam->calibration.posesMatch);
	}
}

static int UpdateCandidates (std::vector<TransformCandidate> &candidates, Eigen::Isometry3f transform, float weight, float maxTError, float maxRError)
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

static void UpdateCameraRelationCandidates(TrackingState *state)
{
	for (int r = 0; r < state->calibration.relations.size(); r++)
	{
		CameraRelation *rel = &state->calibration.relations[r];
		CameraState *camA = &state->cameras[rel->camA];
		CameraState *camB = &state->cameras[rel->camB];

		for (int a = 0; a < camA->calibration.poses.size(); a++)
		{
			if (camA->calibration.posesMSE[a] > MAX_POSE_MSE) continue;
			const Eigen::Isometry3f poseA = camA->calibration.poses[a];
			for (int b = 0; b < camB->calibration.poses.size(); b++)
			{
				if (camB->calibration.posesMSE[b] > MAX_POSE_MSE) continue;
				const Eigen::Isometry3f poseB = camB->calibration.poses[b];
				// Datapoint (Transformation from camera a to camera b)
				Eigen::Isometry3f transAB = poseA * poseB.inverse();
				// Weight of datapoint
				float distA = poseA.translation().norm(), rotA = std::abs(Eigen::AngleAxisf(poseA.rotation()).angle())/PI*2;
				float distB = poseB.translation().norm(), rotB = std::abs(Eigen::AngleAxisf(poseB.rotation()).angle())/PI*2;
				float weight = distA * (1+rotA) + distB * (1+rotB);
				weight = 1000.0f/(weight*weight);
				// Add to suitable candidate
				int candidateInd = UpdateCandidates(rel->candidates, transAB, weight, 20, 2);

				{ // Debug
					// Calculate error between data point and ground truth
					Eigen::Isometry3f transABGT = camA->camera.transform.inverse() * camB->camera.transform;
					Eigen::Vector3f tDiff = transAB.translation() - transABGT.translation();
					Eigen::Matrix3f rDiff = transAB.rotation() * transABGT.rotation().transpose();
					// Compare with candidate
					TransformCandidate *candidate = &rel->candidates[candidateInd];
					Eigen::Vector3f ctDiff = candidate->transform.translation() - transABGT.translation();
					Eigen::Matrix3f crDiff = candidate->transform.rotation() * transABGT.rotation().transpose();
					// Debug
					wxLogMessage("Found rel pose between %d and %d with error (%f, %f) from GT, added to candidate %d with error (%f, %f)", rel->camA, rel->camB, tDiff.norm(), Eigen::AngleAxisf(rDiff).angle()/PI*180, candidateInd, ctDiff.norm(), Eigen::AngleAxisf(crDiff).angle()/PI*180);
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
		for (int p = 0; p < cam->calibration.poses.size(); p++)
		{
			if (cam->calibration.posesMSE[p] > MAX_POSE_MSE) continue;
			const Eigen::Isometry3f pose = cam->calibration.poses[p];
			float dist = pose.translation().norm(), rot = std::abs(Eigen::AngleAxisf(pose.rotation()).angle())/PI*2;
			float weight = dist * (1+rot);
			weight = 1000.0f/(weight*weight);
			int candidateInd = UpdateCandidates(cam->calibration.originCandidates, pose, weight, 5, 1);
			
			{ // Debug
				// Calculate error between data point and ground truth
				Eigen::Isometry3f poseGT = cam->camera.transform.inverse();
				Eigen::Vector3f tDiff = pose.translation() - poseGT.translation();
				Eigen::Matrix3f rDiff = pose.rotation() * poseGT.rotation().transpose();
				// Compare with candidate
				TransformCandidate *candidate = &cam->calibration.originCandidates[candidateInd];
				Eigen::Vector3f ctDiff = candidate->transform.translation() - poseGT.translation();
				Eigen::Matrix3f crDiff = candidate->transform.rotation() * poseGT.rotation().transpose();
				// Debug
				wxLogMessage("Found origin pose with error (%f, %f) from GT, added to candidate %d with error (%f, %f)", tDiff.norm(), Eigen::AngleAxisf(rDiff).angle()/PI*180, candidateInd, ctDiff.norm(), Eigen::AngleAxisf(crDiff).angle()/PI*180);
			}

		}
	}
}

static void TrackMarkers(TrackingState *state)
{
	for (int m = 0; m < state->cameras.size(); m++)
	{
		CameraState *cam = &state->cameras[m];

		// Create 3D Rays for triangulation assuming calibrated camera
		cam->tracking.rays3D.clear();
		castRays(cam->points2D, cam->camera, cam->tracking.rays3D);
	}

	// Perform triangulation using 3D Rays
	std::vector<std::vector<Ray>*> rayGroups;
	for (int m = 0; m < state->cameras.size(); m++)
		rayGroups.push_back(&state->cameras[m].tracking.rays3D);
	state->tracking.points3D.clear();
	state->tracking.conflicts.clear();
	state->tracking.nonconflictedCount = triangulateRayIntersections(rayGroups, state->tracking.points3D, state->tracking.conflicts, 0.01f);

	// Detect markers in triangulated point cloud
	state->tracking.poses3D.clear();
	state->tracking.posesMSE.clear();
	detectMarkers3D(state->tracking.points3D, state->tracking.conflicts, state->tracking.nonconflictedCount, state->tracking.poses3D, state->tracking.posesMSE);
}

void HandleCameraState(TrackingState *state)
{
	if (state->phase == PHASE_Tracking)
	{
		TrackMarkers(state);
	}
	else if (state->phase == PHASE_Calibration_Intrinsic || state->phase == PHASE_Calibration_Extrinsic || state->phase == PHASE_Calibration_Room)
	{
		// Find calibration marker, match to previous frame and evaluate accuracy/error
		UpdateTrackedCalibrationMarker(state);

		if (state->phase == PHASE_Calibration_Intrinsic)
		{ // Add to calibration list of good candidate, update calibration values
			// TODO
		}
		else if (state->phase == PHASE_Calibration_Extrinsic)
		{ // Use pose to establish relations between cameras to figure out relative transforms
			UpdateCameraRelationCandidates(state);
		}
		else if (state->phase == PHASE_Calibration_Room)
		{ // Establish similar relations from each camera to the room origin (or later, floor)
			UpdateRoomOriginCandidates(state);
		}
	}
}