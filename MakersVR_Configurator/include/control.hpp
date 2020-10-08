/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_CONTROL
#define DEF_CONTROL

#include "eigenutil.hpp"
#include "calibration.hpp"
#include "tracking.hpp"

#include <map>
#include <thread>

/**
 * Control
 */


enum ControlPhase
{
	PHASE_None = 0,
	PHASE_Calibration_Intrinsic,
	PHASE_Calibration_Extrinsic,
	PHASE_Calibration_Room,
	PHASE_Calibration_Marker,
	PHASE_Tracking
};

/**
 * Camera State of a MarkerDetector with currently estimated transform, blobs and estimated poses
 */
typedef struct
{
	Camera camera; // Calibration data
	std::vector<Eigen::Vector2f> points2D; // Points2D input
	std::vector<Eigen::Vector2f> undistorted2D; // undistorted points2D
	std::vector<float> pointSizes; // Points2D input
	struct { // Single-Camera state (Intrinsic calibration)
		std::vector<Marker> calibrationSelection;
		float markerSelectionThreshold;
		// Current frame state
		std::vector<Marker> markers;
		std::vector<int> freeBlobs;
		// Radial marker condition
		std::multimap<float,uint16_t> markerRadialLookup;
		float markerDensityTarget;
		float markerDensityGranularity;
		// Spatial marker condition
		std::vector<std::vector<uint16_t>> markerGridBuckets;
		int markerGridSize;
		int markerGridCountTarget;
		// Threaded calibration round
		std::vector<double> calibrationMarkerErrors;
		std::thread *calibrationThread;
		bool calibrationRunning;
		bool calibrationFinished;
		bool calibrationStopped;
		int calibrationMarkerCount;
	} intrinsic;
	struct { // Single-Camera state (Extrinsic calibration)
		// Extrinsic calibration state
		std::vector<CameraRelation*> relations;
		std::vector<TransformCandidate> originCandidates;
		TransformSample origin; // Result
		// Current frame state
		std::vector<Marker> markers;
		std::vector<int> freeBlobs;
		// Poses of current frame
		std::vector<Eigen::Isometry3f> poses;
		std::vector<float> posesMSE;
		std::vector<int> posesMatch;
		// Poses of last frame
		std::vector<Eigen::Isometry3f> lastPoses;
		std::vector<float> poseDiffRot;
		std::vector<Eigen::Vector3f> poseDiffPos;
	} extrinsic;
	struct { // Multi-Camera state (Tracking)
		std::vector<Ray> rays3D;
	} tracking;
	struct { // Ground-Truth values used in testing
		Camera camera;
		std::bitset<MAX_MARKER_POINTS> markerPtsVisible;
	} testing;
} CameraState;

typedef struct
{
	enum ControlPhase lastPhase;
	enum ControlPhase phase;
	std::vector<CameraState> cameras;
	struct { // Single-Camera state (Calibration)
		std::vector<CameraRelation> relations;
	} calibration;
	struct { // Multi-Camera state (Tracking)
		float sigmaError;
		float intersectError;
		std::vector<TriangulatedPoint> points3D;
		int nonconflictedCount;
		std::vector<std::vector<int>> conflicts;
		std::vector<Eigen::Isometry3f> poses3D;
		std::vector<std::pair<float,int>> posesMSE;
	} tracking;
	struct { // Testing and visualization state
		Eigen::Isometry3f targetPose;
		Eigen::Isometry3f GT;
		std::vector<Ray> rays3D;
		std::vector<Eigen::Vector3f> triangulatedPoints3D;
		float blobPxStdDev;
	} testing;
} TrackingState;

/* Functions */

/**
 * Finalizes the current phase and returns to idle phase
 */
void FinalizePhase(TrackingState *state);

/**
 * Enters the next phase after last has been finalized
 */
void NextPhase(TrackingState *state);

/**
 * Enters the specified phase after last one has been finalized
 */
void EnterPhase(TrackingState *state, ControlPhase phase);

/**
 * Handles a new frame
 */
void HandleCameraState(TrackingState *state);


#endif