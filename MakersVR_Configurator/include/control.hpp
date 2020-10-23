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
#include <bitset>

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
struct CameraState
{
	Camera camera; // Calibration data
	std::vector<Eigen::Vector2f> points2D; // Points2D input
	std::vector<Eigen::Vector2f> undistorted2D; // undistorted points2D
	std::vector<float> pointSizes; // Points2D input
	struct { // Single-Camera state (Intrinsic calibration)
		std::vector<Marker2D> calibrationSelection;
		float markerSelectionThreshold;
		// Current frame state
		std::vector<Marker2D> markers;
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
		std::vector<Marker2D> markers;
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
};

struct TrackingState
{
	enum ControlPhase lastPhase;
	enum ControlPhase phase;
	std::vector<CameraState> cameras;
	struct { // Single-Camera state (Calibration)
		std::vector<DefMarker> markerTemplates2D;
		DefMarker *markerTemplate2D;
		std::vector<CameraRelation> relations;
	} calibration;
	struct { // Multi-Camera state (Tracking)
		std::vector<MarkerTemplate3D> markerTemplates3D;
		int trackID; // ID of currently tracked marker
		float sigmaError;
		float minIntersectError;
		float maxIntersectError;
		std::vector<TriangulatedPoint> points3D;
		int nonconflictedCount;
		std::vector<std::vector<int>> conflicts;
		std::vector<Eigen::Isometry3f> poses3D;
		std::vector<std::pair<float,int>> posesMSE;
	} tracking;
	struct { // Marker Calibration state
		MarkerTemplate3D iterativeMarker;
		int iteration;
		std::vector<Eigen::Vector3f> markerPoints;
		std::vector<std::pair<float, int>> markerPointRating;
	} markerCalib;
	struct { // Testing and visualization state
		bool isTesting;
		Eigen::Isometry3f targetPose;
		Eigen::Isometry3f GT;
		std::vector<Ray> rays3D;
		std::vector<Eigen::Vector3f> triangulatedPoints3D;
		float blobPxStdDev;
		DefMarker *markerTemplate3D;
	} testing;
};

/* Functions */

/**
 * Finalize the current phase and return to the idle phase
 */
void FinalizePhase(TrackingState *state);

/**
 * Discard the current phase and return to the idle phase
 */
void DiscardPhase(TrackingState *state);

/**
 * Enters the specified phase from the idle phase
 */
void EnterPhase(TrackingState *state, ControlPhase phase);

/**
 * Enters the next phase, discarding any unfinalized current phases
 */
void NextPhase(TrackingState *state);

/**
 * Handles a new frame
 */
void HandleCameraState(TrackingState *state);


#endif