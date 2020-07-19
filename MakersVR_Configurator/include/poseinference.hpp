/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_POSE_INFERENCE
#define DEF_POSE_INFERENCE

#include <vector>

#define USE_CV
#define USE_EIGEN

#ifdef USE_CV
// Don't include full library, only needed modules
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#endif

/*
 * Pose Inference
 * Responsible for trying to infer 3D poses from a set of 2D blobs in screen space using solvePnP
 * Currently works on a frame-by-frame basis, so no association to witch drone exists
 *
 * Stretch Goal:
 * Takes poses from previous frames and tries to match them, to provide solvePnP with an initial estimation
 * This will improve performance AND accuracy
 */

/* Structures  */

// Point with size
typedef struct Point{
	float X;
	float Y;
	float S;
} Point;

// Pose of LED marker
typedef struct Pose
{
	Point *center;
	double translation[3];
	double rotation[3];
	// TODO
	// 3D AND 2D pixel space of all LEDs
	// Need access to at least the center LED screen space position for color fetching later
} Pose;

typedef struct Marker
{
	Point *center;
	Point *endA;
	Point *endB;
	Point *header;
} Marker;
typedef struct MarkerCandidate
{
	uint8_t c;
	uint8_t a;
	uint8_t b;
	uint8_t h;
	float error;
} MarkerCandidate;

/*typedef struct Line
{
	Point edgeIntersect;
	float angle;
	float start;
	float end;
	std::vector<Point*> points;

} Line;*/

typedef struct LinePoint
{
	float X;
	float Y;
	float R;
	float G;
	float B;
} LinePoint;





// Marker pose inference
#ifdef USE_CV
extern std::vector<cv::Point3f> marker3DTemplate;
extern cv::Matx33f camMat;
extern cv::Matx14f distort;
// Current values used for intrinsic guess
extern cv::Matx31d rotation;
extern cv::Matx31d translation;
#endif




/* Functions */

/*
 * Initialize resources required for LED Tracking and setup camera parameters
 */
void initPoseInference(int Width, int Height);

/*
 * Finds all (potential) marker and also returns all free (unassociated) blobs left
 */
void findMarkerCandidates(std::vector<Point> &blobs, std::vector<Marker> &marker, std::vector<Point*> &freeBlobs);

/*
 * Infer the pose of a set of (likely) markers
 */
void inferMarkerPoses(std::vector<Marker> &marker, std::vector<Pose> &poses);


void inferMarkerPose(std::vector<Point> &marker, Pose &pose);

/*
 * Projects marker into image plane provided translation in centimeters and rotation, both relative to camera
 */
void projectMarker(std::vector<Point> &imagePoints, double *translation, double *rotation, float ptScale);

/*
 * Visualize poses
 */
void visualizePoses(const std::vector<Point> &blobs, const std::vector<Marker> &marker, const std::vector<Pose> &poses);

void cleanPoseInference();

#endif