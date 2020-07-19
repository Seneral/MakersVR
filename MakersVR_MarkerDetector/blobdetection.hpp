/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_BLOB_DETECTION
#define DEF_BLOB_DETECTION

#include "camGL.h"
#include "eglUtil.h"

#include <vector>

/*
 * Blob Detection
 * Responsible for extracting blobs (LEDs) from the input camera frames
 * Output is a unordered set of points in screen space,
 * without any association to 3D pose, and is completely unfiltered
 */

// Maximum number of components supported
// NOT blob number, fractured blobs could take up many components that are eventually merged together
#define MAX_COMPONENTS 256

/* Structures  */

// Point with size
typedef struct Point{
	float X;
	float Y;
	float S;
} Point;
// Pixel rectangle bounds
typedef struct Bounds
{
	int minX;
	int minY;
	int maxX;
	int maxY;
} Bounds;
// Pixel dot of size one
typedef struct Dot
{
	int X;
	int Y;
} Dot;
// Cluster of pixel dots with bounds that define a point (centroid)
typedef struct Cluster
{
	Point centroid;
	Bounds bounds;
	std::vector<Dot> dots;
} Cluster;
// Color
typedef struct Color
{
	float R;
	float G;
	float B;
} Color;

/* Functions */

void initBlobDetection (int width, int height, EGL_Setup eglSetup);
void performBlobDetection(CamGL_Frame *frame, std::vector<Cluster> &blobs);
void performBlobDetectionGPU(CamGL_Frame *frame);
void performBlobDetectionRegionsFetch();
void performBlobDetectionCPU(std::vector<Cluster> &blobs);
void visualizeBlobDetection(const std::vector<Cluster> &blobs, Bounds viewBounds, float pixelDensity);
void blobColorLookup (const std::vector<Point> &points, std::vector<Color> &colors);
void cleanBlobDetection();

#endif
