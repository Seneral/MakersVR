/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DEF_BLOB_DETECTION
#define DEF_BLOB_DETECTION

#include <vector>
#include <cstdint>

/*
 * Blob Detection
 * Responsible for extracting blobs (LEDs) from the input camera frames
 * Output is a unordered set of points in screen space,
 * without any association to 3D pose, and is completely unfiltered
 */

/* Structures  */

// Point with size
typedef struct Point{
	float X;
	float Y;
	float S;
	Point() {}
	Point(float x, float y) : X(x), Y(y) {}
} Point;
// Pixel rectangle bounds
typedef struct Bounds
{
	uint16_t minX, maxX;
	uint16_t minY, maxY;
	Bounds() {}
	Bounds(uint16_t xl, uint16_t xu, uint16_t yl, uint16_t yu) : minX(xl), maxX(xu), minY(yl), maxY(yu) {}
} Bounds;
// Pixel dot of size one
typedef struct Dot
{
	uint16_t X;
	uint16_t Y;
	Dot() {}
	Dot(uint16_t x, uint16_t y) : X(x), Y(y) {}
} Dot;
// Cluster of pixel dots with bounds that define a point (centroid)
typedef struct Cluster
{
	Point centroid;
	Bounds bounds;
	int ptCnt;
	std::vector<Dot> dots;
	Cluster() {}
	Cluster(float x, float y, Bounds b, uint16_t cnt) : centroid(x,y), bounds(b), ptCnt(cnt) {}
	Cluster(float x, float y, uint16_t xl, uint16_t xu, uint16_t yl, uint16_t yu, uint16_t cnt) : centroid(x,y), bounds(xl, xu, yl, yu), ptCnt(cnt) {}
} Cluster;
// Color
typedef struct Color
{
	float R;
	float G;
	float B;
} Color;

/* Functions */

bool initBlobDetection(int width, int height, int offsetX = 0, int offsetY = 0);
void performBlobDetectionRegionsFetch(uint32_t *frame);
void performBlobDetectionCPU(std::vector<Cluster> &blobs);
void visualizeBlobDetection(const std::vector<Cluster> &blobs, Bounds viewBounds, float pixelDensity);
void blobColorLookup (const std::vector<Point> &points, std::vector<Color> &colors);
void cleanBlobDetection();

#endif
