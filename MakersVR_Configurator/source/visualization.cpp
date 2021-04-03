/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "visualization.hpp"

#include "mesh.hpp"

/* Structures */

struct Line2D
{
	float startX;
	float startY;
	float endX;
	float endY;
};


/* Variables */

Mesh *vizCoordCross;
GLuint vizLinesVBO; // Line buffer for uploading visualization lines to GPU

/* Functions */

/**
 * Initialize resources for visualization
 */
void initVisualization()
{
	// Init visualization coordinate cross
	vizCoordCross = new Mesh ({ POS, COL }, {
		 0, 0, 0, 1, 0, 0,
		 1, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 1, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1,
		 0, 0, 1, 0, 0, 1,
	}, {});
	vizCoordCross->setMode(GL_LINES);

	// Setup VertexBufferObject for line data
	glGenBuffers(1, &vizLinesVBO);
}

/**
 * Cleanup of resources
 */
void cleanVisualization()
{
	if (vizCoordCross != nullptr)
		delete vizCoordCross;
	if (vizLinesVBO != 0)
		glDeleteBuffers(1, &vizLinesVBO);
}

/**
 * Visualize 2D points in pixel space
 */
void visualizePoints2D(const Camera &camera, const std::vector<Eigen::Vector2f> &points2D, Color color, float size, float depth, bool undistort)
{
	glColor3f(color.r, color.g, color.b);
	glPointSize(size);
	glBegin(GL_POINTS);
	for (int i = 0; i < points2D.size(); i++)
		glVertex3f(points2D[i].x()/camera.width*2-1, points2D[i].y()/camera.height*2-1, depth);
	glEnd();
}

/**
 * Visualize 3D points in world space
 */
void visualizePoints3D(const Camera &camera, const std::vector<Eigen::Vector3f> &points3D, Color color, float size, float depth)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);
	glColor3f(color.r, color.g, color.b);
	glPointSize(size);
	glBegin(GL_POINTS);
	for (int i = 0; i < points3D.size(); i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i]);
		glVertex3f(pt.x(), pt.y(), depth == 0? pt.z() : depth);
	}
	glEnd();
}

/**
 * Visualize poses in camera or world space
 */
void visualizePoses(const Camera &camera, const std::vector<Eigen::Isometry3f> &poses3D, bool cameraSpace, Color color, float scale)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV);
	if (!cameraSpace) vp = vp * createViewMatrix(camera.transform);

	// Render poses
	glColor3f(color.r, color.g, color.b);
	glLineWidth(2.0f);
	for (int i = 0; i < poses3D.size(); i++)
	{
		glLoadMatrixf((vp * poses3D[i] * Eigen::Scaling(scale)).data());
		vizCoordCross->draw();
	}
	glLoadIdentity();
}

/**
 * Visualize calibration markers in pixel space
 */
void visualizeMarkers(const Camera &camera, const std::vector<Marker2D> &markers2D, Color color)
{
	// Render marker lines
	std::vector<struct Line2D> markerLines;
	for (int m = 0; m < markers2D.size(); m++)
	{
		const Marker2D *marker = &markers2D[m];
		// Display base
		markerLines.push_back({
			marker->points[0].x() / camera.width * 2 - 1,
			marker->points[0].y() / camera.height * 2 - 1,
			marker->points[1].x() / camera.width * 2 - 1,
			marker->points[1].y() / camera.height * 2 - 1
		});
		// Display spine
		markerLines.push_back({
			marker->points[5].x() / camera.width * 2 - 1,
			marker->points[5].y() / camera.height * 2 - 1,
			marker->points[2].x() / camera.width * 2 - 1,
			marker->points[2].y() / camera.height * 2 - 1
		});
		// Display head left
		markerLines.push_back({
			marker->points[2].x() / camera.width * 2 - 1,
			marker->points[2].y() / camera.height * 2 - 1,
			marker->points[3].x() / camera.width * 2 - 1,
			marker->points[3].y() / camera.height * 2 - 1
		});
		// Display head right
		markerLines.push_back({
			marker->points[2].x() / camera.width * 2 - 1,
			marker->points[2].y() / camera.height * 2 - 1,
			marker->points[4].x() / camera.width * 2 - 1,
			marker->points[4].y() / camera.height * 2 - 1
		});
	}
	if (markerLines.size() > 0)
	{
		glLineWidth(2.0f);
		glColor3f(color.r, color.g, color.b);
		glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line2D) * markerLines.size(), &markerLines[0], GL_STREAM_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line2D)/2, (void *)0);
		glEnableVertexAttribArray(0);
		glDrawArrays(GL_LINES, 0, (int)markerLines.size()*2);
	}
}

/**
 * Visualize 3D Rays in world space
 */
void visualizeRays(const Camera &camera, const std::vector<Ray> &rays3D, Color color)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);

	// Render ray lines
	std::vector<struct Line2D> rayLines;
	for (int r = 0; r < rays3D.size(); r++)
	{
		const Ray *ray = &rays3D[r];
		const int segCnt = 5;
		Eigen::Vector3f segStart = projectPoint(vp, ray->pos);
		for (int i = 0; i < segCnt; i ++)
		{ // Display ray in segments to prevent weird clipping
			Eigen::Vector3f segEnd = projectPoint(vp, (ray->pos + ray->dir * (float)(i+1)/segCnt * 1000));
			// Register ray segment
			rayLines.push_back({
				segStart.x(),
				segStart.y(),
				segEnd.x(),
				segEnd.y()
			});
			segStart = segEnd;
		}
	}
	if (rayLines.size() > 0)
	{
		glLineWidth(0.5f);
		glColor3f(color.r, color.g, color.b);
		glBindBuffer(GL_ARRAY_BUFFER, vizLinesVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(struct Line2D) * rayLines.size(), &rayLines[0], GL_STREAM_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(struct Line2D)/2, (void *)0);
		glEnableVertexAttribArray(0);
		glDrawArrays(GL_LINES, 0, (int)rayLines.size()*2);
	}
}

/**
 * Visualize triangulated point cloud in world space
 */
void visualizeTriangulation(const Camera &camera, const std::vector<TriangulatedPoint> &points3D, int nonconflictedCount, Color colorNC, Color colorC)
{
	Eigen::Projective3f vp = createProjectionMatrix(camera.fovH, camera.fovV) * createViewMatrix(camera.transform);

	// Render all triangulated points
	glColor3f(colorNC.r, colorNC.g, colorNC.b);
	for (int i = 0; i < nonconflictedCount; i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i].pos);
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x(), pt.y(), pt.z());
		glEnd();
	}
	glColor3f(colorC.r, colorC.g, colorC.b);
	for (int i = nonconflictedCount; i < points3D.size(); i++)
	{
		Eigen::Vector3f pt = projectPoint(vp, points3D[i].pos);
//		glPointSize(4.0f);
		glPointSize(3.0f * points3D[i].confidence + 1.0f);
		glBegin(GL_POINTS); // Has to be called here to change point size without a separate shader
		glVertex3f(pt.x(), pt.y(), pt.z());
		glEnd();
	}
}

/**
 * Visualize camera distortion using a grid of size num
 */
void visualizeDistortion(const Camera &cameraCB, const Camera &cameraGT)
{
	// Camera projections
	Eigen::Projective3f PGT = createProjectionMatrix(cameraGT.fovH, cameraGT.fovV);
	Eigen::Projective3f PCB = createProjectionMatrix(cameraCB.fovH, cameraCB.fovV);

	// Grid parameters
	int num = 10;
	float hSize = 150, vSize = 150, dist = 100;

	// Resulting points
	std::vector<Eigen::Vector2f> undistorted, distortedGT, distortedCB;
	undistorted.reserve(num*num);
	distortedGT.reserve(num*num);
	distortedCB.reserve(num*num);

	for (int i = 1; i < num+1; i++)
	{
		for (int j = 1; j < num+1; j++)
		{
			Eigen::Vector3f point (i * hSize/(num+1) - hSize/2, j * vSize/(num+1) - vSize/2, -dist);
			Eigen::Vector2f projGT = projectPoint(PGT, point).head<2>();
			Eigen::Vector2f projCB = projectPoint(PCB, point).head<2>();
			projGT.x() = (projGT.x()+1)/2 * cameraGT.width;
			projGT.y() = (projGT.y()+1)/2 * cameraGT.height;
			projCB.x() = (projCB.x()+1)/2 * cameraCB.width;
			projCB.y() = (projCB.y()+1)/2 * cameraCB.height;
			undistorted.push_back(projGT);
			distortedGT.push_back(distortPoint(cameraGT, projGT));
			distortedCB.push_back(distortPoint(cameraCB, projCB));
		}
	}

	visualizePoints2D(cameraGT, undistorted, { 0,0,1 }, 2.0f, 0.6f);
	visualizePoints2D(cameraGT, distortedGT, { 0,1,0 }, 2.0f, 0.5f);
	visualizePoints2D(cameraCB, distortedCB, { 1,1,0 }, 2.0f, 0.4f);
}