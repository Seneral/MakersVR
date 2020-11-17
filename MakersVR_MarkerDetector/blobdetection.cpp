/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// Alternative: Store buffer in VCSM (Video Core Shared Memory)
// In theory VCSM removes need for the expensive read pixels call
// In practice that is only relevant for larger buffers, for the extremely small bitmap it makes no difference
#define USE_READ_PIXELS

//#define BLOB_DEBUG
//#define BLOB_TRACE
//#define BLOB_VIZ_DOTS

#include "blobdetection.hpp"

// General GL
#include "mesh.hpp"
#include "shader.hpp"
#include "texture.hpp"

#include <cmath>
#include <map>

#ifndef USE_READ_PIXELS
#include <interface/vcsm/user-vcsm.h>
#endif

// Accessors for 4x4 region encoded in 16bit integer
#define COL(BYTES, X) (uint8_t)((BYTES) >> ((X)*4))
#define DOT(BYTES, X, Y) (uint8_t)(((BYTES) >> ((X)*4+(3-Y))) & 1)
#define ROW_PART(BYTES, X, Y) ((((BYTES) >> ((X)*4+(3-Y))) & 1) << (X))
#define ROW(BYTES, Y) (uint8_t)(ROW_PART(BYTES, 0, Y) | ROW_PART(BYTES, 1, Y) | ROW_PART(BYTES, 2, Y) | ROW_PART(BYTES, 3, Y))

// Maximum number of components supported
// NOT blob number, fractured blobs could take up many components that are eventually merged together
// Discard components above to limit performance drop
#define MAX_COMPONENTS 128
typedef uint8_t CompID; // Seems to be little to no performance difference

/* Structures  */

// Regions as read from the GPU memory
typedef uint16_t BlobMapRegion;
// Region as used for processing
typedef struct Region
{
	uint16_t x, y;
	uint16_t bytes;
	CompID compMap[4][4];
} Region;

/* Variables */

// Sizes
static int maskW, maskH, mapW, mapH;
// Relative pixels gap at which two blobs are considered one and merged
static int blobMergeBorder = 4;
// Screen Space Quad for rendering
static Mesh *SSQuad;
// Screen Space Shaders
static ShaderProgram *shaderESBlobDetectRGB, *shaderESBlobDetectY, *shaderESBlobDetectYUV;
static ShaderProgram *shaderESBlobEncode, *shaderESBlobViz, *shaderESPoint;
static ShaderProgram *shaderCamBlitRGB, *shaderCamBlitY, *shaderCamBlitYUV;
// Shader texture locations
static int texRGBAdr, texYAdrY, texYUVAdrY, texYUVAdrU, texYUVAdrV;
static int texBlitRGBAdr, texBlitYAdrY, texBlitYUVAdrY, texBlitYUVAdrU, texBlitYUVAdrV;
// Intermediate render targets
#ifdef USE_READ_PIXELS
static FrameRenderTarget *blobMask, *blobMap;
#else
static FrameRenderTarget *blobMask;
static VCSMRenderTarget *blobMap;
#endif
// Regions map buffer for readback from GPU
static BlobMapRegion *blobMapRegions;
// Dynamic buffer for all 4x4 regions that are part of a blob
static std::vector<Region> blobRegions;
// Shared buffer serving as a map from initial component ID to merged component ID
static CompID *blobCompMerge;
// Memory block of zeros to optimize checking whole rows of regions for dots at once with memcmp
static int mapRowSize;
static unsigned char *mapRowZeros;
// Point cloud buffer for uploading visualization points to GPU
static GLuint vizPointsVBO;

/* Local Functions */

static void bindExternalTexture (GLuint adr, GLuint tex, int slot);
static CompID resolveComponentMerge(CompID compID);

/*
 * Initialize resources required for blob detection
 */
bool initBlobDetection (int width, int height, EGL_Setup eglSetup)
{
	maskW = width;
	maskH = height;
	mapW = maskW / 4;
	mapH = maskH / 4;

	// Adapt blob merge border to resolution
//	blobMergeBorder = blobMergeBorder * (maskW/512);

	// Create screen-space quad for rendering
	SSQuad = new Mesh ({ POS, TEX }, {
		-1,  1, 0, 0, 1,
		 1,  1, 0, 1, 1,
		-1, -1, 0, 0, 0,
		 1,  1, 0, 1, 1,
		 1, -1, 0, 1, 0,
		-1, -1, 0, 0, 0,
	}, {});

	// Load and compile Screen Space Shaders
	shaderESBlobDetectRGB = new ShaderProgram("../gl_shaders/BlobES/vert.glsl", "../gl_shaders/BlobES/frag_blobDetectRGB.glsl");
	shaderESBlobDetectY = new ShaderProgram("../gl_shaders/BlobES/vert.glsl", "../gl_shaders/BlobES/frag_blobDetectY.glsl");
	shaderESBlobDetectYUV = new ShaderProgram("../gl_shaders/BlobES/vert.glsl", "../gl_shaders/BlobES/frag_blobDetectYUV.glsl");
	shaderESBlobEncode = new ShaderProgram("../gl_shaders/BlobES/vert.glsl", "../gl_shaders/BlobES/frag_blobEncode.glsl");
	shaderESBlobViz = new ShaderProgram("../gl_shaders/BlobES/vert.glsl", "../gl_shaders/BlobES/frag_blobViz.glsl");
	shaderESPoint = new ShaderProgram("../gl_shaders/PointES/vert.glsl", "../gl_shaders/PointES/frag.glsl");
	shaderCamBlitRGB = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camRGB.glsl");
	shaderCamBlitY = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camY.glsl");
	shaderCamBlitYUV = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camYUV.glsl");
	if (shaderESBlobDetectRGB->ID == 0 || shaderESBlobDetectY->ID == 0 || shaderESBlobDetectYUV->ID == 0 || shaderESBlobEncode->ID == 0 || shaderESBlobViz->ID == 0 || shaderESPoint->ID == 0 || shaderCamBlitRGB->ID == 0 || shaderCamBlitY->ID == 0 || shaderCamBlitYUV->ID == 0) return false;

	// Find adresses of textures in shaders
	texRGBAdr = glGetUniformLocation(shaderESBlobDetectRGB->ID, "image");
	texYAdrY = glGetUniformLocation(shaderESBlobDetectY->ID, "imageY");
	texYUVAdrY = glGetUniformLocation(shaderESBlobDetectYUV->ID, "imageY");
	texYUVAdrU = glGetUniformLocation(shaderESBlobDetectYUV->ID, "imageU");
	texYUVAdrV = glGetUniformLocation(shaderESBlobDetectYUV->ID, "imageV");
	texBlitRGBAdr = glGetUniformLocation(shaderCamBlitRGB->ID, "image");
	texBlitYAdrY = glGetUniformLocation(shaderCamBlitY->ID, "imageY");
	texBlitYUVAdrY = glGetUniformLocation(shaderCamBlitYUV->ID, "imageY");
	texBlitYUVAdrU = glGetUniformLocation(shaderCamBlitYUV->ID, "imageU");
	texBlitYUVAdrV = glGetUniformLocation(shaderCamBlitYUV->ID, "imageV");

#ifdef USE_READ_PIXELS
	// Setup intermediate Render Targets
	blobMask = new FrameRenderTarget(maskW, maskH, GL_RGBA, GL_UNSIGNED_BYTE);
	blobMap = new FrameRenderTarget(mapW/2, mapH, GL_RGBA, GL_UNSIGNED_BYTE);
	// Allocate memory for regions map read back from the GPU
	blobMapRegions = (BlobMapRegion*)malloc(mapW * mapH * 2);
#else
	// Setup Render Targets in Shared Memory
	vcsm_init();
	blobMask = new FrameRenderTarget(maskW, maskH, GL_RGBA, GL_UNSIGNED_BYTE);
	blobMap = new VCSMRenderTarget(mapW/2, mapH, eglSetup.display);
#endif

	// Setup resources used during blob detection
	blobRegions.reserve(128);
	blobCompMerge = (CompID*)malloc(MAX_COMPONENTS * sizeof(CompID));

	// Allocate helper block of zeros of apropriate size of one map row
	mapRowSize = mapW*2; // 2 Byte per map pixel (4x4 16Bit region)
	mapRowZeros = (unsigned char*)malloc(mapRowSize);
	memset(mapRowZeros, 0, mapRowSize);

	// Setup VertexBufferObject for point data
	glGenBuffers(1, &vizPointsVBO);

	return true;
}

/*
 * Perform blob detection step on the frame texture and output it into both target arrays in point and blob format
 * Intermediate results are available in blobMask and blobMap until next step
 */
void performBlobDetectionBlocking(CamGL_Frame *frame, std::vector<Cluster> &blobs)
{
	performBlobDetectionGPU(frame);
	performBlobDetectionRegionsFetch();
	performBlobDetectionCPU(blobs);
}

/*
 * Perform blob detection GPU passes on the frame texture
 * Results are stored in blobMask and blobMap on the GPU, ready for readback
 */
void performBlobDetectionGPU(CamGL_Frame *frame)
{
	// Apply filter to extract binary decision (part of blob or not) to alpha channel (keeping color intact)
	ShaderProgram *shader;

	// Select external textures to bind as source
	if (frame->format == CAMGL_RGB)
	{
		shader = shaderESBlobDetectRGB;
		shader->use();
		bindExternalTexture(texRGBAdr, frame->textureRGB, 0);
	}
	else if (frame->format == CAMGL_Y)
	{
		shader = shaderESBlobDetectY;
		shader->use();
		bindExternalTexture(texYAdrY, frame->textureY, 0);
	}
	else if (frame->format == CAMGL_YUV)
	{
		shader = shaderESBlobDetectYUV;
		shader->use();
		bindExternalTexture(texYUVAdrY, frame->textureY, 0);
		bindExternalTexture(texYUVAdrU, frame->textureU, 1);
		bindExternalTexture(texYUVAdrV, frame->textureV, 2);
	}
	glUniform1i(shader->uWidthAdr, maskW);
	glUniform1i(shader->uHeightAdr, maskH);

	// Render from camera frame source to blobMask
	blobMask->setTarget();
	SSQuad->draw();

	// Encode binary blob flag in source alpha into regions of full color
	// Each region is 4x4 and stores 4bit per channel in 4 channels
	shaderESBlobEncode->use();
	blobMask->setSource(shaderESBlobEncode, 0);
	blobMap->setTarget();
	SSQuad->draw();
}

/*
 * Reads back blobMap from the GPU into the specified buffer, ready for analysation on the CPU
 */
void performBlobDetectionRegionsFetch()
{
	int xMin = 0, xMax = mapW, yMin = 0, yMax = mapH;
	int bufW = mapW, bufH = mapH;

	blobRegions.clear();

#ifdef USE_READ_PIXELS
	// Read back encoded regions map from GPU memory
	glReadPixels(0, 0, mapW/2, mapH, GL_RGBA, GL_UNSIGNED_BYTE, blobMapRegions);
#else
	// Wait for current GL operations to finish
	glFinish();
	// Lock the blobMap shared memory so CPU can access it
	blobMapRegions = (BlobMapRegion*)blobMap->lock();
	bufW = blobMap->bufferWidth*2;
	bufH = blobMap->bufferHeight;
#endif

	// Read map and extract regions with dots (1s) as blob regions and enter them in a list and a map
	for (int y = yMin; y < yMax; y++)
	{
		if (!memcmp(&blobMapRegions[y * bufW], mapRowZeros, mapRowSize)) continue;
		// Row (4 rows in source, 1 in map) has at least one dot in it
		// Test if this brings any real gains. Since for rows with dots this basically checks twice

		for (int x = xMin; x < xMax; x++)
		{
			uint16_t bytes = blobMapRegions[y * bufW + x];
			if (bytes != 0)
			{ // Region (4x4 pixels) has at least one dot in it
				int regInd = blobRegions.size();
				blobRegions.push_back({});
				blobRegions[regInd].x = x;
				blobRegions[regInd].y = y;
				blobRegions[regInd].bytes = bytes;
			}
		}
	}

#ifdef BLOB_DEBUG
	printf("Found %d regions with blobs!\n", blobRegions.size());
#endif

#ifndef USE_READ_PIXELS
	blobMap->unlock();
#endif
}

/*
 * Analyses the regions detected in the last GPU step and outputs detected blobs into target array
 */
void performBlobDetectionCPU(std::vector<Cluster> &blobs)
{
	blobCompMerge[0] = 0;

	CompID compNum = 0; // Number of final components
	CompID compIndex = 0; // Number of intermediary components

	// Iterate over blob regions do connected component labeling
	for (int i = 0; i < blobRegions.size(); i++)
	{
		Region *region = &blobRegions[i];

		// Find top region
		Region *topRegion = nullptr;
		for (int j = i-1; j >= 0; j--)
		{
			if (blobRegions[j].y+1 < region->y) break; // Checked full top row
			if (blobRegions[j].y+1 == region->y)
			{
				if (blobRegions[j].x < region->x) break; // Checked up until current column in top row
				if (blobRegions[j].x == region->x)
				{ // Found top region
					topRegion = &blobRegions[j];
					break;
				}
			}
		}

		// Find left region
		Region *leftRegion = nullptr;
		if (i > 0 && blobRegions[i-1].x+1 == region->x && blobRegions[i-1].y == region->y)
			leftRegion = &blobRegions[i-1];

#ifdef BLOB_TRACE
		std::cout << "Region " << region->x << " / " << region->y << ": T?" << (topRegion? "y" : "n") << ", L?" << (leftRegion? "y" : "n") << "! -- State: " << (int)compNum << "(" << (int)compIndex << ") components!\n";
#endif

		// Do connected component labelling within 4x4 region using connected components from top and left regions
		for (int x = 0; x < 4; x++)
		{
			CompID top = 0;
			if (topRegion) top = topRegion->compMap[x][3];

			for (int y = 0; y < 4; y++)
			{
				CompID compID = 0;
				if (DOT(region->bytes, x, y))
				{ // Dot at current pixel

					CompID left = 0;
					if (x > 0) left = region->compMap[x-1][y];
					else if (leftRegion) left = leftRegion->compMap[3][y];

					if (top != 0 && left != 0)
					{ // Two connected dots, assign and make sure both are merged
						CompID topComp = resolveComponentMerge(top);
						CompID leftComp = resolveComponentMerge(left);
						compID = topComp;

						if (leftComp != topComp)
						{ // Separated components connected through this dot, merge components
							blobCompMerge[leftComp] = topComp;
							compNum--;
#ifdef BLOB_TRACE
							std::cout << "Merging " << (int)left << "(" << (int) leftComp << ") into " << (int)top << " (" << (int)topComp << ") at pos " << x << "/" << y << "!\n";
#endif
						}
					}
					else if (top == 0 && left == 0)
					{ // No connected components from top or left, assign new component
						if (compIndex < MAX_COMPONENTS-1)
						{
							compID = ++compIndex;
							blobCompMerge[compID] = compID;
							compNum++;
#ifdef BLOB_TRACE
							std::cout << "Assigning new component ID " << (int)compID << "! Merge: " << (int)blobCompMerge[compID] << "\n";
#endif
						}
					}
					else
					{ // One connected component to assign to
						compID = resolveComponentMerge(top | left);
					}
				}

				// Assign component ID
				top = compID;
				region->compMap[x][y] = compID;
			}
		}
	}

	// Merge all components (flatten merge hierarchy)
	for (int i = 0; i < compIndex; i++)
	{
		resolveComponentMerge(i);
	}

#ifdef BLOB_DEBUG
	printf("Found %d initial merged components, with max %d!\n", compNum, compIndex);
#endif

	// Compile final cluster assignment
	std::map<CompID, Cluster*> clusterTable;
	blobs.reserve(blobs.size() + compNum);

	for (int i = 0; i < blobRegions.size(); i++)
	{
		Region *region = &blobRegions[i];
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				CompID compID = blobCompMerge[region->compMap[x][y]];
				if (compID == 0) continue;
				// Dot here
				auto clusterSearch = clusterTable.find(compID);
				Cluster *cluster;
				if (clusterSearch != clusterTable.end())
				{ // Existing cluster for component
					cluster = clusterSearch->second;
				}
				else
				{ // No cluster created for this component
					blobs.push_back({});
					cluster = &blobs[blobs.size()-1];
					cluster->bounds = { .minX = maskW, .minY = maskH, .maxX = 0, .maxY = 0 };
					clusterTable.insert({ compID, cluster });
#ifdef BLOB_TRACE
					std::cout << "Generated new cluster " << blobs.size()-1 << " for component ID " << (int)compID << " (originally " << (int)region->compMap[x][y] << ")! Bytes: " << region->bytes <<  "!\n";
#endif
				}
				// Add dot to cluster
				int dotX = region->x * 4 + x, dotY = region->y * 4 + y;
				cluster->dots.push_back({ dotX, dotY });
				// Update sum for centroid
				cluster->centroid.X += dotX;
				cluster->centroid.Y += dotY;
				// Update bounds
				cluster->bounds.minX = std::min(cluster->bounds.minX, dotX);
				cluster->bounds.minY = std::min(cluster->bounds.minY, dotY);
				cluster->bounds.maxX = std::max(cluster->bounds.maxX, dotX);
				cluster->bounds.maxY = std::max(cluster->bounds.maxY, dotY);
			}
		}
	}

#ifdef BLOB_DEBUG
	printf("Found %d blobs!\n", blobs.size());
#endif

	// Finalize clusters and add to points
	for (int i = 0; i < blobs.size(); i++)
	{
		Cluster *cluster = &blobs[i];
		// Finalize centroid and move to pixel center
		cluster->centroid.X = cluster->centroid.X / cluster->dots.size() + 0.5f;
		cluster->centroid.Y = cluster->centroid.Y / cluster->dots.size() + 0.5f;
		//cluster->centroid.S = ((cluster->bounds.maxX-cluster->bounds.minX)+(cluster->bounds.maxY-cluster->bounds.minY))/2;
		cluster->centroid.S = std::sqrt((float)cluster->dots.size()); // Nice approximation for circular blobs
#ifdef BLOB_TRACE
		std::cout << "Cluster " << i << " had size " << cluster->centroid.S << " around " << cluster->centroid.X << " / " << cluster->centroid.Y << " with " << cluster->dots.size() << " dots!\n";
#endif
	}
}

void visualizeFrame(CamGL_Frame *frame)
{
	if (frame->format == CAMGL_RGB)
	{
		shaderCamBlitRGB->use();
		bindExternalTexture(texBlitRGBAdr, frame->textureRGB, 0);
	}
	else if (frame->format == CAMGL_Y)
	{
		shaderCamBlitY->use();
		bindExternalTexture(texBlitYAdrY, frame->textureY, 0);
	}
	else if (frame->format == CAMGL_YUV)
	{
		shaderCamBlitYUV->use();
		bindExternalTexture(texBlitYUVAdrY, frame->textureY, 0);
		bindExternalTexture(texBlitYUVAdrU, frame->textureU, 1);
		bindExternalTexture(texBlitYUVAdrV, frame->textureV, 2);
	}
	SSQuad->draw();
}

/*
 * Visualizes given point and blob results using last steps intermediate results
 */
void visualizeBlobDetection(const std::vector<Cluster> &blobs, Bounds viewBounds, float pixelDensity)
{
	// Visualize camera image and initial blob map
	shaderESBlobViz->use();
	blobMask->setSource(shaderESBlobViz, 0);
	glUniform1i(glGetUniformLocation(shaderESBlobViz->ID, "minX"), viewBounds.minX);
	glUniform1i(glGetUniformLocation(shaderESBlobViz->ID, "minY"), viewBounds.minY);
	glUniform1i(glGetUniformLocation(shaderESBlobViz->ID, "maxX"), viewBounds.maxX);
	glUniform1i(glGetUniformLocation(shaderESBlobViz->ID, "maxY"), viewBounds.maxY);
	SSQuad->draw();

	// Visualize detected blobs
	std::vector<Point> vizPoints;
#ifdef BLOB_VIZ_DOTS
	for (int i = 0; i < blobs.size(); i++)
	{
		for (int j = 0; j < blobs[i].dots.size(); j++)
		{
			vizPoints.push_back({
				.X = (((float)blobs[i].dots[j].X + 0.5f - viewBounds.minX) / (viewBounds.maxX-viewBounds.minX) - 0.5f) * 2.0f,
				.Y = (((float)blobs[i].dots[j].Y + 0.5f - viewBounds.minY) / (viewBounds.maxY-viewBounds.minY) - 0.5f) * 2.0f,
				.S = 4.0f
			});
		}
	}
#else
	vizPoints.resize(blobs.size());
	for (int i = 0; i < vizPoints.size(); i++)
	{
		vizPoints[i].X = ((blobs[i].centroid.X - viewBounds.minX) / (viewBounds.maxX-viewBounds.minX) - 0.5f) * 2.0f;
		vizPoints[i].Y = ((blobs[i].centroid.Y - viewBounds.minY) / (viewBounds.maxY-viewBounds.minY) - 0.5f) * 2.0f;
		vizPoints[i].S = blobs[i].centroid.S / pixelDensity;
	}
#endif
	// Render viz points
	shaderESPoint->use();
	glBindBuffer(GL_ARRAY_BUFFER, vizPointsVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * vizPoints.size(), &vizPoints[0], GL_STREAM_DRAW);
	glVertexAttribPointer(vPosAdr, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, (void *)0);
	glEnableVertexAttribArray(vPosAdr);
	glDrawArrays(GL_POINTS, 0, vizPoints.size());
}

/*
 * Looks up the color of the specified set of points and puts them in the colors list
 */
void blobColorLookup (const std::vector<Point> &points, std::vector<Color> &colors)
{
	// TODO:
	// Probably simple fetch shader pass on a 1D texture
	// Upload a 2xn texture from CPU detailing the UV coordinates to be fetched
	// Create a 1xn texture render target for results
	// Perform shader pass, taking UVs from source texture and putting color in target
	// Download target texture from GPU and populate colors array
	// Try HARD to do this asynchronously if at all possible!
}

/*
 * Destroys resources used for blob detection
 */
void cleanBlobDetection()
{
	// Meshes
	delete SSQuad;
	// Shaders
	delete shaderESBlobDetectRGB;
	delete shaderESBlobDetectY;
	delete shaderESBlobDetectYUV;
	delete shaderESBlobEncode;
	delete shaderESBlobViz;
	delete shaderESPoint;
	delete shaderCamBlitRGB;
	delete shaderCamBlitYUV;
	delete shaderCamBlitY;
	// Render Targets
	delete blobMask;
	delete blobMap;
	// Buffers
#ifdef USE_READ_PIXELS
	delete blobMapRegions;
#endif
	delete blobCompMerge;
	delete mapRowZeros;
	glDeleteBuffers(1, &vizPointsVBO);
}

/* Bind external EGL tex to adr using specified texture slot */
static void bindExternalTexture (GLuint adr, GLuint tex, int slot)
{
	glUniform1i(adr, slot);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, tex);
}

static CompID resolveComponentMerge(CompID compID)
{
	CompID id = blobCompMerge[compID];
	if (id != blobCompMerge[id])
	{ // Resolve recursive merge hierarchy
		while (id != blobCompMerge[id])
		{
			id = blobCompMerge[id];
		}
		// Update all components on the way
		CompID idIt = compID, idNxt;
		while ((idNxt = blobCompMerge[idIt]) != id)
		{
			blobCompMerge[idIt] = id;
			idIt = idNxt;
		}
	}
	return id;
}
