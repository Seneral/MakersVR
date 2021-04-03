/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "blobdetection.hpp"

#include <cmath>
#include <map>
#include <cstring>
#include <cassert>
#include <algorithm>

#include <interface/vcsm/user-vcsm.h>

//#define BLOB_DEBUG
//#define BLOB_TRACE

#define CALCULATE_BOUNDS_EARLY
#define BLOB_MERGE_SATELLITES
#define BLOB_POST_PASS


// Accessors for 8x4 tile encoded in 32bit integer
#define DOTID(X, Y) ((((X)%4)*8)+(((X)>>2)&1)+((Y)*2))
#define DOT(BYTES, X, Y) (uint8_t)(((BYTES) >> DOTID(X,Y)) & 1)

// Component data config
typedef uint8_t CompID; // Lower -> significantly better performance (cache size?), but limits MaxCompID
const CompID MaxCompID = 256-1; // Max intermediate component ID - higher than final merged component count
typedef uint16_t TileID; // Lower->better performance (cache size?), but limits MaxCompID
const TileID MaxTileID = 2048-1; // Max regarded tile count, highest reserved for none (-1)

/* Structures  */

// Tile in blob mask
typedef uint32_t MaskTile;
// Tile as used for processing
typedef struct
{ // This should be packed by default
	uint16_t x, y;
	TileID t, l;
	MaskTile bytes;
	CompID compMap[8][4];
} Tile;
// Additional data for components
typedef struct
{
	uint32_t x, y;
	uint16_t ptCnt;
#ifdef CALCULATE_BOUNDS_EARLY
	Bounds bounds;
#endif
} Comp;

bool ClusterLargeFirst(const Cluster& a, const Cluster& b) { return a.ptCnt > b.ptCnt; }

/* Variables */

// Sizes
static int maskW, maskH, tileW, tileH, blkH;
// Blob offset to compensate for padding
static int blobOffsetX, blobOffsetY;
// Relative pixels gap at which two blobs are considered one and merged
//static int blobMergeBorder = 4;
// Dynamic buffer for all 8x4 tiles that are part of a blob
static std::vector<Tile> blobTiles;
// Shared buffer serving as a map from initial component ID to merged component ID
static CompID *blobCompMerge;
// Shared buffer serving as a data accumulator while components are processed, used in conjunction with merge map
static Comp *blobCompData;
// Map from initial blob to merged blob index
static CompID *blobMerge;
// Temporary cluster list
static std::vector<Cluster> tempBlobs;

/* Local Functions */

static inline CompID resolveComponentMerge(CompID compID);

/*
 * Initialize resources required for blob detection
 */
bool initBlobDetection (int width, int height, int offsetX, int offsetY)
{
	maskW = width;
	maskH = height;
	tileW = maskW / 8;
	tileH = maskH / 4;
	blkH = tileH / 5;
	blobOffsetX = offsetX;
	blobOffsetY = offsetY;

	// Adapt blob merge border to resolution
//	blobMergeBorder = blobMergeBorder * (maskW/512);

	// Setup resources used during blob detection
	blobTiles.reserve(256);
	blobCompMerge = (CompID*)malloc((MaxCompID+1) * sizeof(CompID));
	blobCompData = (Comp*)malloc((MaxCompID+1) * sizeof(Comp));
	blobMerge = (CompID*)malloc((MaxCompID+1) * sizeof(CompID));

	return true;
}

/*
 * Reads back blobMap from the GPU into the specified buffer, ready for analysation on the CPU
 */
void performBlobDetectionRegionsFetch(uint32_t *frame)
{
//	int xMin = 0, xMax = tileW, yMin = 0, yMax = blkH;
	int bufW = tileW*5;

	blobTiles.clear();
	MaskTile *maskTiles = (MaskTile*)frame;

	// Read map and extract mask tiles with dots (1s) as blob tiles and enter them in a list

	auto registerBlobTile = [&bufW, &maskTiles](MaskTile *ptr)
	{
		int tileID = ptr-maskTiles;
		int blkID = tileID / 5;
		int blkRow = tileID % 5;
		int blkX = blkID % tileW;
		int blkY = blkID / tileW;
		uint16_t x = blkX;
		uint16_t y = blkY*5 + blkRow;
		TileID t = -1;
		TileID l = -1;
		// Now find any tiles to the top or left quickly
		if (blkRow > 0)
		{ // Quick way of finding top tile (bounded by 1)
			if (*(ptr-1))
				t = blobTiles.size()-1;
		}
		else if (blkY > 0)
		{ // Very slow way, have to go to top row (bounded by bufW, 400 for 480p)
			if (*(ptr-bufW+4))
			{ // Have a top tile
				TileID max = blobTiles.size()-1;
				for (int i = 0; i < tileW; i++)
				{
					if (blobTiles[max-i].x == x && blobTiles[max-i].y == y-1)
					{
						t = max-i;
						break;
					}
				}
			}
		}
		if (x > 0)
		{ // Relatively fast way of finding left region (bounded by 5)
			if (*(ptr-5))
			{ // Have a left tile
				TileID max = blobTiles.size()-1;
				for (int i = 0; i < 5; i++)
				{
					if (blobTiles[max-i].y == y && blobTiles[max-i].x == x-1)
					{
						l = max-i;
						break;
					}
				}
			}
		}
//		printf("Added region %d (%d, %d) with top %d and left %d\n", pos, x, y, t, l)
		blobTiles.push_back({ x, y, t, l, *ptr });
	};

	uint64_t *max = (uint64_t*)maskTiles+(tileW*tileH/2);
	uint64_t *ptr = (uint64_t*)__builtin_assume_aligned(maskTiles, 4096); // Not sure if this helps much
	for (; ptr < max; ptr++)
//	while (ptr < max)
//	#pragma GCC unroll 20
//	for (int i = 0; i < 16*5/2; i++, ptr++) // Possible slight optimization? assuming the buffer is a multiple of 80 blocks
	{
		if (*ptr)
		{ // Found dots in 64-Bit region, check each 32-Bit tile individually
			MaskTile *tile = (MaskTile*)ptr;
			if (*(tile+0)) registerBlobTile(tile+0);
			if (*(tile+1)) registerBlobTile(tile+1);
			if (blobTiles.size() >= MaxTileID-1)
				break;
		}
	}

#ifdef BLOB_DEBUG
		printf("Found %d regions with blobs!\n", blobTiles.size());
#endif
}

/*
 * Analyses the regions detected in the last GPU step and outputs detected blobs into target array
 */
void performBlobDetectionCPU(std::vector<Cluster> &blobs)
{
	blobCompMerge[0] = 0; // Setup 'No Component' ID

	CompID compNum = 0; // Number of final components
	CompID compIndex = 0; // Number of intermediary components

	// Iterate over blob regions do connected component labeling
	for (int i = 0; i < blobTiles.size(); i++)
	{
		Tile *tile = &blobTiles[i];
		Tile *topTile = (tile->t != (uint16_t)-1)? &blobTiles[tile->t] : nullptr;
		Tile *leftTile = (tile->l != (uint16_t)-1)? &blobTiles[tile->l] : nullptr;

#ifdef BLOB_TRACE
		printf("Tile %d / %d: T?%c, L?%c! -- State: %d(%d) components!\n", tile->x, tile->y, topTile? 'y' : 'n', leftTile? 'y' : 'n', compNum, compIndex);
#endif

		if (tile->bytes == 0xFFFFFFFF)
		{ // Optimization on large blobs
			auto updateComp = [&compNum](CompID compID, CompID comp)
			{ // Update compID with comp and merge to smallest if needed
				if (comp == 0) return compID;
				comp = resolveComponentMerge(comp);
				if (compID == 0) compID = comp;
				else if (comp < compID)
				{ // Merge compID with comp
					blobCompMerge[compID] = comp;
					compNum--;
					compID = comp;
				}
				else if (comp > compID)
				{ // Merge comp with CompID
					blobCompMerge[comp] = compID;
					compNum--;
				}
				return compID;
			};
			// Merge components among shared border with left and top
			CompID compID = 0;
			if (leftTile)
			{
				for (int y = 0; y < 4; y++)
					compID = updateComp(compID, leftTile->compMap[7][y]);
			}
			if (topTile)
			{
				for (int x = 0; x < 8; x++)
					compID = updateComp(compID, topTile->compMap[x][3]);
			}
			// If none shared, create new component
			if (compID == 0 && compIndex < MaxCompID)
			{
				compID = ++compIndex;
				blobCompMerge[compID] = compID;
				blobCompData[compID] = { 0, 0, 0
#ifdef CALCULATE_BOUNDS_EARLY
					, { (uint16_t)maskW, 0, (uint16_t)maskH, 0 }
#endif
				};
				compNum++;
#ifdef BLOB_TRACE
				printf("Assigning new component ID %d!\n", (int)compID);
#endif
			}
			// Apply merged component to all cells
			for (int x = 0; x < 8; x++)
			for (int y = 0; y < 4; y++)
				tile->compMap[x][y] = compID;
			// Update component data with new dots
			Comp *comp = &blobCompData[compID];
			comp->x += (tile->x*8*8 + 28)*4;
			comp->y += (tile->y*4*4 + 6)*8;
			comp->ptCnt += 8*4;
#ifdef CALCULATE_BOUNDS_EARLY
			// Update bounds
			comp->bounds.minX = std::min(comp->bounds.minX, (uint16_t)(tile->x*8));
			comp->bounds.minY = std::min(comp->bounds.minY, (uint16_t)(tile->y*4));
			comp->bounds.maxX = std::max(comp->bounds.maxX, (uint16_t)(tile->x*8+7));
			comp->bounds.maxY = std::max(comp->bounds.maxY, (uint16_t)(tile->y*4+3));
#endif
			continue;
		}

		//#pragma GCC unroll n
		// Do connected component labelling within 8x4 tile using connected components from top and left tiles
		for (int x = 0; x < 8; x++)
		{
			CompID top = topTile? topTile->compMap[x][3] : 0;

			for (int y = 0; y < 4; y++)
			{
				CompID compID = 0;
				if (DOT(tile->bytes, x, y))
				{ // Dot at current pixel
					CompID left = 0;
					if (x > 0) left = tile->compMap[x-1][y];
					else if (leftTile) left = leftTile->compMap[7][y];

					if (top != 0 && left != 0)
					{ // Two connected dots, assign and make sure both are merged
						CompID topComp = resolveComponentMerge(top);
						CompID leftComp = resolveComponentMerge(left);
						compID = topComp;

						if (leftComp != topComp)
						{ // Separated components connected through this dot, merge components
							if (topComp > leftComp)
								blobCompMerge[topComp] = compID = leftComp;
							else
								blobCompMerge[leftComp] = compID = topComp;
							compNum--;
#ifdef BLOB_TRACE
							printf("Merging %d (%d) into %d (%d) at pos %d/%d!\n", left, leftComp, top, topComp, x, y);
#endif
						}
					}
					else if (top == 0 && left == 0)
					{ // No connected components from top or left, assign new component
						if (compIndex < MaxCompID)
						{
							compID = ++compIndex;
							blobCompMerge[compID] = compID;
							blobCompData[compID] = { 0, 0, 0
#ifdef CALCULATE_BOUNDS_EARLY
								, { (uint16_t)maskW, 0, (uint16_t)maskH, 0 }
#endif
							};
							compNum++;
#ifdef BLOB_TRACE
							printf("Assigning new component ID %d!\n", (int)compID);
#endif
						}
					}
					else
					{ // One connected component to assign to
						compID = resolveComponentMerge(top | left);
					}
					// Update component data with new dot
					uint16_t dotX = tile->x * 8 + x, dotY = tile->y * 4 + y;
					Comp *comp = &blobCompData[compID];
					comp->x += dotX;
					comp->y += dotY;
					comp->ptCnt++;
#ifdef CALCULATE_BOUNDS_EARLY
					// Update bounds
					comp->bounds.minX = std::min(comp->bounds.minX, dotX);
					comp->bounds.minY = std::min(comp->bounds.minY, dotY);
					comp->bounds.maxX = std::max(comp->bounds.maxX, dotX);
					comp->bounds.maxY = std::max(comp->bounds.maxY, dotY);
#endif
				}

				// Assign component ID
				top = compID;
				tile->compMap[x][y] = compID;
			}
		}
	}

	// Compile clusters for merged components
	// Assumption: blobCompMerge[i] <= i -- e.g. the true ID comes before the merged IDs
	int clusterID = 0;
	tempBlobs.clear();
	tempBlobs.reserve(compNum);
	for (int i = 1; i < compIndex+1; i++)
	{
		assert(blobCompMerge[i] <= i);
		if (blobCompMerge[i] == i)
		{ // Claim cluster index
			blobCompMerge[i] = clusterID++;
			blobMerge[blobCompMerge[i]] = blobCompMerge[i];
			tempBlobs.emplace_back(
				(float)blobCompData[i].x, (float)blobCompData[i].y,
#ifdef CALCULATE_BOUNDS_EARLY
				blobCompData[i].bounds,
#else
				(uint16_t)maskW, 0, (uint16_t)maskH, 0,
#endif
				blobCompData[i].ptCnt
			);
		}
		else
		{ // Copy cluster index from previous merged components
			int b = blobCompMerge[i] = blobCompMerge[blobCompMerge[i]];
			tempBlobs[b].centroid.X += blobCompData[i].x;
			tempBlobs[b].centroid.Y += blobCompData[i].y;
			tempBlobs[b].ptCnt += blobCompData[i].ptCnt;
#ifdef CALCULATE_BOUNDS_EARLY
			// Update bounds
			tempBlobs[b].bounds.minX = std::min(tempBlobs[b].bounds.minX, blobCompData[i].bounds.minX);
			tempBlobs[b].bounds.minY = std::min(tempBlobs[b].bounds.minY, blobCompData[i].bounds.minY);
			tempBlobs[b].bounds.maxX = std::max(tempBlobs[b].bounds.maxX, blobCompData[i].bounds.maxX);
			tempBlobs[b].bounds.maxY = std::max(tempBlobs[b].bounds.maxY, blobCompData[i].bounds.maxY);
#endif
		}
	}

	for (int i = 0; i < tempBlobs.size(); i++)
	{
		tempBlobs[i].centroid.X /= tempBlobs[i].ptCnt;
		tempBlobs[i].centroid.Y /= tempBlobs[i].ptCnt;
	}

#ifdef BLOB_DEBUG
	printf("Found %d initial merged components, with max %d, list %d!\n", compNum, compIndex, (int)blobs.size());
#endif

#ifdef BLOB_MERGE_SATELLITES
	// Create sorted mapping
	std::vector<int> sortOrder;
	sortOrder.reserve(tempBlobs.size());
	for (int i = 0; i < tempBlobs.size(); i++)
		sortOrder.push_back(i);
	std::partial_sort(sortOrder.begin(), sortOrder.begin() + std::min(30, (int)sortOrder.size()), sortOrder.end(), [](const int &a, const int &b) -> bool {
		return tempBlobs[a].ptCnt > tempBlobs[b].ptCnt;
	});

	// Finalize clusters
	blobs.clear();
	blobs.reserve(tempBlobs.size());
	for (int i = 0; i < sortOrder.size(); i++)
	{
		int index = sortOrder[i];
		if (index < 0) continue; // Already claimed

		blobMerge[index] = 0;
		if (tempBlobs[index].ptCnt < 2) continue; // Too small

		// Setup as actual blob
		blobs.push_back(tempBlobs[index]);
		blobMerge[index] = blobs.size();
		Cluster *cluster = &blobs[blobMerge[index]-1];

		// Average bound sizes
		//cluster->centroid.S = ((cluster->bounds.maxX-cluster->bounds.minX)+(cluster->bounds.maxY-cluster->bounds.minY))/2;
		// Nice approximation for circular blobs
		cluster->centroid.S = std::sqrt((float)cluster->ptCnt);

		Point circle;
		float extendsXSq, extendsYSq;
		auto recalcArea = [&circle, &extendsXSq, &extendsYSq, cluster]()
		{
			// Extend range factor
			float s = 1.5f * (1.0f + 10.0f/cluster->ptCnt);

#ifdef CALCULATE_BOUNDS_EARLY
			// Find squared bound extends
			extendsXSq = (cluster->bounds.maxX-cluster->bounds.minX)/2 * s;
			extendsXSq = extendsXSq * extendsXSq;
			extendsYSq = (cluster->bounds.maxY-cluster->bounds.minY)/2 * s;
			extendsYSq = extendsYSq * extendsYSq;

			// Maximum bounds regardless of shape
			circle.X = (cluster->bounds.minX+cluster->bounds.maxX)/2;
			circle.Y = (cluster->bounds.minY+cluster->bounds.maxY)/2;
			circle.S = std::max(extendsXSq, extendsYSq);

			// Conservative circle around center of mass
			//circle = cluster->centroid;
			//circle.S = std::max(std::min((float)cluster->bounds.maxX-cluster->centroid.X, cluster->centroid.X-(float)cluster->bounds.minX), std::min((float)cluster->bounds.maxY-cluster->centroid.Y, (float)cluster->centroid.Y-cluster->bounds.minY));
			//circle.S *= s;
			//circle.S = circle.S * circle.S;
#else
			// Use circle approximation
			circle = cluster->centroid;
			circle.S *= s;
			circle.S = circle.S * circle.S;
#endif
		};

		recalcArea();

//		printf("Cluster %d had size %f around %f / %f  with %d dots!\n", i, cluster->centroid.S, cluster->centroid.X, cluster->centroid.Y, (int)cluster->ptCnt);
//		printf("Cluster %d had bounds %d/%d/%d/%d!\n", i, cluster->bounds.minX, cluster->bounds.maxX, cluster->bounds.minY, cluster->bounds.maxY);
//		printf("Cluster %d search around %f / %f of size %f (%f)!\n", i, circle.X, circle.Y, circle.S, cluster->centroid.S);

		int mergeCnt = 0;
		for (int n = 0; n < 2; n++) // Two passes
		for (int j = i+1; j < sortOrder.size(); j++)
		{
			int candIndex = sortOrder[j];
			if (candIndex < 0) continue; // Already claimed
			if ((tempBlobs[candIndex].ptCnt*2 > cluster->ptCnt && cluster->ptCnt > 10) || tempBlobs[candIndex].ptCnt > cluster->ptCnt) continue; // Not small enough to be a satellite

			Cluster *cand = &tempBlobs[candIndex];
			float distX = cand->centroid.X - circle.X;
			float distY = cand->centroid.Y - circle.Y;
			float distXSq = distX * distX;
			float distYSq = distY * distY;
			float distSq = distXSq + distYSq;
//			if (distX < circle.S && distY < circle.S)
			if (distSq < circle.S)
			{
#ifdef CALCULATE_BOUNDS_EARLY
				if (distXSq > extendsXSq || distYSq > extendsYSq)
					continue;
#endif
				// Peform merge
				blobMerge[candIndex] = blobMerge[index];
				sortOrder[j] = -1;
				mergeCnt++;
				// Update centroid
				int totalCnt = cluster->ptCnt + cand->ptCnt;
				cluster->centroid.X = (cluster->centroid.X * cluster->ptCnt + cand->centroid.X * cand->ptCnt) / totalCnt;
				cluster->centroid.Y = (cluster->centroid.Y * cluster->ptCnt + cand->centroid.Y * cand->ptCnt) / totalCnt;
				cluster->ptCnt = totalCnt;
#ifdef CALCULATE_BOUNDS_EARLY
				// Update bounds
				cluster->bounds.minX = std::min(cluster->bounds.minX, cand->bounds.minX);
				cluster->bounds.minY = std::min(cluster->bounds.minY, cand->bounds.minY);
				cluster->bounds.maxX = std::max(cluster->bounds.maxX, cand->bounds.maxX);
				cluster->bounds.maxY = std::max(cluster->bounds.maxY, cand->bounds.maxY);
#endif
				// Update search area
				recalcArea();
			}
#ifdef BLOB_TRACE
			else
			{
				printf("Won't merge with %d, as distance %f/%f is farther than %ff!\n", j, distX, distY, circle.S);
			}
#endif
		}

		cluster->centroid.X += blobOffsetX;
		cluster->centroid.Y += blobOffsetY;
#ifdef CALCULATE_BOUNDS_EARLY
		cluster->bounds.minX += blobOffsetX;
		cluster->bounds.minY += blobOffsetY;
		cluster->bounds.maxX += blobOffsetX;
		cluster->bounds.maxY += blobOffsetY;
#endif

#ifdef BLOB_TRACE
		printf("Cluster %d has size %f around %f / %f  with %d dots, merged from %d satellites!\n", i, cluster->centroid.S, cluster->centroid.X, cluster->centroid.Y, (int)cluster->ptCnt, mergeCnt);
#endif
	}
#else // BLOB_MERGE_SATELLITES
		blobs.swap(tempBlobs);
#endif


#ifdef BLOB_POST_PASS
	// Compile final cluster data (Around half the time is spend here, uselessly arranging dots)
	for (int i = 0; i < blobs.size(); i++)
	{
		Cluster *cluster = &blobs[i];
		cluster->dots.reserve(cluster->ptCnt);
	}
	for (int i = 0; i < blobTiles.size(); i++)
	{
		Tile *tile = &blobTiles[i];
		for (int x = 0; x < 8; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				CompID compID = tile->compMap[x][y];
				if (compID == 0) continue;
				// Dot here
				CompID clusterID = blobMerge[blobCompMerge[compID]];
				if (clusterID == 0) continue;
				Cluster *cluster = &blobs[clusterID-1];
				// Add dot to cluster
				uint16_t dotX = tile->x * 8 + x + blobOffsetX, dotY = tile->y * 4 + y + blobOffsetY;
				cluster->dots.push_back({ dotX, dotY });
#ifndef CALCULATE_BOUNDS_EARLY
				// Update bounds
				cluster->bounds.minX = std::min(cluster->bounds.minX, dotX);
				cluster->bounds.minY = std::min(cluster->bounds.minY, dotY);
				cluster->bounds.maxX = std::max(cluster->bounds.maxX, dotX);
				cluster->bounds.maxY = std::max(cluster->bounds.maxY, dotY);
#endif
			}
		}
	}
#endif

#ifdef BLOB_DEBUG
	printf("Found %d blobs from %d initial blobs!\n", blobs.size(), blobTemp.size());
#endif
}

/*
 * Looks up the color of the specified set of points and puts them in the colors list
 */
void blobColorLookup (const std::vector<Point> &points, std::vector<Color> &colors)
{
}

/*
 * Destroys resources used for blob detection
 */
void cleanBlobDetection()
{
	delete blobCompMerge;
	delete blobCompData;
	delete blobMerge;
}

static inline CompID resolveComponentMerge(CompID compID)
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
