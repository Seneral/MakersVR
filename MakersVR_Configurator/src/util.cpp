/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "util.h"
#include "eigenutil.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <algorithm>
#include <iostream>
#include <fstream>
#include <map>

/**
 * Updates the given statistical value with the new value
 */
void UpdateStatValue(StatValue *stat, double cur)
{
	if (stat->num == 0) stat->start = cur;
	stat->min = std::min(stat->min, cur);
	stat->max = std::max(stat->max, cur);
	stat->avg = (stat->avg * stat->num + cur) / (stat->num + 1);
	stat->diff += (abs(cur - stat->avg) - stat->diff) / (stat->num + 1);	
	stat->num++;
	stat->cur = cur;
}

/**
 * Parses the configuration file config.json
 */
void parseConfigFile(std::string path, Config *config)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json cfg;
	fs >> cfg;

	// Parse JSON config file
	if (cfg.contains("testsetup"))
	{
		auto testsetup = cfg["testsetup"];
		if (testsetup.contains("markers"))
		{
			auto markers = testsetup["markers"];
			if (markers.is_array())
			{
				for (auto& md : markers) {
					if (md.is_string()) config->testing.markerDefinitionFiles.push_back(md);
				}
			}
			else if (markers.is_string())
			{
				config->testing.markerDefinitionFiles.push_back(markers);
			}
		}
		if (testsetup.contains("cameras"))
		{
			auto cameras = testsetup["cameras"];
			if (cameras.is_array())
			{
				for (auto& cam : cameras) {
					if (cam.is_object())
					{
						DefCamera camera;
						// Parse position
						std::string posStr = cam["position"].get<std::string>();
						Eigen::Vector3f pos;
						sscanf(posStr.c_str(), "%f %f %f", &pos.x(), &pos.y(), &pos.z());
						camera.pos = pos * 100; // m to cm
						// Parse rotation
						std::string rotStr = cam["rotation"].get<std::string>();
						Eigen::Vector3f rotEA;
						sscanf(rotStr.c_str(), "%f %f %f", &rotEA.x(), &rotEA.y(), &rotEA.z());
						camera.rot = getRotationXYZ(rotEA / 180.0f * PI);
						// Parse label and assign
						camera.label = cam["label"].get<std::string>();
						config->testing.cameraDefinitions.push_back(camera);
					}
				}
			}
		}
	}
}

/**
 * Parses a Marker Definition from a .obj file
 */
bool parseMarkerDataFile(std::string path, std::vector<DefMarker> *markers)
{
	std::vector<Eigen::Vector3f> verts;
	std::vector<Eigen::Vector3f> nrms;

	std::ifstream fs(path);
	if (!fs.is_open()) return false;

	std::map<std::string, DefMarker> groups = { { path, {} } };
	DefMarker *curGroup = &groups[path];

	while (true)
	{
		// Read line header if possible
		std::string header;
		if (!(fs >> header)) break;
		// Handle line of data
		if (header == "v")
		{ // Read vertex
			Eigen::Vector3f vert;
			fs >> vert.x();
			fs >> vert.z();
			fs >> vert.y();
			verts.push_back(vert);
		}
		else if (header == "vn")
		{ // Read normal
			Eigen::Vector3f nrm;
			fs >> nrm.x();
			fs >> nrm.z();
			fs >> nrm.y();
			nrms.push_back(nrm);
		}
		else if (header == "f")
		{ // Read face
			std::getline(fs, header);
			DefMarkerPoint pt { Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 120.0f };
			int pos = 0, sz = 0, count = 0;
			int vID, nID;
			while(sscanf(header.c_str() + pos, "%d//%d%n", &vID, &nID, &sz) == 2)
			{ // Sum vert values
				pos += sz;
				count++;
				pt.pos += verts.at(vID-1);
				pt.nrm += nrms.at(nID-1);
			}
			if (count == 0) return false;
			if (count < 3) continue; // Failed to read face, probably because UVs were exported
			// Calculate face center as average
			pt.pos = pt.pos * 100.0f/count;
			pt.nrm.normalize();
			pt.fov = 160;
			curGroup->pts.push_back(pt);
		}
		else if (header == "g")
		{ // Read next group name and assign current group
			std::getline(fs >> std::ws, header);
			curGroup = &groups[header];
		}
		else
		{ // Skip line
			std::getline(fs, header);
		}
	}

	// Turn vertex groups found into markers
	for (auto& group : groups)
	{
		if (group.first != "Base" && group.first != "(null)" && (groups.size() == 1 || group.first != path))
		{ // Given vertex group (polygroup) is a set of markers
			group.second.label.assign(group.first);
			markers->push_back(group.second);
		}
	}
	return true;
}

