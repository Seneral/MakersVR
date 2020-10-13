/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "util.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <fstream>
#include <iomanip>

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

	if (cfg.contains("mode"))
	{
		auto mode = cfg["mode"];
		if (mode.contains("cameraResolutionX"))
			config->mode.cameraResolutionX = mode["cameraResolutionX"].get<int>();
		if (mode.contains("cameraResolutionY"))
			config->mode.cameraResolutionY = mode["cameraResolutionY"].get<int>();
		if (mode.contains("cameraFramerate"))
			config->mode.cameraFramerate = mode["cameraFramerate"].get<int>();
	}

	if (cfg.contains("tracking"))
	{
		auto tracking = cfg["tracking"];
		if (tracking.contains("intersectError"))
			config->tracking.intersectError = tracking["intersectError"].get<float>();
		if (tracking.contains("sigmaError"))
			config->tracking.sigmaError = tracking["sigmaError"].get<float>();
	}

	if (cfg.contains("testsetup"))
	{
		auto testsetup = cfg["testsetup"];
		if (testsetup.contains("blobPxStdDev"))
		{
			config->testing.blobPxStdDev = testsetup["blobPxStdDev"].get<float>();
		}
		int defaultFoV = 160;
		if (testsetup.contains("defaultFoV") )
		{
			auto default = testsetup["defaultFoV"];
			if (default.is_number_integer())
				defaultFoV = default;
		}
		if (testsetup.contains("calibrationMarkers"))
		{
			auto markers = testsetup["calibrationMarkers"];
			if (markers.is_array())
			{
				for (auto& md : markers)
				{
					if (md.is_string())
						parseMarkerDataFile(md, config->testing.calibrationMarkers, defaultFoV);
					else if (md.is_object())
					{
						int FoV = md.contains("FoV") && md["FoV"].is_number_integer()? md["FoV"] : defaultFoV;
						parseMarkerDataFile(md["path"], config->testing.calibrationMarkers, FoV);
					}
				}
			}
			else if (markers.is_string())
				parseMarkerDataFile(markers, config->testing.calibrationMarkers, defaultFoV);
		}
		if (testsetup.contains("trackingMarkers"))
		{
			auto markers = testsetup["trackingMarkers"];
			if (markers.is_array())
			{
				for (auto& md : markers)
				{
					if (md.is_string())
						parseMarkerDataFile(md, config->testing.trackingMarkers, defaultFoV);
					else if (md.is_object())
					{
						int FoV = md.contains("FoV") && md["FoV"].is_number_integer()? md["FoV"] : defaultFoV;
						parseMarkerDataFile(md["path"], config->testing.trackingMarkers, FoV);
					}
				}
			}
			else if (markers.is_string())
				parseMarkerDataFile(markers, config->testing.trackingMarkers, defaultFoV);
		}
		if (testsetup.contains("cameras"))
		{
			auto cameras = testsetup["cameras"];
			if (cameras.is_array())
			{
				for (auto& cam : cameras) {
					if (cam.is_object())
					{
						DefCamera camera = {};
						// Parse position
						Eigen::Vector3f pos;
						sscanf(cam["position"].get<std::string>().c_str(), "%f %f %f", &pos.x(), &pos.y(), &pos.z());
						camera.pos = pos * 100; // m to cm
						// Parse rotation
						Eigen::Vector3f rotEA;
						sscanf(cam["rotation"].get<std::string>().c_str(), "%f %f %f", &rotEA.x(), &rotEA.y(), &rotEA.z());
						camera.rot = getRotationXYZ(rotEA / 180.0f * PI);
						// Parse distortions
						if (cam.contains("distortion") && cam["distortion"].is_string())
						{
							sscanf(cam["distortion"].get<std::string>().c_str(), "%f %f %f %f %f",
								&camera.distortion[0], &camera.distortion[1], &camera.distortion[2], &camera.distortion[3], &camera.distortion[4]);
						}
						// Parse fov
						if (cam.contains("fov"))
						{
							if (cam["fov"].is_string())
							{
								sscanf(cam["fov"].get<std::string>().c_str(), "%f %f", &camera.fovH, &camera.fovV);
							}
							else if (cam["fov"].is_object())
							{
								float sensorX = cam["fov"]["sensorSizeX"].get<float>();
								float sensorY = cam["fov"]["sensorSizeY"].get<float>();
								float focalLen = cam["fov"]["focalLength"].get<float>();
								camera.fovH = 2 * std::atan(sensorX/focalLen/2) / PI * 180.0f;
								camera.fovV = 2 * std::atan(sensorY/focalLen/2) / PI * 180.0f;
							}
						}
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
bool parseMarkerDataFile(std::string path, std::vector<DefMarker> &markers, int fov)
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
			pt.fov = (float)fov;
			curGroup->points.push_back(pt);
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
			markers.push_back(group.second);
		}
	}
	return true;
}


/**
 * Parses the given calibration
 */
void parseCalibrationFile(std::string path, std::vector<Camera> &cameraCalib)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json calib;
	fs >> calib;

	// Parse JSON calib file
	if (calib.contains("cameras"))
	{
		auto cameras = calib["cameras"];
		if (cameras.is_array())
		{
			cameraCalib.resize(cameras.size());
			int i = 0;
			for (auto& cam : cameras)
			{
				Camera *clb = &cameraCalib[i++];
				// Parse intrinsic calibration
				clb->fovH = cam["fovH"].get<float>();
				clb->fovV = cam["fovV"].get<float>();
				if (cam.contains("distortion"))
				{
					auto distortion = cam["distortion"];
					if (distortion.is_object())
					{
						auto *dst = &clb->distortion;
						dst->k1 = distortion["k1"].get<float>();
						dst->k2 = distortion["k2"].get<float>();
						dst->k3 = distortion["k3"].get<float>();
						dst->p1 = distortion["p1"].get<float>();
						dst->p2 = distortion["p2"].get<float>();
					}
				}
				// Parse extrinsic calibration
				if (cam.contains("position"))
				{
					auto position = cam["position"];
					if (position.is_object())
					{
						clb->transform.translation().x() = position["x"].get<float>();
						clb->transform.translation().y() = position["y"].get<float>();
						clb->transform.translation().z() = position["z"].get<float>();
					}
				}
				if (cam.contains("rotation"))
				{
					auto rotation = cam["rotation"];
					if (rotation.is_object())
					{
						Eigen::Vector3f rotEA;
						rotEA.x() = rotation["x"].get<float>();
						rotEA.y() = rotation["y"].get<float>();
						rotEA.z() = rotation["z"].get<float>();
						clb->transform.linear() = getRotationXYZ(rotEA / 180.0f * PI);
					}
				}
			}
		}
	}
}

/**
 * Writes the given calibration to file
 */
void writeCalibrationFile(std::string path, std::vector<Camera> &cameraCalib)
{
	json calib;

	// Write JSON calib file
	calib["cameras"] = json::array();
	for (int c = 0; c < cameraCalib.size(); c++)
	{
		Camera *clb = &cameraCalib[c];
		json cam;
		// Write intrinsic calibration
		cam["fovH"] = clb->fovH;
		cam["fovV"] = clb->fovV;
		auto *dst = &clb->distortion;
		cam["distortion"]["k1"] = dst->k1;
		cam["distortion"]["k2"] = dst->k2;
		cam["distortion"]["k3"] = dst->k3;
		cam["distortion"]["p1"] = dst->p1;
		cam["distortion"]["p2"] = dst->p2;
		// Write extrinsic calibration
		Eigen::Vector3f pos = clb->transform.translation();
		cam["position"]["x"] = pos.x();
		cam["position"]["y"] = pos.y();
		cam["position"]["z"] = pos.z();
		Eigen::Vector3f rot = getEulerXYZ(clb->transform.rotation()) / PI * 180.0f;
		cam["rotation"]["x"] = rot.x();
		cam["rotation"]["y"] = rot.y();
		cam["rotation"]["z"] = rot.z();
		calib["cameras"].push_back(cam);
	}

	// Write JSON config file
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::setw(4) << calib;
}