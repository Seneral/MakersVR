/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "config.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <fstream>
#include <iomanip>

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
		config->mode.cameraResolutionX = mode["cameraResolutionX"].empty()? 640 : mode["cameraResolutionX"].get<int>();
		config->mode.cameraResolutionY = mode["cameraResolutionY"].empty()? 480 : mode["cameraResolutionY"].get<int>();
		config->mode.cameraFramerate = mode["cameraFramerate"].empty()? 10 : mode["cameraFramerate"].get<int>();
		config->mode.cameraShutterSpeed = mode["cameraShutterSpeed"].empty()? 500 : mode["cameraShutterSpeed"].get<int>();
		config->mode.cameraGain = mode["cameraGain"].empty()? 2 : mode["cameraGain"].get<int>();
		config->mode.cameraAbsThreshold = mode["cameraAbsThreshold"].empty()? 10 : mode["cameraAbsThreshold"].get<int>();
		config->mode.cameraEdgeThreshold = mode["cameraEdgeThreshold"].empty()? 6 : mode["cameraEdgeThreshold"].get<int>();
	}

	if (cfg.contains("tracking"))
	{
		auto tracking = cfg["tracking"];
		config->tracking.minIntersectError = tracking["minIntersectError"].empty()? 0.01f : tracking["minIntersectError"].get<float>();
		config->tracking.maxIntersectError = tracking["maxIntersectError"].empty()? 0.10f : tracking["maxIntersectError"].get<float>();
		config->tracking.maxTemporalStaticErrorT = tracking["maxTemporalStaticErrorT"].empty()? 10.0f : tracking["maxTemporalStaticErrorT"].get<float>();
		config->tracking.maxTemporalStaticErrorR = tracking["maxTemporalStaticErrorR"].empty()? 10.0f : tracking["maxTemporalStaticErrorR"].get<float>();
		config->tracking.maxTemporalDynamicErrorT = tracking["maxTemporalDynamicErrorT"].empty()? 0.5f : tracking["maxTemporalDynamicErrorT"].get<float>();
		config->tracking.maxTemporalDynamicErrorR = tracking["maxTemporalDynamicErrorR"].empty()? 0.5f : tracking["maxTemporalDynamicErrorR"].get<float>();
		config->tracking.sigmaError = tracking["sigmaError"].empty()? 3.0f : tracking["sigmaError"].get<float>();
	}

	auto parseIterativeParamFloat = [](json cfg)
	{
		IterativeParam<float> param = { 0, 1, 0 };
		if (cfg.is_object())
		{
			if (cfg.contains("start")) param.start = cfg["start"];
			if (cfg.contains("factor")) param.factor = cfg["factor"];
			if (cfg.contains("summand")) param.summand = cfg["summand"];
		}
		else param.start = cfg;
		return param;
	};
	auto parseIterativeParamInt = [](json cfg)
	{
		IterativeParam<int> param = { 0, 1, 0 };
		if (cfg.is_object())
		{
			if (cfg.contains("start")) param.start = cfg["start"];
			if (cfg.contains("factor")) param.factor = cfg["factor"];
			if (cfg.contains("summand")) param.summand = cfg["summand"];
		}
		else param.start = cfg;
		return param;
	};

	if (cfg.contains("intrinsicCalib"))
	{
		auto intrinsic = cfg["intrinsicCalib"];
		config->intrinsicCalib.selectionThreshold = parseIterativeParamFloat(intrinsic["selectionThreshold"]);
		config->intrinsicCalib.maxMarkerCount = parseIterativeParamInt(intrinsic["maxMarkerCount"]);
		config->intrinsicCalib.radialDensityGranularity = parseIterativeParamFloat(intrinsic["radialDensityGranularity"]);
		config->intrinsicCalib.radialDensityTarget = parseIterativeParamFloat(intrinsic["radialDensityTarget"]);
		config->intrinsicCalib.gridSize = intrinsic["gridSize"];
		config->intrinsicCalib.gridCountTarget = parseIterativeParamInt(intrinsic["gridCountTarget"]);
	}

	if (cfg.contains("extrinsicCalib"))
	{
		auto extrinsic = cfg["extrinsicCalib"];
		config->extrinsicCalib.maxRelationCandidateDiffT = extrinsic["maxRelationCandidateDiffT"].empty()? 20 : extrinsic["maxRelationCandidateDiffT"].get<float>();
		config->extrinsicCalib.maxRelationCandidateDiffR = extrinsic["maxRelationCandidateDiffR"].empty()? 5 : extrinsic["maxRelationCandidateDiffR"].get<float>();
		config->extrinsicCalib.maxOriginCandidateDiffT = extrinsic["maxOriginCandidateDiffT"].empty()? 5 : extrinsic["maxOriginCandidateDiffT"].get<float>();
		config->extrinsicCalib.maxOriginCandidateDiffR = extrinsic["maxOriginCandidateDiffR"].empty()? 2 : extrinsic["maxOriginCandidateDiffR"].get<float>();
		config->extrinsicCalib.maxPoseMSE = extrinsic["maxPoseMSE"].empty()? 1 : extrinsic["maxPoseMSE"].get<float>();
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
			auto fov = testsetup["defaultFoV"];
			if (fov.is_number_integer())
				defaultFoV = fov.get<int>();
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
						int FoV = md.contains("FoV") && md["FoV"].is_number_integer()? md["FoV"].get<int>() : defaultFoV;
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
						int FoV = md.contains("FoV") && md["FoV"].is_number_integer()? md["FoV"].get<int>() : defaultFoV;
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
						Camera camera = {};
						// Parse position
						Eigen::Vector3f pos;
						sscanf(cam["position"].get<std::string>().c_str(), "%f %f %f", &pos.x(), &pos.y(), &pos.z());
						camera.transform.translation() = pos * 100; // m to cm
						// Parse rotation
						Eigen::Vector3f rotEA;
						sscanf(cam["rotation"].get<std::string>().c_str(), "%f %f %f", &rotEA.x(), &rotEA.y(), &rotEA.z());
						camera.transform.linear() = getRotationXYZ(rotEA / 180.0f * PI);
						// Parse distortions
						if (cam.contains("distortion") && cam["distortion"].is_string())
						{
							sscanf(cam["distortion"].get<std::string>().c_str(), "%f %f %f %f %f",
								&camera.distortion.k1, &camera.distortion.k2, &camera.distortion.p1, &camera.distortion.p2, &camera.distortion.k3);
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
void parseCalibrationFile(std::string path, std::vector<Camera> &cameraCalib, std::vector<MarkerTemplate3D> &markerTemplates)
{
	// Read JSON config file
	std::ifstream fs(path);
	if (!fs.is_open()) return;
	json calib;
	fs >> calib;

	// Parse camera calibration
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

	// Parse marker calibration
	if (calib.contains("markers"))
	{
		auto markers = calib["markers"];
		if (markers.is_array())
		{
			markerTemplates.resize(markers.size());
			int i = 0;
			for (auto& marker : markers)
			{
				MarkerTemplate3D *mrk = &markerTemplates[i++];
				if (marker.contains("id"))
				{
					mrk->id = marker["id"].get<int>();
				}
				if (marker.contains("points"))
				{
					auto points = marker["points"];
					if (points.is_array())
					{
						mrk->points.resize(points.size());
						int j = 0;
						for (auto& point : points)
						{
							if (point.is_object())
							{
								mrk->points[j++] = Eigen::Vector3f(
									point["x"].get<float>(),
									point["y"].get<float>(),
									point["z"].get<float>());
							}
						}
					}
				}
			}
		}
	}
}

/**
 * Writes the given calibration to file
 */
void writeCalibrationFile(std::string path, const std::vector<Camera> &cameraCalib, const std::vector<MarkerTemplate3D> &markerTemplates)
{
	json calib;

	// Write camera calibration
	calib["cameras"] = json::array();
	for (int c = 0; c < cameraCalib.size(); c++)
	{
		const Camera *clb = &cameraCalib[c];
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

	// Write marker calibration
	calib["markers"] = json::array();
	for (int m = 0; m < markerTemplates.size(); m++)
	{
		const MarkerTemplate3D *mrk = &markerTemplates[m];
		json marker;
		marker["id"] = mrk->id;
		// Write points
		marker["points"] = json::array();
		for (int p = 0; p < mrk->points.size(); p++)
		{
			json point;
			point["x"] = mrk->points[p].x();
			point["y"] = mrk->points[p].y();
			point["z"] = mrk->points[p].z();
			marker["points"].push_back(point);
		}
		calib["markers"].push_back(marker);
	}

	// Write JSON calib file
	std::ofstream fs(path);
	if (!fs.is_open()) return;
	fs << std::setw(4) << calib;
}