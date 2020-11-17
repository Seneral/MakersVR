/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include "eigenutil.hpp"
#include "config.hpp" // Config
#include "control.hpp" // Phase control
#include "comm.hpp" // USB Device comm

#include <thread>

enum ConfiguratorMode
{
	MODE_None = 0,
	MODE_Device,
	MODE_Testing
};

struct ConfiguratorState
{
	Config config = {};
	ConfiguratorMode mode = MODE_None;
	ControlState control = {};
	CommState comm = { NULL, { false }, { false }, { false }, { false }, NULL, NULL, NULL };
	// Device Camera Mapping
	int cameraCount = 0;
	std::vector<int> cameraMapping = {}; // Device port Index -> Camera index
	// Comm thread
	std::atomic<bool> runCommThread = false;
	std::thread *commThread = NULL;
	// Test thread
	std::atomic<bool> runTestThread = false;
	std::thread *testThread = NULL;
	// Testing state
	Eigen::Vector3f TGT = Eigen::Vector3f(0,0,100);
	Eigen::Vector3f RGT = Eigen::Vector3f::Zero();
	Eigen::Vector3f TD = Eigen::Vector3f::Zero();
	Eigen::Vector3f RD = Eigen::Vector3f::Zero();
	bool updateTestThread = true;
	int focusOnCamera = 0;
	// Visualization callbacks
	void (*OnUpdateCam)(int n) = NULL;
};

bool ConfiguratorInit(ConfiguratorState &state);
void ConfiguratorExit(ConfiguratorState &state);
bool CommSetup(ConfiguratorState &state);
void CommConnect(ConfiguratorState &state);
void CommDisconnect(ConfiguratorState &state);
void StartStreaming(ConfiguratorState &state);
void StopStreaming(ConfiguratorState &state);
void StartTesting(ConfiguratorState &state);
void StopTesting(ConfiguratorState &state);

#endif // CONFIGURATOR_H