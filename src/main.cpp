/*
 * main.cpp
 * Copyright (c) 2013, Güner ORHAN - Kovan Research Laboratory, METU
 * gunerorhan2010@gmail.com
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "../include/ledReader.h"
#include "../include/cloudViewer.h"

uint init(char* path, std::vector<LedReader*>& kinectList, std::vector<std::string>& pcdFilePaths) {
	Property* rf = new Property();
	rf->fromConfigFile(path);

	int numKinect = rf->findGroup("KINECT").find("number").asInt();

	char* kinectGroupName;
	char* kinectPortName;

	for(uint i = 0; i < numKinect; i++) {
		kinectGroupName = new char(16);
		kinectPortName = new char(64);

		rf = new Property();
		rf->fromConfigFile(path);

		Bottle& botPorts = rf->findGroup("PORTNAMES");
		sprintf(kinectGroupName, "KINECT_%d", i + 1);
		sprintf(kinectPortName, "%s/kinect_%d", botPorts.find("local").asString().c_str(), i + 1);

//		LedReader* reader = new LedReader(numKinect, botPorts.find("remote").asString().c_str(),kinectPortName);

		Bottle& bot = rf->findGroup(kinectGroupName);
//		reader->setLedNames(bot.find("originLedName").asString().c_str(),
//						   bot.find("xAxisLedName").asString().c_str(),
//						   bot.find("xyPlaneLedName").asString().c_str());

		pcdFilePaths.push_back(bot.find("pcdFilePath").asString().c_str());
//		kinectList.push_back(reader);
	}

	return numKinect;
}

int main(int argc, char** argv) {
	if(argc != 2) {
		fprintf(stderr, "Invalid Initialization\n");
		fprintf(stderr, "Usage: %s <conf_file_path>\n", argv[0]);
		return -1;
	}

//	Network yarp;
//	yarp.init();
//
//	if (!yarp.checkNetwork()) {
//		fprintf(stderr,"Error: yarp server does not seem available\n");
//		return -1;
//	}
	std::vector<LedReader*> kinectList;
	std::vector<std::string> pcdFilePaths;

	uint numKinect = init(argv[1], kinectList, pcdFilePaths);

//	std::cout << pcdFilePaths.data()[0] << " " << pcdFilePaths.data()[1] << std::endl;

	/*while(!kinectList.at(0)->readLedPositions()) {
		Time::delay(0.25);
		std::cout << "Waiting to read.." << std::endl;
	}

	while(!kinectList.at(0)->readLedPositions() && !kinectList.at(1)->readLedPositions());

	for(uint i = 0; i < numKinect; i++) {
		std::cout << kinectList.at(i)->getDataPos()[kinectList.at(i)->getOriginPortName()].toString() << std::endl;
		std::cout << kinectList.at(i)->getDataPos()[kinectList.at(i)->getXAxisPortName()].toString() << std::endl;
		std::cout << kinectList.at(i)->getDataPos()[kinectList.at(i)->getXYPlanePortName()].toString() << std::endl;
	}*/

	CloudViewer cv(pcdFilePaths.at(0), pcdFilePaths.at(1));
	cv.runViewer();

//	for(uint i = 0; i < numKinect; i++) {
//		sprintf(kinectName, "Kinect_%d", i + 1);
//		std::cout << kinectName << std::endl;
//		std::cout << "Origin Led Name: " << rf->findGroup(kinectName).find("originLedName").asString() << std::endl;
//		std::cout << "X-Axis Led Name: " << rf->findGroup(kinectName).find("xAxisLedName").asString() << std::endl;
//		std::cout << "XY-Plane Led Name: " << rf->findGroup(kinectName).find("xyPlaneLedName").asString() << std::endl;
//	}

//	std::cout << "Module Name: " << rf->check("module", Value("kinect_tf")).asString() << std::endl;
//	std::cout << "Number of Kinects: " << rf->check("numOfKinect", Value("2")).asInt() << std::endl;
	return 0;
}
