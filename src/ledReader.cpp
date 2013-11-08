/*
 * ledReader.cpp
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

#include "../include/ledReader.h"

bool LedReader::networkInit() {
	return (this->localPort.open(this->localPortName.c_str()) &&
			Network::connect(this->remotePortName.c_str(), this->localPortName.c_str()));
}

void LedReader::networkFini() {
	Network::disconnect(this->remotePortName.c_str(), this->localPortName.c_str());
	this->localPort.close();
}

LedReader::LedReader(uint numKinect, std::string remotePortName, std::string localPortName) {
	this->remotePortName = remotePortName;
	this->localPortName = localPortName;
	this->numOfKinect = numKinect;

	if(!networkInit()) {
		fprintf(stderr, "ERROR: Unable to create and connect yarp port..\n");
		fprintf(stderr, "<REMOTE>: %s\n<LOCAL>: %s\n", this->remotePortName.c_str(), this->localPortName.c_str());
	}
}

void LedReader::setLedNames(std::string orig, std::string x, std::string xy) {
	this->originLedName = orig;
	this->xAxisLedName = x;
	this->xyPlaneLedName = xy;

	std::cout << "this->ledPosData: start" << std::endl;
	this->ledPosData[this->originLedName] = Vector(3);
	this->ledPosData[this->xAxisLedName] = Vector(3);
	this->ledPosData[this->xyPlaneLedName] = Vector(3);
	std::cout << "this->ledPosData: end" << std::endl;
}

std::map<std::string, Vector> LedReader::getDataPos() {
	return this->ledPosData;
}

bool LedReader::readLedPositions() {
	uint ledCount = 3;
	Bottle* input = this->localPort.read(false);
	if(input) {
		for(uint i = 0; i < ledCount * this->numOfKinect; i++) {
			if(this->originLedName == input->get(i * 5 + 1).asString().c_str()) {
				this->ledPosData[this->originLedName][0] = input->get(i * 5 + 4).asDouble();
				this->ledPosData[this->originLedName][1] = input->get(i * 5 + 3).asDouble();
				this->ledPosData[this->originLedName][2] = input->get(i * 5 + 2).asDouble();
			}
			else if(this->xAxisLedName == input->get(i * 5 + 1).asString().c_str()) {
				this->ledPosData[this->xAxisLedName][0] = input->get(i * 5 + 4).asDouble();
				this->ledPosData[this->xAxisLedName][1] = input->get(i * 5 + 3).asDouble();
				this->ledPosData[this->xAxisLedName][2] = input->get(i * 5 + 2).asDouble();
			}
			else if(this->xyPlaneLedName == input->get(i * 5 + 1).asString().c_str()) {
				this->ledPosData[this->xyPlaneLedName][0] = input->get(i * 5 + 4).asDouble();
				this->ledPosData[this->xyPlaneLedName][1] = input->get(i * 5 + 3).asDouble();
				this->ledPosData[this->xyPlaneLedName][2] = input->get(i * 5 + 2).asDouble();
			}
		}
		return true;
	}

	return false;
}

std::string LedReader::getOriginPortName() {
	return this->originLedName;
}

std::string LedReader::getXAxisPortName() {
	return this->xAxisLedName;
}

std::string LedReader::getXYPlanePortName() {
	return this->xyPlaneLedName;
}

LedReader::~LedReader() {
	networkFini();
}
