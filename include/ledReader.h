/*
 * ledReader.h
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

#ifndef LEDREADER_H_
#define LEDREADER_H_

#include <iostream>
#include <vector>
#include <map>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

class LedReader {
public:
	LedReader();
	~LedReader();
	LedReader(uint numKinect, std::string remotePortName, std::string localPortName);

	void setRemotePortName(std::string str);
	void setLocalPortName(std::string str);
	void setLedNames(std::string orig, std::string x, std::string xy);
	bool readLedPositions();
	std::map<std::string, Vector> getDataPos();
	std::string getOriginPortName();
	std::string getXAxisPortName();
	std::string getXYPlanePortName();

private:
	bool networkInit();
	void networkFini();

	std::string originLedName;
	std::string xAxisLedName;
	std::string xyPlaneLedName;

	std::map<std::string, Vector> ledPosData;

	std::string remotePortName;
	std::string localPortName;

	BufferedPort<Bottle> localPort;
	uint numOfKinect;
};


#endif /* LEDREADER_H_ */
