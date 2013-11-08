/*
 * cloudViewer.cpp
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

#include "../include/cloudViewer.h"

void delete_line(bool new_line) {
	if(new_line)
		std::cout << std::endl;

	std::cout << "\033[2K\r" << "\033[0m";
}

void CloudViewer::print_info(char* info_msg, bool end) {
	std::string color_spec = "\033[32;1m";
	std::string reset_spec = "\033[0m";

	std::cout << color_spec << info_msg << reset_spec;
	fflush(stdout);
	delete_line(end);
}

void CloudViewer::print_info(float info_msg, bool end) {
	std::string color_spec = "\033[32;1m";
	std::string reset_spec = "\033[0m";

	std::cout << color_spec << info_msg << reset_spec;
	fflush(stdout);
	delete_line(end);
}

void CloudViewer::saveTransformationMatrix(bool left) {
	std::ofstream file;

	if(left) {
		file.open("leftTransformationMatrix.txt", std::ofstream::out);
		file << leftTransformationMatrix(0, 0);
		std::cout << leftTransformationMatrix(0, 0);
		for(uint i = 0; i < leftTransformationMatrix.rows() - 1; i++) {
			for(uint j = 0; j < leftTransformationMatrix.cols() - 1; j++) {
				if( i == 0 && j == 0)
					continue;
				file << " " << leftTransformationMatrix(i, j);
				std::cout << " " << leftTransformationMatrix(i, j);
			}
		}
		file << " " << leftTransformationMatrix(0, 3);
		std::cout << " " << leftTransformationMatrix(0, 3);
		file << " " << leftTransformationMatrix(1, 3);
		std::cout << " " << leftTransformationMatrix(1, 3);
		file << " " << leftTransformationMatrix(2, 3);
		std::cout << " " << leftTransformationMatrix(2, 3);

		file << std::endl;
		std::cout << std::endl;
	}
	else {
		file.open("rightTransformationMatrix.txt", std::ofstream::out);
		file << rightTransformationMatrix(0, 0);
		for(uint i = 0; i < rightTransformationMatrix.rows() - 1; i++) {
			for(uint j = 0; j < rightTransformationMatrix.cols() - 1; j++) {
				if( i == 0 && j == 0)
					continue;
				file << " " << rightTransformationMatrix(i, j);
			}
		}
		file << " " << rightTransformationMatrix(0, 3);
		file << " " << rightTransformationMatrix(1, 3);
		file << " " << rightTransformationMatrix(2, 3);

		file << std::endl;
	}

	file.close();
}

void CloudViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* cv_obj) {
	std::string key = event.getKeySym();

	if(event.keyDown()) {
		if(key == "l") {
			if(states[0] && states[1])
				kinect_props.name = leftKinect;
		}
		else if(key == "L") {
			if(states[0] && states[1])
				kinect_props.name = rightKinect;
		}
		else if(key == "s") {
			if(!states[0]) {
				states[0] = true;
				kinect_props.name = rightKinect;
				kinect_props.mode = rotationMode;

				viewer->removePointCloud("cloud_left");
				viewer->addPointCloud<pcl::PointXYZRGBA> (rightCloud, rgb_right, "cloud_right");
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_right");
			}
			else if(!states[1]) {
				states[1] = true;
				kinect_props.name = leftKinect;

				viewer->addPointCloud<pcl::PointXYZRGBA> (leftCloud, rgb_left, "cloud_left");
			}
			else {
				saveTransformationMatrix(true);
				saveTransformationMatrix(false);
			}
		}
		else if(key == "x") {
			kinect_props.axis = "x";
		}
		else if(key == "y") {
			kinect_props.axis = "y";
		}
		else if(key == "z") {
			kinect_props.axis = "z";
		}
		else if(key == "t") {
			kinect_props.mode = translationMode;
		}
		else if(key == "k") {
			kinect_props.mode = rotationMode;
		}
		else if(key == "i" || key == "I") {
			if(kinect_props.name == leftKinect) {
				if(kinect_props.mode == translationMode) {
					if(kinect_props.axis == "x") {
						this->current_left_translations[0] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_left_translations[1] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else {
						this->current_left_translations[2] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
				}
				else {
					if(kinect_props.axis == "x") {
						this->current_left_rotations[0] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_left_rotations[1] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else {
						this->current_left_rotations[2] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
				}
			}
			else {
				if(kinect_props.mode == translationMode) {
					if(kinect_props.axis == "x") {
						this->current_right_translations[0] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_right_translations[1] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else {
						this->current_right_translations[2] += (key == "i") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
				}
				else {
					if(kinect_props.axis == "x") {
						this->current_right_rotations[0] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_right_rotations[1] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else {
						this->current_right_rotations[2] += (key == "i") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
				}
			}
			transformCloud(kinect_props.name);
		}
		else if(key == "d" || key == "D") {
			if(kinect_props.name == leftKinect) {
				if(kinect_props.mode == translationMode) {
					if(kinect_props.axis == "x") {
						this->current_left_translations[0] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_left_translations[1] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else {
						this->current_left_translations[2] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
				}
				else {
					if(kinect_props.axis == "x") {
						this->current_left_rotations[0] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_left_rotations[1] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else {
						this->current_left_rotations[2] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
				}
			}
			else {
				if(kinect_props.mode == translationMode) {
					if(kinect_props.axis == "x") {
						this->current_right_translations[0] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_right_translations[1] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
					else {
						this->current_right_translations[2] -= (key == "d") ? (TRANSLATION_OFFSET) : (TRANSLATION_OFFSET_FINE);
					}
				}
				else {
					if(kinect_props.axis == "x") {
						this->current_right_rotations[0] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else if(kinect_props.axis == "y") {
						this->current_right_rotations[1] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
					else {
						this->current_right_rotations[2] -= (key == "d") ? (ROTATION_OFFSET) : (ROTATION_OFFSET_FINE);
					}
				}
			}
			transformCloud(kinect_props.name);
		}
	}
}

CloudViewer::CloudViewer(std::string leftPcdPath, std::string rightPcdPath):
	leftCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), rightCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
	initialLeftCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), initialRightCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
	viewer(new pcl::visualization::PCLVisualizer("3D VIEWER")),
	rgb_left(leftCloud), rgb_right(rightCloud) {
	if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leftPcdPath.c_str(), *this->leftCloud) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leftPcdPath.c_str(), *this->initialLeftCloud) == -1 ||
	   pcl::io::loadPCDFile<pcl::PointXYZRGBA>(rightPcdPath.c_str(), *this->rightCloud) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGBA>(rightPcdPath.c_str(), *this->initialRightCloud) == -1) {
		fprintf(stderr, "ERROR: PCD cannot be loaded..");
	}

	kinect_props.axis = "x";
	kinect_props.name = leftKinect;
	kinect_props.mode = rotationMode;

	this->states = new bool[2];
	this->states[0] = this->states[1] = false;

	this->current_left_rotations = new float[3];
	this->current_right_rotations = new float[3];

	this->current_left_translations = new float[3];
	this->current_right_translations = new float[3];

	for(uint i = 0; i < 3; i++) {
		this->current_left_rotations[i] = 0.0;
		this->current_right_rotations[i] = 0.0;

		this->current_left_translations[i] = 0.0;
		this->current_right_translations[i] = 0.0;
	}

	initCamera();
}

bool CloudViewer::initCamera() {

	for(uint i = 0; i < 4; i++) {
		for(uint j = 0; j < 4; j++) {
			if(i == j && i == 3) {
				this->leftRotationMatrixX(i, j) = 1.0;
				this->leftRotationMatrixY(i, j) = 1.0;
				this->leftRotationMatrixZ(i, j) = 1.0;

				this->rightRotationMatrixX(i, j) = 1.0;
				this->rightRotationMatrixY(i, j) = 1.0;
				this->rightRotationMatrixZ(i, j) = 1.0;
			}
			else {
				this->leftTranslationMatrix(i, j) = 0.0;
				this->rightTranslationMatrix(i, j) = 0.0;

				this->leftRotationMatrixX(i, j) = 0.0;
				this->leftRotationMatrixY(i, j) = 0.0;
				this->leftRotationMatrixZ(i, j) = 0.0;

				this->rightRotationMatrixX(i, j) = 0.0;
				this->rightRotationMatrixY(i, j) = 0.0;
				this->rightRotationMatrixZ(i, j) = 0.0;
			}
		}
	}

	calculateTransformationMatrix(leftKinect);
	calculateTransformationMatrix(rightKinect);

	viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGBA> (leftCloud, rgb_left, "cloud_left");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_left");
	viewer->addCoordinateSystem (0.3);
	viewer->initCameraParameters ();

	viewer->registerKeyboardCallback (&CloudViewer::keyboardEventOccurred, *this);

	return true;
}

void CloudViewer::setTranslationMatrix(kinect_name name, std::string axis, float dist) {
	if(name == leftKinect) {
		if(axis == "x")
			leftTranslationMatrix(0,3) = dist;
		else if(axis == "y")
			leftTranslationMatrix(1,3) = dist;
		else
			leftTranslationMatrix(2,3) = dist;
	}
	else {
		if(axis == "x")
			rightTranslationMatrix(0,3) = dist;
		else if(axis == "y")
			rightTranslationMatrix(1,3) = dist;
		else
			rightTranslationMatrix(2,3) = dist;
	}
}

void CloudViewer::setRotationMatrix(kinect_name name, std::string axis, float theta) {
	if(name == leftKinect) {
		if(axis == "x") {
			leftRotationMatrixX(1, 1) = cos(theta);
			leftRotationMatrixX(1, 2) = -sin(theta);
			leftRotationMatrixX(2, 1) = sin(theta);
			leftRotationMatrixX(2, 2) = cos(theta);
			leftRotationMatrixX(0, 0) = 1.0;
		}
		else if(axis == "y") {
			leftRotationMatrixY(0, 0) = cos(theta);
			leftRotationMatrixY(0, 2) = sin(theta);
			leftRotationMatrixY(2, 0) = -sin(theta);
			leftRotationMatrixY(2, 2) = cos(theta);
			leftRotationMatrixY(1, 1) = 1.0;
		}
		else {
			leftRotationMatrixZ(0, 0) = cos(theta);
			leftRotationMatrixZ(0, 1) = -sin(theta);
			leftRotationMatrixZ(1, 0) = sin(theta);
			leftRotationMatrixZ(1, 1) = cos(theta);
			leftRotationMatrixZ(2, 2) = 1.0;
		}
	}
	else {
		if(axis == "x") {
			rightRotationMatrixX(1, 1) = cos(theta);
			rightRotationMatrixX(1, 2) = -sin(theta);
			rightRotationMatrixX(2, 1) = sin(theta);
			rightRotationMatrixX(2, 2) = cos(theta);
			rightRotationMatrixX(0, 0) = 1.0;
		}
		else if(axis == "y") {
			rightRotationMatrixY(0, 0) = cos(theta);
			rightRotationMatrixY(0, 2) = sin(theta);
			rightRotationMatrixY(2, 0) = -sin(theta);
			rightRotationMatrixY(2, 2) = cos(theta);
			rightRotationMatrixY(1, 1) = 1.0;
		}
		else {
			rightRotationMatrixZ(0, 0) = cos(theta);
			rightRotationMatrixZ(0, 1) = -sin(theta);
			rightRotationMatrixZ(1, 0) = sin(theta);
			rightRotationMatrixZ(1, 1) = cos(theta);
			rightRotationMatrixZ(2, 2) = 1.0;
		}
	}
}

void CloudViewer::calculateTransformationMatrix(kinect_name name) {
	if(name == leftKinect) {
		setTranslationMatrix(leftKinect, "x", current_left_translations[0]);
		setTranslationMatrix(leftKinect, "y", current_left_translations[1]);
		setTranslationMatrix(leftKinect, "z", current_left_translations[2]);
		setRotationMatrix(leftKinect, "x", current_left_rotations[0]);
		setRotationMatrix(leftKinect, "y", current_left_rotations[1]);
		setRotationMatrix(leftKinect, "z", current_left_rotations[2]);

		leftTransformationMatrix = leftRotationMatrixZ * leftRotationMatrixY * leftRotationMatrixX + leftTranslationMatrix;
	}
	else {
		setTranslationMatrix(rightKinect, "x", current_right_translations[0]);
		setTranslationMatrix(rightKinect, "y", current_right_translations[1]);
		setTranslationMatrix(rightKinect, "z", current_right_translations[2]);
		setRotationMatrix(rightKinect, "x", current_right_rotations[0]);
		setRotationMatrix(rightKinect, "y", current_right_rotations[1]);
		setRotationMatrix(rightKinect, "z", current_right_rotations[2]);

		rightTransformationMatrix = rightRotationMatrixZ * rightRotationMatrixY * rightRotationMatrixX + rightTranslationMatrix;
	}
}

void CloudViewer::transformCloud(kinect_name name) {
	calculateTransformationMatrix(name);

	if(name == leftKinect) {
		pcl::transformPointCloud(*this->initialLeftCloud, *this->leftCloud, leftTransformationMatrix);
		this->viewer->updatePointCloud(this->leftCloud, "cloud_left");
	}
	else {
		pcl::transformPointCloud(*this->initialRightCloud, *this->rightCloud, rightTransformationMatrix);
		this->viewer->updatePointCloud(this->rightCloud, "cloud_right");
	}
}

void CloudViewer::runViewer() {
	char* info = new char[256];
	while(!this->viewer->wasStopped()) {
		sprintf(info, "Camera: %s -- Mode: %s -- Axis: %s",
				(kinect_props.name == leftKinect) ? "LeftKinect" : "RightKinect",
				(kinect_props.mode == translationMode) ? "Translation" : "Rotation",
				kinect_props.axis.c_str());
		print_info(info);
		this->viewer->spinOnce(50);
	}
}
