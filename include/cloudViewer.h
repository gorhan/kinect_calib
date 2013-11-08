/*
 * cloudViewer.h
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

#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include <iostream>
#include <fstream>
#include <ostream>
#include <boost/thread/thread.hpp>
#include <pcl-1.6/pcl/point_cloud.h>
#include <pcl-1.6/pcl/pcl_config.h>
#include <pcl-1.6/pcl/common/common.h>
#include <pcl-1.6/pcl/common/common_headers.h>
#include <eigen3/Eigen/src/Core/DenseCoeffsBase.h>
#include <pcl-1.6/pcl/io/pcd_io.h>
#include <pcl-1.6/pcl/point_types.h>
#include <pcl-1.6/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.6/pcl/common/transforms.h>

#define TRANSLATION_OFFSET float(0.01)
#define ROTATION_OFFSET float(M_PI / 180.0)

#define TRANSLATION_OFFSET_FINE (TRANSLATION_OFFSET / 2.0)
#define ROTATION_OFFSET_FINE (ROTATION_OFFSET / 2.0)

enum transformation_mode {
	translationMode,
	rotationMode
};

enum kinect_name {
	leftKinect,
	rightKinect
};

struct Kinect {
	transformation_mode mode;
	kinect_name name;
	std::string axis;
};

class CloudViewer {
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leftCloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initialLeftCloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rightCloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initialRightCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_left;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_right;

	Kinect kinect_props;
	bool* states;

	Eigen::Vector3d leftCameraPosition;
	Eigen::Vector3d rightCameraPosition;

	Eigen::Matrix4f leftRotationMatrixX;
	Eigen::Matrix4f leftRotationMatrixY;
	Eigen::Matrix4f leftRotationMatrixZ;
	Eigen::Matrix4f leftTranslationMatrix;

	Eigen::Matrix4f rightRotationMatrixX;
	Eigen::Matrix4f rightRotationMatrixY;
	Eigen::Matrix4f rightRotationMatrixZ;
	Eigen::Matrix4f rightTranslationMatrix;

	Eigen::Matrix4f leftTransformationMatrix;
	Eigen::Matrix4f rightTransformationMatrix;

	float* current_left_translations;
	float* current_right_translations;

	float* current_left_rotations;
	float* current_right_rotations;

	bool initCamera();
	void saveTransformationMatrix(bool left=true);
	void print_info(char* info_msg, bool end=false);
	void print_info(float info_msg, bool end=false);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* cv_obj);

	void setRotationMatrix(kinect_name name, std::string axis, float theta);
	void setTranslationMatrix(kinect_name name, std::string axis, float dist);
	void calculateTransformationMatrix(kinect_name name);
public:
	CloudViewer();
	CloudViewer(std::string leftPcdPath, std::string rightPcdPath);

	void transformCloud(kinect_name name);
	void runViewer();
};


#endif /* CLOUDVIEWER_H_ */
