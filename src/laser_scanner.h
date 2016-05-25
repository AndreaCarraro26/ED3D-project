#ifndef LASER_SCANNER_H
#define LASER_SCANNER_H

//#include "stdafx.h"

#include <osg/Group>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h> //Per PointCloud<T>
/*
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>   // Khronos C++ Wrapper API
*/
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
/*
// Contains openCL parameters 
struct OpenCLparam {
	cl::Platform platform;
	cl::Device device;
	cl::Context context;
	cl::CommandQueue queue;
	cl::Program  program;
	cl::Buffer d_v1, d_v2, d_v3, d_sourceCamera, d_directions, d_results, d_new_directions;
};*/

// Contains Configuration data
struct Configuration {

	std::string modelName;
	float cameraHeight;
	float alphaLaser;
	float fanLaser;
	float baseline;
	float laserLength;
	int numLaser;

	bool useBounds;
	float minY;
	float maxY;

	int scanSpeed;
	int fpsCam;

	cv::Vec3f cameraPos;

	int sensor_width;
	int sensor_height;

	double f_x;
	double f_y;
	double x_0;
	double y_0;

	int roi_height;
	int roi_y_1;
	int roi_y_2;

	cv::Mat intrinsicMat;

	bool useMoller;
};

void read_config(Configuration&);
void edit_conf(Configuration&);
void save_config(Configuration&);
void convert_to_3d(cv::Mat, Configuration, float[4], float[4], pcl::PointCloud<pcl::PointXYZ>::Ptr);
cv::Mat reproject(osg::ref_ptr<osg::Node>, Configuration);
int draw_lasers(osg::ref_ptr<osg::Node>, Configuration);
//void init_moller(Configuration&, OpenCLparam&);
//cv::Mat reprojectMoller(OpenCLparam, Configuration, int, int, float);
//int mollerAndata(std::vector<float>, std::vector<float>, std::vector<float>, cv::Vec3f, std::vector<cv::Vec3f>, std::vector<cv::Vec3f>&);
//int mollerRitorno(std::vector<float>, std::vector<float>, std::vector<float>, cv::Vec3f, std::vector<cv::Vec3f>, std::vector<cv::Vec3f>, std::vector<cv::Vec3f>&);
void calculatePlan(Configuration&, float[4], float[4]);

#endif
