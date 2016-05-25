#include "stdafx.h"

#include "laser_scanner.h"

#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>   // Khronos C++ Wrapper API
#include <cstdio>      // For C style 
#include <iostream>    // For C++ style IO
#include <vector>      // For C++ vector types
#include <math.h> 

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


cv::Mat reprojectMoller(OpenCLparam CLparam, Configuration conf, int numPolygons, int numDirections, float position) {

	float *cameraPosition = (float*)malloc(sizeof(float) * 3);	
	cameraPosition[0] = conf.cameraPos[0];
	cameraPosition[1] = position;
	cameraPosition[2] = conf.cameraPos[2];

	float * newResults = (float *)malloc(numDirections * 3 * 2 * sizeof(float));
	
	CLparam.queue.enqueueWriteBuffer(CLparam.d_sourceCamera, CL_TRUE, 0, sizeof(float) * 3, cameraPosition);

	//run the kernel
	cl::Kernel andata = cl::Kernel(CLparam.program, "mollerAndata");
	try {
		andata.setArg(0, numPolygons);
		andata.setArg(1, CLparam.d_v1);
		andata.setArg(2, CLparam.d_v2);
		andata.setArg(3, CLparam.d_v3);
		andata.setArg(4, CLparam.d_directions);
		andata.setArg(5, CLparam.d_results);
		andata.setArg(6, CLparam.d_new_directions);
		andata.setArg(7, CLparam.d_sourceCamera);
		andata.setArg(8, numDirections);
		andata.setArg(9, conf.baseline);
		CLparam.queue.enqueueNDRangeKernel(andata, cl::NullRange, cl::NDRange(numDirections * 3 * 2),  cl::NullRange);

		cl::Buffer sources = cl::Buffer::Buffer(CLparam.context, CL_MEM_READ_WRITE, sizeof(float) * numDirections * 3 * 2);
	}
	catch (cl::Error error) {
		std::cout << error.err() << std::endl;
		throw error;
	}

	//////////////////////////////////////////////////
	// Eliminazione punti nascosti

	cl::Kernel ritorno = cl::Kernel(CLparam.program, "mollerRitorno");

	ritorno.setArg(0, numPolygons);
	ritorno.setArg(1, CLparam.d_v1);
	ritorno.setArg(2, CLparam.d_v2);
	ritorno.setArg(3, CLparam.d_v3);
	ritorno.setArg(4, CLparam.d_sourceCamera);
	ritorno.setArg(5, CLparam.d_new_directions);
	ritorno.setArg(6, CLparam.d_results);
	CLparam.queue.enqueueNDRangeKernel(ritorno, cl::NullRange, cl::NDRange(numDirections * 6), cl::NullRange);


	CLparam.queue.enqueueReadBuffer(CLparam.d_results, CL_TRUE, 0, sizeof(float) * numDirections * 2 * 3, newResults);


	/////////////////////////////////////////////////////////////////////////////////////
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;	rVec.at<double>(1) = 0;	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
	distCoeffs.at<double>(0) = 0; 	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0; 	distCoeffs.at<double>(3) = 0;
	distCoeffs.at<double>(4) = 0;

	std::vector<cv::Vec2f> imagePoints;
	cv::Mat image_to_return(conf.sensor_height, conf.sensor_width, CV_8UC1);
	image_to_return.setTo(0);

	std::vector<cv::Vec3f> finalResults;
	for (int i = 0; i < numDirections * 2 * 3; i = i + 3) {
		if (newResults[i] < FLT_MAX) {
			finalResults.push_back({ newResults[i], newResults[i + 1], newResults[i + 2] });
		}
	}

	if (finalResults.size() != 0)
		cv::projectPoints(finalResults, rVec, tVec, conf.intrinsicMat, distCoeffs, imagePoints);

	float x, y;
	for (int i = 0; i < imagePoints.size(); i++) {
		x = imagePoints[i][0];
		y = imagePoints[i][1];
		if (0<((int)x) && ((int)x) < conf.sensor_width && 0<((int)y) && ((int)y) < conf.sensor_height) {
			//std::cout << "y " << y << "x "<<x << std::endl;
			image_to_return.at<uchar>((int)y, (int)x) = 255;
		}
	}

	return image_to_return;
}