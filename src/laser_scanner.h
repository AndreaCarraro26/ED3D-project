#ifndef LASER_SCANNER_H
#define LASER_SCANNER_H

//#include "stdafx.h"
#include <iostream>
#include <string>
#include <vector>

#include <osg/Group>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct Configuration {

	std::string modelName;
	float cameraHeight;
	float alphaLaser;
	float fanLaser;
	float laserDistance;
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
	
};

//osg::ref_ptr<osg::Geode> scan_scene(osg::ref_ptr<osg::Node>, float, osg::Vec4d*, osg::Vec4d*);
//cv::Mat get_pic(osg::ref_ptr<osg::Geode>, osg::Matrix &);

void read_config(Configuration&);
void edit_conf(Configuration&);
cv::Mat reproject(osg::ref_ptr<osg::Node>, Configuration);
int draw_lasers(osg::ref_ptr<osg::Node>, Configuration);
void convert_to_3d (cv::Mat, Configuration, osg::Vec4d , osg::Vec4d , pcl::PointCloud<pcl::PointXYZ>::Ptr );

#endif