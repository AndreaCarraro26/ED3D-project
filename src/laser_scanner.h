//#include "stdafx.h"

#include <osg/Group>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

osg::ref_ptr<osg::Geode> scan_scene(osg::ref_ptr<osg::Node>, float, osg::Vec4d*, osg::Vec4d*);
void convert_to_3d (cv::Mat, double, std::vector<double> , std::vector<double> , pcl::PointCloud<pcl::PointXYZ>::Ptr );
cv::Mat get_pic(osg::ref_ptr<osg::Geode>, osg::Matrix &);
cv::Mat reproject(osg::ref_ptr<osg::Node>, float);
	
