#ifndef laser_scanner_H
#define laser_scanner_H
 
#include <osg/Group>
#include <pcl/point_types.h>

#include "screen_capture.cpp"

int scan_scene(osg::ref_ptr<osg::Group>, int, float, osg::Vec4d*, osg::Vec4d*);
std::vector<pcl::PointXYZ> convert_to_3d (cv::Mat, std::vector<double>);

#endif
