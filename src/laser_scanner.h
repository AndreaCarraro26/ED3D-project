#ifndef laser_scanner_H
#define laser_scanner_H
 
#include <osg/Group>
#include <osg/Node>

#include "screen_capture.cpp"

int scan_scene(osg::ref_ptr<osg::Group>, osg::ref_ptr<osg::Node>, float, osg::Vec4d*, osg::Vec4d*);
std::vector<cv::Vec3d> convert_to_3d (cv::Mat, std::vector<double>);

#endif
