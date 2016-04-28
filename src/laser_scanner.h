#ifndef laser_scanner_H
#define laser_scanner_H
 
#include <osg/Group>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>		//per PointCloud

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat get_pic(osg::ref_ptr<osg::Geode>, osg::Matrixd &);
osg::ref_ptr<osg::Geode> scan_scene(osg::ref_ptr<osg::Node>, float, osg::Vec4d&, osg::Vec4d&);
void convert_to_3d (cv::Mat , std::vector<double> , std::vector<double> , pcl::PointCloud<pcl::PointXYZ>::Ptr );

#endif
