#ifndef laser_scanner_H
#define laser_scanner_H
 
#include <osg/Group>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

osg::ref_ptr<osg::Geode> scan_scene(osg::ref_ptr<osg::Node>, float, osg::Vec4d*, osg::Vec4d*);
void convert_to_3d (cv::Mat , std::vector<double> , std::vector<double> , pcl::PointCloud<pcl::PointXYZ>::Ptr );
cv::Mat get_pic(osg::ref_ptr<osg::Geode>, osg::Matrix &);

#endif
