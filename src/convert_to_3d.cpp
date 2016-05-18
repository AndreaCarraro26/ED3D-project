//#include "stdafx.h"

#include "laser_scanner.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <pcl/point_types.h>									
#include <pcl/point_cloud.h>

void convert_to_3d(cv::Mat image, Configuration params, std::vector<double> planeA_coeff, std::vector<double> planeB_coeff, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {


	cv::Mat intrinsic(3, 3, CV_64FC1);
	
	// Popolamento della matrice degli intrinsici
	intrinsic.at<double>(0, 0) = params.f_x;
	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(0, 2) = params.x_0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(1, 1) = params.f_y;
	intrinsic.at<double>(1, 2) = params.y_0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	cv::Mat M_inv = intrinsic.inv();
	
	cv::Mat P;
	double z;
	cv::Mat point(3, 1, CV_64FC1);	//allocazione di un punto tridimensionale
	
	double A1, B1, C1, D1, A2, B2, C2, D2;
	A1 = planeB_coeff[0];	A2 = planeA_coeff[0];
	B1 = planeB_coeff[1];	B2 = planeA_coeff[1];
	C1 = planeB_coeff[2];	C2 = planeA_coeff[2];
	D1 = planeB_coeff[3];	D2 = planeA_coeff[3];

	int row;
	for (int i = 0; i < params.roi_height; ++i) {
		for (int j = 0; j < params.sensor_width; ++j) {

			// elaborazione per la prima ROI
			row = i + params.roi_y_1;	
			if (image.at<uchar>(row, j) == 128) {
				point.at<double>(0,0) = j;
				point.at<double>(1,0) = row;
				point.at<double>(2,0) = 1;
				
				P = M_inv*point;

				z = -D1 / (A1 * P.at<double>(0) + B1 * P.at<double>(1) + C1);

				pcl::PointXYZ point_3d(	P.at<double>(0)*z + params.cameraPos.x(), 
										P.at<double>(1)*z + params.cameraPos.y(), 
										P.at<double>(2)*z + params.cameraPos.z());
				cloud->push_back(point_3d);
				

			}
			// elaborazione per la seconda ROI
			row = i + params.roi_y_2;
			
			if (image.at<uchar>(row, j) == 255) {
				point.at<double>(0) = j;
				point.at<double>(1) = row;
				point.at<double>(2) = 1;

				P = M_inv*point;

				z = -(D2 / (A2*P.at<double>(0) + B2*P.at<double>(1) + C2));
				pcl::PointXYZ point_3d(	P.at<double>(0)*z + params.cameraPos.x(), 
										P.at<double>(1)*z + params.cameraPos.y(), 
										P.at<double>(2)*z + params.cameraPos.z());

				cloud->push_back(point_3d);
			}

		}

	}

	return;
}
