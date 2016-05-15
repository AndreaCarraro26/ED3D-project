#include "stdafx.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>										

void convert_to_3d(cv::Mat image, double position, std::vector<double> planeA_coeff, std::vector<double> planeB_coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//std::cout << "dentro al 3d" << std::endl;

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	double f_x;			fs["sensor_f_x"] >> f_x;
	double f_y;			fs["sensor_f_y"] >> f_y;
	double x_0;			fs["sensor_x_0"] >> x_0;
	double y_0;			fs["sensor_y_0"] >> y_0;

	int width;			fs["sensor_width"] >> width;
	int height;			fs["sensor_height"] >> height;

	int roi_y_1;		fs["roi_y_1"] >> roi_y_1;
	int roi_y_2;		fs["roi_y_2"] >> roi_y_2;
	int roi_height;		fs["roi_height"] >> roi_height;

	int cameraX = 0 ;
	double cameraY = position;
	int cameraZ;		fs["cameraHeight"] >> cameraZ;

	fs.release();

	cv::Mat intrinsic(3, 3, CV_64FC1);

	// Popolamento della matrice degli intrinsici
	intrinsic.at<double>(0, 0) = f_x;
	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(0, 2) = x_0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(1, 1) = f_y;
	intrinsic.at<double>(1, 2) = y_0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	cv::Mat M_inv = intrinsic.inv();
	
	cv::Mat P;
	double z;
	cv::Mat point(3, 1, CV_64FC1);	//allocazione di un punto tridimensional
	
	double A1, B1, C1, D1, A2, B2, C2, D2;
	A1 = planeB_coeff[0];	A2 = planeA_coeff[0];
	B1 = planeB_coeff[1];	B2 = planeA_coeff[1];
	C1 = planeB_coeff[2];	C2 = planeA_coeff[2];
	D1 = planeB_coeff[3];	D2 = planeA_coeff[3];
	
	int num_point_inserted = 0 ;
	int row;
	for (int i = 0; i < roi_height; ++i) {
		for (int j = 0; j < width; ++j) {

			// && image.at<Vec3b>(i + roi_y_1, j)[0]<100
			// elaborazione per la ROI1
			row = i + roi_y_1;
						
			if (image.at<uchar>(row, j) == 128) {
				point.at<double>(0,0) = j;
				point.at<double>(1,0) = row;
				point.at<double>(2,0) = 1;
				
				P = M_inv*point;

				z = -D1 / (A1 * P.at<double>(0) + B1 * P.at<double>(1) + C1);

				pcl::PointXYZ point_3d(P.at<double>(0)*z + cameraX, P.at<double>(1)*z + cameraY, P.at<double>(2)*z + cameraZ);
				//std::cout << point_3d << std::endl;
				//std::cout << "Prova appartenenza piano " << point_3d.x*A1 + point_3d.y*B1 + point_3d.z*C1 + D << std::endl;
				//cv::waitKey(100);
				cloud->push_back(point_3d);
				num_point_inserted++;

			}

			row = i + roi_y_2;
			
			if (image.at<uchar>(row, j) == 255) {
				point.at<double>(0) = j;
				point.at<double>(1) = row;
				point.at<double>(2) = 1;

				P = M_inv*point;

				z = -(D2 / (A2*P.at<double>(0) + B2*P.at<double>(1) + C2));
				pcl::PointXYZ point_3d(P.at<double>(0)*z + cameraX, P.at<double>(1)*z+ cameraY, P.at<double>(2)*z + cameraZ);

				cloud->push_back(point_3d);
				num_point_inserted++;
			}

		}

	}

	//std::cout << "Inseriti nella PC " << num_point_inserted << " punti"<< std::endl;

	return;
}

//	TEST MAIN - FUNZIONA TUTTO A OCCHIO (perlomeno calcola e stampa, poi se sono giusti e un'altra cosa)
/*
int main() {
double f_x = 4615.04;
double f_y = 4615.51;
double x_0 = 1113.41;
double y_0 = 480.016;
Mat image = imread("../data/laseramano.bmp");
Mat intrinsic(3,3,CV_64FC1);
intrinsic.at<double>(0,0) = f_x;
intrinsic.at<double>(0,2) = x_0;
intrinsic.at<double>(1,1) = f_y;
intrinsic.at<double>(1,2) = y_0;
intrinsic.at<double>(2,2) = 1;
std::vector<double> plane;
plane.push_back(1.0);
plane.push_back(-1.0);
plane.push_back(1.0);
plane.push_back(10.0);
Vec3d point = convert_to_3d(image, intrinsic, plane);
std::cout<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;
return 0;
}*/
