//#include <math.h>
#include "stdafx.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace cv;												

void convert_to_3d(Mat image, std::vector<double> planeA_coeff, std::vector<double> planeB_coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	std::cout << "dentro al 3d" << std::endl;
	Mat point(3, 1, CV_64FC1);	//allocazione di un punto tridimensionale
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
	int roi_heigh;		fs["roi_heigh"] >> roi_heigh;

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

	Mat W_inv = intrinsic.inv();
	Mat P;
	double z;

	//// ricorda: roi(Xapplicazione, Yapplicazione, larghezza, altezza) 
	//Rect roi1(0, roi_y_1, width, roi_heigh);
	//Mat image_roi1 = image(roi1);
	//
	//Rect roi2(0, roi_y_2, width, roi_heigh);
	//Mat image_roi2 = image(roi2);

	// imshow("Example1", image);	waitKey(0);
	int temp;

	std::vector<pcl::PointXYZ> point_line;
	for (int i = 0; i < roi_heigh; ++i) {
		for (int j = 0; j < width; ++j) {
			
			// && image.at<Vec3b>(i + roi_y_1, j)[0]<100
			// elaborazione per la ROI1
			int col = i + roi_y_1;
			temp = image.at<Vec3b>(col, j)[2];
 			if (image.at<Vec3b>(col, j)[2]>200 && image.at<Vec3b>(i + roi_y_1, j)[0]<50) {
				point.at<double>(0) = j ;
				point.at<double>(1) = i+ roi_y_1;
				point.at<double>(2) = 1;

				P = W_inv*point;
				
				z = -planeA_coeff[3] / (planeA_coeff[0] * P.at<double>(0, 0) + planeA_coeff[1] * P.at<double>(1, 0) + planeA_coeff[2]);
				pcl::PointXYZ point_3d(P.at<double>(0, 0)*z, P.at<double>(1, 0)*z, P.at<double>(2, 0)*z);

				point_line.push_back(point_3d);

				std::cout << "inserted first: " << i + roi_y_1 << " " << j << std::endl;
				
			}
			// && image.at<Vec3b>(i + roi_y_2, j	)[0]<5
			// elaborazione per la ROI2
			col = i + roi_y_2;
			temp = image.at<Vec3b>(col, j)[2];
			if (image.at<Vec3b>(col, j )[2]>200 && image.at<Vec3b>(i + roi_y_2, j)[0]<50) {
				point.at<double>(0) = j ;
				point.at<double>(1) = i+ roi_y_2;
				point.at<double>(2) = 1;

				P = W_inv*point;

				z = -planeB_coeff[3] / (planeB_coeff[0] * P.at<double>(0, 0) + planeB_coeff[1] * P.at<double>(1, 0) + planeB_coeff[2]);
				pcl::PointXYZ point_3d(P.at<double>(0, 0)*z, P.at<double>(1, 0)*z, P.at<double>(2, 0)*z);

				point_line.push_back(point_3d);

				std::cout << "inserted second: " << i + roi_y_2 << " " << j << std::endl;
			}

		}
			
	}
	
	cloud->insert(cloud->begin(), point_line.begin(), point_line.end());

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
