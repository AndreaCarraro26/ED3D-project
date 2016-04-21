//#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pcl/point_types.h>



using namespace cv;												//OCCHIO SE COPINCOLLI

std::vector<pcl::PointXYZ> convert_to_3d (Mat image, std::vector<double> plane_coeff) {	
	
	Mat point(3,1,CV_64FC1);
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	double f_x;			fs["sensor_f_x"] >> f_x;
	double f_y;			fs["sensor_f_y"] >> f_y;
	double x_0;			fs["sensor_x_0"] >> x_0;
	double y_0;			fs["sensor_y_0"] >> y_0;
	fs.release();
	cv::Mat intrinsic(3,3,CV_64FC1);
	
	intrinsic.at<double>(0,0) = f_x;
	intrinsic.at<double>(0,1) = 0;
	intrinsic.at<double>(0,2) = x_0;
	intrinsic.at<double>(1,0) = 0;
	intrinsic.at<double>(1,1) = f_y;
	intrinsic.at<double>(1,2) = y_0;
	intrinsic.at<double>(2,0) = 0;
	intrinsic.at<double>(2,1) = 0;
	intrinsic.at<double>(2,2) = 1;
	
	Mat W_inv = intrinsic.inv();
	Mat P;
	double z;

	std::vector<pcl::PointXYZ> point_line;
	for (int i=0; i < image.rows; ++i)
		for (int j=0; j < image.cols; ++j)
			if (image.at<Vec3f>(i,j)[2]>250 && image.at<Vec3f>(i,j)[0]<5) {
				point.at<double>(0) = j;
				point.at<double>(1) = i;
				point.at<double>(2) = 1;				
				
				P = W_inv*point;

				z = -plane_coeff[3]/(plane_coeff[0]*P.at<double>(0,0) + plane_coeff[1]*P.at<double>(1,0) + plane_coeff[2]);
				pcl::PointXYZ point_3d( P.at<double>(0,0)*z, P.at<double>(1,0)*z, P.at<double>(2,0)*z);

				point_line.push_back(point_3d);
				std::cout<<point_3d<<std::endl;
	}
	
	return point_line;
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
