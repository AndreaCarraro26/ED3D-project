//#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;												//OCCHIO SE COPINCOLLI

Vec3d convert_to_3d (Mat image, Mat matrix, std::vector<double> plane) {	
	
	Mat point(3,1,CV_64FC1);
	
	for (int i=0; i < image.rows; ++i)
		for (int j=0; j < image.cols; ++j)
			if (image.at<Vec3f>(i,j)[2]>250 && image.at<Vec3f>(i,j)[0]<5) {
				point.at<double>(0) = j;
				point.at<double>(1) = i;
				point.at<double>(2) = 1;
			}
	
	Mat W_inv = matrix.inv();
	
	Mat P = W_inv*point;
	Vec3d point_3d;
	std::cout<<"qui\n\n\n";
	double z = -plane[3]/(plane[0]*P.at<double>(0,0) + plane[1]*P.at<double>(1,0) + plane[2]);
	point_3d[0] = P.at<double>(0,0)*z;
	point_3d[1] = P.at<double>(1,0)*z;
	point_3d[2] = P.at<double>(2,0)*z;
	
	return point_3d;
}

/* 	TEST MAIN - FUNZIONA TUTTO A OCCHIO (perlomeno calcola e stampa, poi se sono giusti e un'altra cosa)

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
