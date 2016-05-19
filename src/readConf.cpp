#include "laser_scanner.h"

void read_config(Configuration& confData) {

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	fs["model"] >> confData.modelName;

	fs["cameraHeight"] >> confData.cameraHeight;

	fs["alphaLaser"] >> confData.alphaLaser;
	fs["laserDistance"] >> confData.laserDistance;
	fs["fanLaser"] >> confData.fanLaser;
	fs["numLaser"] >> confData.numLaser;
	confData.laserLength = confData.cameraHeight/cos(3.1416/180*confData.alphaLaser);
	
	fs["ignoreHeight"] >> confData.ignoreHeight;
	fs["useBounds"] >> confData.useBounds; // Se true usa il range letto, altrimenti calcola quello ottimale
	fs["minY"] >> confData.minY;
	fs["maxY"] >> confData.maxY;

	fs["scanSpeed"] >> confData.scanSpeed;
	fs["fpsCam"] >> confData.fpsCam;


	fs["sensor_f_x"] >> confData.f_x;
	fs["sensor_f_y"] >> confData.f_y;
	fs["sensor_x_0"] >> confData.x_0;
	fs["sensor_y_0"] >> confData.y_0;

	fs["sensor_width"] >> confData.sensor_width;
	fs["sensor_height"] >> confData.sensor_height;

	fs["roi_height"] >> confData.roi_height;
	fs["roi_y_1"] >> confData.roi_y_1; 
	fs["roi_y_2"] >> confData.roi_y_2; 
	
	fs.release();

	cv::Mat intrinsic(3, 3, CV_64FC1);
	
	// Popolamento della matrice degli intrinsici
	intrinsic.at<double>(0, 0) = confData.f_x;
	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(0, 2) = confData.x_0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(1, 1) = confData.f_y;
	intrinsic.at<double>(1, 2) = confData.y_0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	confData.intrinsicMat = intrinsic;

}