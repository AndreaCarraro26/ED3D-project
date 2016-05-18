#include "laser_scanner.h"

void read_config(Configuation confData) {

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	struct Configuration confData; //struct contentente tutti i dati di configurazione


	std::string modelName;
	fs["model"] >> modelName;

	fs["cameraHeight"] >> confData.cameraHeight;

	fs["alphaLaser"] >> confData.alphaLaser;
	fs["laserDistance"] >> confData.laserDistance;
	confData.laserLength = confData.cameraHeight/cos(deg2rad*confData.alphaLaser);
	
	fs["ignoreHeight"] >> confData.ignoreHeight;
	fs["useBounds"] >> confData.useBounds; // Se true usa il range letto, altrimenti calcola quello ottimale
	fs["minY"] >> confData.minY;
	fs["maxY"] >> confData.maxY;

	fs["scanSpeed"] >> confData.scanSpeed;
	fs["fpsCam"] >> confData.fpsCam;
	fs["fanLaser"] >> confData.fanLaser;
	fs["numLaser"] >> confData.numLaser;

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


}