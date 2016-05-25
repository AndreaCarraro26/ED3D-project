#include "laser_scanner.h"
#include <osg/Plane>

void read_config(Configuration& confData) {

	// this functions reads from config file and save data in a Configuration struct

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	fs["model"] >> confData.modelName;

	fs["cameraHeight"] >> confData.cameraHeight;

	fs["alphaLaser"] >> confData.alphaLaser;
	fs["baseline"] >> confData.baseline;
	fs["fanLaser"] >> confData.fanLaser;
	fs["numLaser"] >> confData.numLaser;
	confData.laserLength = confData.cameraHeight / cos(3.1416 / 180 * confData.alphaLaser);

	fs["useBounds"] >> confData.useBounds; 
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

	fs["moller"] >> confData.useMoller;

	fs.release();

	cv::Mat intrinsic(3, 3, CV_64FC1);

	// Intrisics matrix
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



void edit_conf(Configuration& confData) {

	// this function is used to edit the Configuration struct

	double deg2rad = 2 * 3.1416 / 360;
	float input;

	bool esc = false;
	bool answer_valid = true;
	
	while (!esc) {

		#ifdef _WIN32
			system("cls");
		#elif linux 
			system("clear");
		#endif

		bool input_valid = false;

		std::string type = "Moller-Trumbore";
		if (!confData.useMoller)
			type = "Naive-OSG";

		std::string useB = "False";
		if (confData.useBounds)
			useB = "True";

		if(!answer_valid)
			std::cout << "Please select a number from the list.\n\n";

		std::cout << "CONFIGURATION MENU\n";
		std::cout << "-----------------------\n";
		std::cout << "Select the parameter to modify:\n";
		std::cout << "[1] Camera/Laser height (" << confData.cameraHeight << " mm)\n";
		std::cout << "[2] Baseline (" << confData.baseline << " mm)\n";
		std::cout << "[3] Angle from XY plane (" << confData.alphaLaser << "°)\n";
		std::cout << "[4] Fan angle (" << confData.fanLaser << "°)\n";
		std::cout << "[5] Use bounds for scan (" << useB << ")\n";
		std::cout << "[6] Y position at start (" << confData.minY << " mm)\n";
		std::cout << "[7] Y position at the end (" << confData.maxY << " mm)\n";
		std::cout << "[8] Select intersection function (" << type << ")\n";

		std::cout << "NB: Camera parameters, such as intrinsics matrix and ROI, must be set up in the XML configuration file.\n\n";

		std::cout << "[0] Exit configuration and visualize the scene.\n\n" << std::flush;

		int answer = -1;
		std::cin >> answer;

		if(std::cin.fail()) {
			answer = -1;
		}
		switch (answer) {
			case 1:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.cameraHeight << std::endl;
					std::cout << "New height: " << std::flush; std::cin >> input;
					if (std::cin.fail() || input <= 0) {
						std::cout << "Input not valid! Insert a value greater than 0.\n";
						input_valid = false;
					}
				}
				confData.cameraHeight = input;
				confData.laserLength = confData.cameraHeight / sin(deg2rad*confData.alphaLaser);
				break;
			case 2:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.baseline << std::endl;
					std::cout << "New baseline (value accepted from 500 to 800 mm): " << std::flush; std::cin >> input;
					if (std::cin.fail() || input <= 500 || input > 800) {
						std::cout << "Input not valid! Insert a value ranging from 500 to 800 mm.\n";
						input_valid = false;
					}
				}
				confData.baseline = input;
				break;
			case 3:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.alphaLaser << std::endl;
					std::cout << "New angle (value accepted from 60 to 70 deg): " << std::flush; std::cin >> input;
					if (std::cin.fail() || input <= 60 || input > 70) {
						std::cout << "input not valid! Insert a value ranging from 60 to 70 deg.\n";
						input_valid = false;
					}

				}
				confData.alphaLaser = (int)input;
				confData.laserLength = confData.cameraHeight / sin(deg2rad*confData.alphaLaser);
				break;
			case 4:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.fanLaser << std::endl;
					std::cout << "New laser fan angle (from 30 to 45 deg): " << std::flush; std::cin >> input;
					if (std::cin.fail() || input < 30 || input > 45) {
						std::cout << "Input not valid! Insert a value ranging from 30 to 45 deg.\n";
						input_valid = false;
					}
				}
				confData.fanLaser = input;
				break;
			case 5:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << useB << std::endl;

					std::cout << "Type (T) to use custom bounds, (F) otherwise. \nIf True, bounds must be setted manually. " << std::flush;
					std::string ch;
					std::cin >> ch;
					if ((ch == "t" || ch == "T") && !std::cin.rdbuf()->in_avail()) {
						confData.useBounds = 1;
						useB = "True";
						break;
					}	
					else if ((ch == "f" || ch == "F") && !std::cin.rdbuf()->in_avail()) {
						confData.useBounds = 0;
						useB = "False";
						break;
					}
					else {
						std::cout << "Input not valid! " << std::endl;
						input_valid = false;
					}
				}
			case 6:
				while (!input_valid) {
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.minY << std::endl;
					std::cout << "New camera start position: " << std::flush; std::cin >> input;
					if (std::cin.fail()) {
						std::cout << "Input not valid! Insert a number value!\n";
						input_valid = false;
					}
				}
				confData.minY = input;
				break;
			case 7:
				while (!input_valid) {		
					std::cin.clear();
					std::cin.ignore(100,'\n');
					input_valid = true;
					std::cout << "Current value: " << confData.maxY << std::endl;
					std::cout << "New camera final position: " << std::flush; std::cin >> input;
					if (std::cin.fail()) {
						std::cout << "Input not valid! Insert a number value!\n";
						input_valid = false;
					}
				}
				confData.maxY = input;
				break;
			case 8:
				while (!input_valid) {
					input_valid = true;
					std::cin.ignore(std::cin.rdbuf()->in_avail(),'\n');
					std::cout << "Current value: " << type << std::endl;

					std::cout << "Type (M) to use Moller-Trumbore parallellized algorithm, (N) to use Naive-OSG based algorithm: " << std::flush;
					std::string ch;
					std::cin >> ch;
					if ((ch == "m" || ch == "M") && !std::cin.rdbuf()->in_avail()) {
						confData.useMoller = 1;
						type = "Moller-Trumbore";
						break;
					}else if ((ch == "n" || ch == "N") && !std::cin.rdbuf()->in_avail()) {
						confData.useMoller = 0;
						break;
					}
					else {
						std::cout << "Input not valid! " << std::endl;
						type = "Naive-OSG";
						input_valid = false;
					}
				}

			case 0:
				esc = true;
				break;
			
			default:
				answer_valid = false;
				std::cin.clear();
				std::cin.ignore(100,'\n');
				break;			
		}
	}
}

void save_config(Configuration& confData) {

	// this function saves the struct data in Configuration.xml 

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::WRITE);

	fs << "model" << confData.modelName;

	fs << "cameraHeight"<< confData.cameraHeight;

	fs << "alphaLaser" << confData.alphaLaser;
	fs << "baseline" << confData.baseline;
	fs << "fanLaser" << confData.fanLaser;
	fs << "numLaser" << confData.numLaser;

	fs << "useBounds" << confData.useBounds; 
	fs << "minY" << confData.minY;
	fs << "maxY" << confData.maxY;

	fs << "scanSpeed" <<  confData.scanSpeed;
	fs << "fpsCam" <<  confData.fpsCam;

	fs << "sensor_f_x" <<  confData.f_x;
	fs << "sensor_f_y" <<  confData.f_y;
	fs << "sensor_x_0" <<  confData.x_0;
	fs << "sensor_y_0" <<  confData.y_0;

	fs << "sensor_width" <<  confData.sensor_width;
	fs << "sensor_height" <<  confData.sensor_height;

	fs << "roi_height" <<  confData.roi_height;
	fs << "roi_y_1" <<  confData.roi_y_1;
	fs << "roi_y_2" <<  confData.roi_y_2;


	fs << "moller" <<  confData.useMoller;

	fs.release();

	std::cout << "Configuration saved in \"" << confFile <<"\""<< std::endl;

}

void calculatePlan(Configuration& confData, float _planeA[4], float _planeB[4]) {
	
	// Obtainig laser plane coefficients
	double deg2rad = 2 * 3.1416 / 360;

	// Three points needed to define a plane

	osg::Vec3 a1(0, confData.baseline, 0);

	osg::Vec3 a2(	0,
					confData.baseline - confData.laserLength*cos(deg2rad*(confData.alphaLaser)),
					-confData.laserLength*sin(deg2rad*(confData.alphaLaser)));

	osg::Vec3 a3(	100,
					confData.baseline - confData.laserLength*cos(deg2rad*(confData.alphaLaser)),
					-confData.laserLength*sin(deg2rad*(confData.alphaLaser)));


	osg::Vec3 b1(0, -confData.baseline, 0);

	osg::Vec3 b2(	0,
					-confData.baseline + confData.laserLength*cos(deg2rad*(confData.alphaLaser)),
					-confData.laserLength*sin(deg2rad*(confData.alphaLaser)));

	osg::Vec3 b3(	100,
					-confData.baseline + confData.laserLength*cos(deg2rad*(confData.alphaLaser)),
					-confData.laserLength*sin(deg2rad*(confData.alphaLaser)));

	osg::Plane planeA(a1, a2, a3);
	osg::Plane planeB(b1, b2, b3);


	osg::Vec4d planeA_coeffs = planeA.asVec4();
	osg::Vec4d planeB_coeffs = planeB.asVec4();

	for (int i = 0; i < 4; i++) {
		_planeA[i] = (float)planeA_coeffs[i];
		_planeB[i] = (float)planeB_coeffs[i];
	}
	
}