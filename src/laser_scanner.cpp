//#include "stdafx.h"
#include "laser_scanner.h"

#include <vector>
#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>

#include <osgViewer/Viewer>
//#include <osgViewer/ViewerEventHandlers>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sstream>

int main(int argc, char** argv)
{

	/////////////////////////////////////////////////////////////////////////////////////
	// lettura dei dati di configurazione
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	int cameraX = 0;	// Posizione X e Y del setting non modificabili 
	int cameraY; 		fs["minY"] >> cameraY;
	int maxY;			fs["maxY"] >> maxY;	
	int cameraZ;		fs["cameraHeight"] >> cameraZ;
	int laserLength;	fs["laserLength"] >> laserLength;
	int	laserDistance;	fs["laserDistance"] >> laserDistance;

	std::cout << cameraZ << std::endl;
	std::cout << laserLength << std::endl;
	std::cout << laserDistance << std::endl;

	int scanSpeed;	fs["scanSpeed"] >> scanSpeed;
	int fpsCam;	fs["fpsCam"] >> fpsCam;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	float time_between_frame = 1.0 / fpsCam;	// in secondi
	float space_between_frame = time_between_frame*scanSpeed; // in millimetri

	fs.release();

	/////////////////////////////////////////////////////////////////////////////////////////

	// Caricamento del modello
	std::cout << "Loading Model from Disk." << std::endl;
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("../data/bin1.stl");	
	if (!model)
	{
		std::cout << "No data loaded" << std::endl;
		return 1;
	}
	std::cout << "Model Loaded. " << std::endl;
	
	osgUtil::Optimizer optimizer;
	optimizer.optimize(model.get());

	osg::ref_ptr<osg::Group> root = new osg::Group;
	//root->addChild(model.get());
	
	osg::Matrix trans;
	
	/////////////////////////////////////////////////////////////
	
	for(float position = cameraY; position < maxY; position += space_between_frame) {
	
		osg::Vec4d planeA_coeffs, planeB_coeffs;
		scan_scene(root, model, position, &planeA_coeffs, &planeB_coeffs);
		
		//convert from vec4d to vector<double>
		std::vector<double> planeA;
		planeA.push_back(planeA_coeffs[0]);
		planeA.push_back(planeA_coeffs[1]);
		planeA.push_back(planeA_coeffs[2]);
		planeA.push_back(planeA_coeffs[3]);	
		
		
		std::vector<double> planeB;
		planeB.push_back(planeB_coeffs[0]);
		planeB.push_back(planeB_coeffs[1]);
		planeB.push_back(planeB_coeffs[2]);
		planeB.push_back(planeB_coeffs[3]);
		
		std::cout << position << std::endl;
		trans.makeLookAt(osg::Vec3d(0., position, cameraZ), osg::Vec3d(0., position, 0.), osg::Vec3d(0.0, position-100, 0.));
		
		cv::Mat pippo = get_pic(root, trans);
		std::stringstream ss;
		ss << "../data/Mat_debug";
		ss << position << ".bmp";
		cv::imwrite(ss.str(), pippo);
		
		root->removeChild(1);

		std::vector<cv::Vec3d> point = convert_to_3d(pippo, planeA);
		
	}

	osgViewer::Viewer viewer;
	viewer.setSceneData(root);
	viewer.run();

	return 0; 

}
