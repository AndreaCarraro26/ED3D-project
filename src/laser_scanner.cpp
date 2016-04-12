#include "stdafx.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <iostream>
#include <sstream>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "laser_scanner.h"


int main(int argc, char** argv)
{

	/////////////////////////////////////////////////////////////////////////////////////
	// lettura dei dati di configurazione
	int width = 2024;
	int height = 1088;

	double f_x = 4615.04;
	double f_y = 4615.51;
	double x_0 = 1113.41;
	double y_0 = 480.016;

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

	int numLaser;	fs["numLaser"] >> numLaser;
	int scanSpeed;	fs["scanSpeed"] >> scanSpeed;
	int fpsCam;	fs["fpsCam"] >> fpsCam;
	int alphaLaser; fs["alphaLaser"] >> alphaLaser;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	float minAngle = fanLaser / numLaser;
	double deg2rad = 2 * 3.1416 / 360;


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
	root->addChild(model.get());

	/////////////////////////////////////////////////////////////
	for(float position = cameraY; position < maxY; position += space_between_frame) {
		scan_scene(root, model, position);
		std::cout << position << std::endl;
	}

	osg::Matrix trans;
	trans.makeTranslate(0., 0., -500);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root);
	viewer.run();

	//while (!viewer.done())
	//{	
	//	viewer.getCamera()->setViewMatrix(trans);
	//	// Draw the next frame.
	//	viewer.frame();
	//}	

	return 0; 

}
