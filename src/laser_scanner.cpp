//#include "stdafx.h"
#include "laser_scanner.h"

#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>

#include <osgViewer/Viewer>
//#include <osgViewer/ViewerEventHandlers>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>	

int main(int argc, char** argv)
{

	/////////////////////////////////////////////////////////////////////////////////////
	// lettura dei dati di configurazione
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	int cameraX = 0;	// Posizione X e Y del setting non modificabili 
	int minY; 		fs["minY"] >> minY;
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

	int insert_index, model_index = 1;

	osg::ref_ptr<osg::Group> root = new osg::Group;
	//root->addChild(model.get());
	
	osg::Matrix trans;
	
	/////////////////////////////////////////////////////////////


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for(float position = minY; position <= maxY; position += space_between_frame) {

		root->insertChild(model_index, model.get());

		osg::Vec4d planeA_coeffs, planeB_coeffs;
		insert_index = scan_scene(root, model_index, position, &planeA_coeffs, &planeB_coeffs);
		
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
		
		osgViewer::Viewer viewosg;
		viewosg.setSceneData(root.get());
		viewosg.run();
		std::cout<<root->getNumChildren()<<std::endl;
		std::cout<<"drawed = "<<insert_index <<" model = "<< model_index <<std::endl;
		if(insert_index != model_index && root->removeChild(insert_index))
			std::cout<<"removed\n";


		std::vector<pcl::PointXYZ> points = convert_to_3d(pippo, planeA);
		cloud->insert(cloud->begin(), points.begin(), points.end());
	}

/*
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.addCoordinateSystem(0.1);
	viewer.addPointCloud<pcl::PointXYZ>(cloud,"input_cloud");
	viewer.setBackgroundColor(0.2,0.2,0.2);
	viewer.spin();*/

	
	return 0; 

}
