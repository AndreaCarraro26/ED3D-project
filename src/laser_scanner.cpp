#include "stdafx.h"
#include "laser_scanner.h"

#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>

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
	double deg2rad = 2 * 3.1416 / 360;

	std::string modelName; fs["model"] >> modelName;

	int cameraX = 0;	// Posizione X e Y del setting non modificabili 
	int cameraZ;		fs["cameraHeight"] >> cameraZ;

	int	alphaLaser;		fs["alphaLaser"] >> alphaLaser;
	int	laserDistance;	fs["laserDistance"] >> laserDistance;
	//double laserLength = 6000;	//laserLength = cameraZ / (double)sin(deg2rad*alphaLaser) +10 ; //fs["laserLength"] >> laserLength;

	bool useBounds;		fs["useBounds"] >> useBounds;
	int minY;			fs["minY"] >> minY; 
	int maxY;			fs["maxY"] >> maxY;

	int scanSpeed;		fs["scanSpeed"] >> scanSpeed;
	int fpsCam;			fs["fpsCam"] >> fpsCam;
	float fanLaser;		fs["fanLaser"] >> fanLaser;

	float time_between_frame = 1.0 / fpsCam;	// in secondi
	float space_between_frame = time_between_frame*scanSpeed; // in millimetri

	
	

	fs.release();

	/////////////////////////////////////////////////////////////////////////////////////////

	// Caricamento del modello
	std::cout << "Caricamento del modello \""<< modelName << "\"...";
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelName);
	if (!model)
	{
		std::cout << "Nessuno modello trovato con quel nome." << std::endl;
		return 1;
	}
	std::cout << "OK " << std::endl;

	osgUtil::Optimizer optimizer;
	optimizer.optimize(model.get());

	// calcolo dimensioni della mesh 
	osg::ComputeBoundsVisitor cbbv;
	model->accept(cbbv);
	osg::BoundingBox bb = cbbv.getBoundingBox();
	osg::Vec3 ModelSize = bb._max - bb._min;

	std::cout << "Dimensioni del modello:" << std::endl;
	std::cout << "x_min: " << bb.xMin() << " x_max: " << bb.xMax() << " x: " << ModelSize[0] << "mm" << std::endl;
	std::cout << "y_min: " << bb.yMin() << " y_max: " << bb.yMax() << " y: " << ModelSize[1] << "mm" << std::endl;
	std::cout << "z_min: " << "0" << " z_max: " << ModelSize[2] << " z: " << ModelSize[2] << "mm" << std::endl;
	std::cout << "-----z_min: " << bb.zMin() << " z_max: " << bb.zMax() << " z: " << ModelSize[2] << "mm" << std::endl;

	if (!useBounds) {
		minY = bb.yMin() - cameraZ*tan(deg2rad*(90 - alphaLaser)) + laserDistance ;
		maxY = bb.yMax() + cameraZ*tan(deg2rad*(90 - alphaLaser)) - laserDistance ;
	}

	float space_spanned = 0;
	float space_to_run = maxY - minY;

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(model.get());
	//root->insertChild(model_index, model.get());

	osg::Matrix trans;

	osg::ref_ptr<osg::MatrixTransform> node_to_intersect = new osg::MatrixTransform;
	node_to_intersect->setMatrix(osg::Matrix::translate(.0f, 0.0f, -bb.zMin()));	// traslazione del modello per imporre che poggi sul piano 0
	node_to_intersect->addChild(model.get());

	/////////////////////////////////////////////////////////////

	// Creazione point cloud da riempire
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (float position = minY; position <= maxY; position += space_between_frame) {
		
		osg::Vec4d planeA_coeffs, planeB_coeffs;
		osg::ref_ptr<osg::Geode> point_node = new osg::Geode;
		std::cout<<"Finding intersection and capturing image...";
		cv::Mat screenshot = reproject(node_to_intersect, position, planeA_coeffs, planeB_coeffs);
		std::cout<<" done.\n";
		//convert from vec4d to vector<double>
		std::vector<double> planeA;
		planeA.push_back(planeA_coeffs[0]);
		planeA.push_back(planeA_coeffs[1]);
		planeA.push_back(planeA_coeffs[2]);
		planeA.push_back(planeA_coeffs[3]);
		std::cout << "PlaneA coefficients: " << planeA[0] <<" "<< planeA[1] <<" "<< planeA[2] <<" "<< planeA[3] << std::endl;


		std::vector<double> planeB;
		planeB.push_back(planeB_coeffs[0]);
		planeB.push_back(planeB_coeffs[1]);
		planeB.push_back(planeB_coeffs[2]);
		planeB.push_back(planeB_coeffs[3]); 
		std::cout << "PlaneB coefficients: " << planeB[0] <<" "<< planeB[1] <<" "<< planeB[2] <<" "<< planeB[3] << std::endl;

		std::cout << position << std::endl;

		trans.makeLookAt(osg::Vec3d(0., position, -cameraZ), osg::Vec3d(0., position, 0.), osg::Vec3d(0.0, position - 100, 0.));

		//VISUAL di prova NON SERVE
	//	root->addChild(point_node);
	//	osgViewer::Viewer viewosg;
	//	viewosg.setSceneData(root.get());
	//	viewosg.run();

		//cv::Mat pippo = get_pic(point_node, trans);

		std::stringstream ss;
		ss << "../data/Mat_debug";
		ss << position << ".bmp";
		cv::imwrite(ss.str(), screenshot);

		convert_to_3d(screenshot, planeA, planeB, cloud);

		space_spanned = space_spanned + space_between_frame;
		float progress = space_spanned / space_to_run * 100;
		printf("\rCompletamento: %2.2f%%", progress);
	}
	
	std::cout<<"Punti nella cloud "<<cloud->points.size()<<std::endl;
	// visualizzazione
	pcl::visualization::PCLVisualizer pcl_viewer("PCL Viewer");
	pcl_viewer.addCoordinateSystem(0.1);
	pcl_viewer.addPointCloud<pcl::PointXYZ>(cloud,"input_cloud");
	pcl_viewer.setBackgroundColor(0.2,0.2,0.2);
	pcl_viewer.spin();

	cout << "Exit program." << endl;

	return 0;

}
