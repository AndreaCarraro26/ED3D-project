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

int _waitKey() {

	#ifdef _WIN32
		system("pause");
	#elif linux 
		system("read");
	#endif
		
	return 0; 
}

int main(int argc, char** argv)
{
	/////////////////////////////////////////////////////////////////////////////////////
	// lettura dei dati di configurazione
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	double deg2rad = 2 * 3.1416 / 360;

	std::string modelName; fs["model"] >> modelName;

	float cameraX = 0;	// Posizione X e Y del setting non modificabili 
	float cameraZ;		fs["cameraHeight"] >> cameraZ;

	int	alphaLaser;		fs["alphaLaser"] >> alphaLaser;
	float	laserDistance;	fs["laserDistance"] >> laserDistance;
	float laserLength;	fs["laserLength"] >> laserLength;

	bool ignoreHeight;		fs["ignoreHeight"] >> ignoreHeight;
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
	std::cout << "Caricamento del modello \"" << modelName << "\"...";
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelName);
	if (!model)
	{
		std::cout << "Nessuno modello trovato con quel nome." << std::endl;
		_waitKey();
		return 1;
	}
	std::cout << "OK " << std::endl;

	// Ottimizzazione della mesh
	osgUtil::Optimizer optimizer;
	optimizer.optimize(model.get());

	// Calcolo delle dimensioni della mesh 
	osg::ComputeBoundsVisitor cbbv;
	model->accept(cbbv);
	osg::BoundingBox bb = cbbv.getBoundingBox();
	osg::Vec3 ModelSize = bb._max - bb._min;

	// Il modello viene traslato in posizione centrale rispetto al sistema di riferimento 
	osg::ref_ptr<osg::MatrixTransform> node_to_intersect = new osg::MatrixTransform;
	node_to_intersect->setMatrix(osg::Matrix::translate(-bb.xMax()+(bb.xMax()-bb.xMin())/2, -bb.yMax() + (bb.yMax() - bb.yMin())/2, -bb.zMin()));	// traslazione del modello per imporre che poggi sul piano 0
	node_to_intersect->addChild(model.get());

	// Riottenimento delle dimensioni del modello e visualizzazione  
	osg::ComputeBoundsVisitor cbbx;
	node_to_intersect->accept(cbbx);
	osg::BoundingBox bbx = cbbx.getBoundingBox();
	ModelSize = bbx._max - bbx._min;

	std::cout << "Dimensioni del modello:" << std::endl;
	std::cout << "\tx_min: " << bbx.xMin() << " x_max: " << bbx.xMax() << " x: " << ModelSize[0] << "mm" << std::endl;
	std::cout << "\ty_min: " << bbx.yMin() << " y_max: " << bbx.yMax() << " y: " << ModelSize[1] << "mm" << std::endl;
	std::cout << "\tz_min: " << bbx.zMin() << " z_max: " << bbx.zMax() << " z: " << ModelSize[2] << "mm" << std::endl;

	if (!useBounds) {		// Se impostato nella configurazione, permette la scansione solo in un range prestabilito
		minY = bbx.yMin() - cameraZ*tan(deg2rad*(90 - alphaLaser)) + laserDistance;
		maxY = bbx.yMax() + cameraZ*tan(deg2rad*(90 - alphaLaser)) - laserDistance;
	}

	// Inizializzazione delle variabili necessarie per la visualizzazione della progressione
	float iter = 0;
	float total_space = maxY - minY;
	float num_iterations = total_space / space_between_frame + 1;

	// Istruzioni di OSG
	//osg::ref_ptr<osg::Group> root = new osg::Group;
	//root->addChild(model.get());

	/////////////////////////////////////////////////////////////

	// Calcolo l'altezza del punto di intersezione dei piani laser. 
	float z_intersection = cameraZ - laserDistance*tan(deg2rad*alphaLaser);
	if (z_intersection <= bbx.zMax()) {
		std::cout << "ATTENZIONE: l'altezza dei punti di intersezione tra i due piani laser " << std::endl;
		std::cout << "^^^^^^^^^^  e' inferiore all'altezza massima del modello caricato." << std::endl;
		std::cout << "^^^^^^^^^^  e' consigliato alzare l'altezza del sistema camera/laser di almeno " << ceil(bbx.zMax() - z_intersection) << "mm." << std::endl;
		if (!ignoreHeight) {
			_waitKey();
			return 1;
		}
		else std::cout << "La scansione viene eseguita ignorando il suggerimento. " << std::endl;
		
	}
	
	//////////////////////////////////////////////////////////////
	// Ottenimento piani laser. Lavorando in coordinate della camera, i piani non cambiano mai equazioni, e possone essere calcolati una volta per tutte

	// Tre punti per piano laser
	osg::Vec3 a1(0,		laserDistance,	0 );
	osg::Vec3 a2(0,		+laserDistance - laserLength*sin(deg2rad*(90 - alphaLaser)),	-laserLength*cos(deg2rad*(90 - alphaLaser)));
	osg::Vec3 a3(100,	+laserDistance - laserLength*sin(deg2rad*(90 - alphaLaser)),	-laserLength*cos(deg2rad*(90 - alphaLaser)));
	osg::Vec3 b1(0,		-laserDistance, 0);
	osg::Vec3 b2(0,		-laserDistance + laserLength*sin(deg2rad*(90 - alphaLaser)),	-laserLength*cos(deg2rad*(90 - alphaLaser)));
	osg::Vec3 b3(100,	+laserDistance - laserLength*sin(deg2rad*(90 - alphaLaser)),	-laserLength*cos(deg2rad*(90 - alphaLaser)));

	osg::Plane planeA(a1, a2, a3);
	osg::Plane planeB(b1, b2, b3);
	
	// Ottenimento dei coefficienti dell'equazioni dei piani laser
	osg::Vec4d planeA_coeffs = planeA.asVec4();
	osg::Vec4d planeB_coeffs = planeB.asVec4();
	//std::cout << "PlaneA coefficients calcolati una tantum: " << planeA_coeffs[0] << " " << planeA_coeffs[1] << " " << planeA_coeffs[2] << " " << planeA_coeffs[3] << std::endl;
	//std::cout << "PlaneB coefficients calcolati una tantum: " << planeB_coeffs[0] << " " << planeB_coeffs[1] << " " << planeB_coeffs[2] << " " << planeB_coeffs[3] << std::endl;

	// Conversione da vec4d to vector<double>
	std::vector<double> planeCoeffA, planeCoeffB;
	planeCoeffA.push_back(planeA_coeffs[0]);	planeCoeffB.push_back(planeB_coeffs[0]);
	planeCoeffA.push_back(planeA_coeffs[1]);	planeCoeffB.push_back(planeB_coeffs[1]);
	planeCoeffA.push_back(planeA_coeffs[2]);	planeCoeffB.push_back(planeB_coeffs[2]);
	planeCoeffA.push_back(planeA_coeffs[3]);	planeCoeffB.push_back(planeB_coeffs[3]);

	// Inizializzazione point cloud da riempire
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fase iterativa. Per ogni frame viene scansionata l'immagine e inseriti i punti nella PC
	for (float position = minY; position <= maxY; position += space_between_frame) {

		// Istruzioni per l'aggiornamento dello stato su console
		float progress = ++iter / num_iterations * 100;
		printf("\rScansionando la posizione y=%2.2f. Completamento al %2.2f%% ", position, progress);

		// Chiamata al metodo per ottenere le catture della macchina fotografica. Vengono forniti anche i piani.
		cv::Mat screenshot = reproject(node_to_intersect, position) ;
				
		//std::stringstream ss;
		//ss << "./exit/Mat_debug";
		//ss << position << ".bmp";
		//cv::imwrite(ss.str(), screenshot);

		// Conversione dall'immagine allo spazio 3D
		convert_to_3d(screenshot, position, planeCoeffA, planeCoeffB, cloud);
	
	}

	// Visualizzazione finale della point cloud
	std::cout << "Punti nella cloud " << cloud->points.size() << std::endl;
	pcl::visualization::PCLVisualizer pcl_viewer("PCL Viewer");
	pcl_viewer.addCoordinateSystem(0.1);
	pcl_viewer.addPointCloud<pcl::PointXYZ>(cloud, "input_cloud");
	pcl_viewer.setBackgroundColor(0.2, 0.2, 0.2);
	pcl_viewer.spin();

	cout << "Exit program." << endl;
	_waitKey();

	return 0;

}
