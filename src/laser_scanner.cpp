//#include "stdafx.h"

#include "laser_scanner.h"

#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>

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

	struct Configuration confData; //struct contentente tutti i dati di configurazione
	double deg2rad = 2 * 3.1416 / 360;

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

	/////////////////////////////////////////////////////////////////////////////////////////

	// Caricamento del modello
	std::cout << "Caricamento del modello \"" << modelName << "\"...";
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelName);
	if (!model)
	{
		std::cout << "Nessun modello trovato con quel nome." << std::endl;
		_waitKey();
		return 1;
	}
	std::cout << " OK" << std::endl;

	// Ottimizzazione della mesh
	osgUtil::Optimizer optimizer;
	optimizer.optimize(model.get());

	// Calcolo delle dimensioni della mesh 
	osg::ComputeBoundsVisitor cbbv;
	model->accept(cbbv);
	osg::BoundingBox bb = cbbv.getBoundingBox();
	osg::Vec3 ModelSize = bb._max - bb._min;

	std::cout << "Dimensioni del modello:" << std::endl;
	std::cout << "  x_min: " << bb.xMin() << "\tx_max: " << bb.xMax() << "\tx: " << ModelSize[0] << "mm" << std::endl;
	std::cout << "  y_min: " << bb.yMin() << "\t\ty_max: " << bb.yMax() << "\ty: " << ModelSize[1] << "mm" << std::endl;
	std::cout << "  z_min: " << bb.zMin() << "\t\tz_max: " << bb.zMax() << "\tz: " << ModelSize[2] << "mm" << std::endl;

	//impostazione posizione iniziale della telecamera
	int cameraZ = (int) bb.zMin() + confData.cameraHeight; 
	int cameraX = (int) (bb.xMax() + bb.xMin())/2;


	if (!confData.useBounds) {	//calcolo bound ottimale per scansionare l'intero oggetto
		confData.minY = bb.yMin() - confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) + confData.laserDistance;
		confData.maxY = bb.yMax() + confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) - confData.laserDistance;
	}

	// Inizializzazione delle variabili necessarie per la visualizzazione della progressione
	float iter = 0;
	float total_space = confData.maxY - confData.minY;
	float space_between_frame = confData.scanSpeed/confData.fpsCam; // in millimetri
	float num_iterations = total_space / space_between_frame + 1;

	/////////////////////////////////////////////////////////////

/*	// Calcolo l'altezza del punto di intersezione dei piani laser. 
	float z_intersection = cameraZ - confData.laserDistance*tan(deg2rad*confData.alphaLaser);
	if (z_intersection <= bb.zMax()) {
		std::cout << "ATTENZIONE: l'altezza dei punti di intersezione tra i due piani laser " << std::endl;
		std::cout << "^^^^^^^^^^  e' inferiore all'altezza massima del modello caricato." << std::endl;
		std::cout << "^^^^^^^^^^  e' consigliato alzare l'altezza del sistema telecamera/laser di almeno " 
					<< ceil(bb.zMax() - z_intersection) << "mm." << std::endl;

		std::cout << "Procedere comunque (s/n)? ";
		char answer;
		while(true) {
			cin >> answer;
			if (answer == 's' || answer == 'S') {
				std::cout << "La scansione viene eseguita ignorando il suggerimento.\n";
				break;
			}
			
			else if (answer == 'n' || answer == 'N') {
					std::cout << "Programma terminato senza eseguire la scansione.\n";
					return 1;
				}
		}
	}*/
	//////////////////////////////////////////////////////////////
	// Ottenimento piani laser. Nel sistema di riferimento della telecamera, i piani non cambiano mai equazione 
	// e possono quindi essere calcolati una volta per tutte

	// Tre punti per piano laser
	osg::Vec3 a1(0,	confData.laserDistance,	0 );

	osg::Vec3 a2(0,		
		confData.laserDistance - confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser)),	
		-confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser)));

	osg::Vec3 a3(100,
		confData.laserDistance - confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser)),
		-confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser)));


	osg::Vec3 b1(0,	-confData.laserDistance, 0);

	osg::Vec3 b2(0,
		-confData.laserDistance + confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser)),
		-confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser)));

	osg::Vec3 b3(100,
		confData.laserDistance - confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser)),
		-confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser)));

	osg::Plane planeA(a1, a2, a3);
	osg::Plane planeB(b1, b2, b3);
	
	// Ottenimento dei coefficienti dell'equazioni dei piani laser
	osg::Vec4d planeA_coeffs = planeA.asVec4();
	osg::Vec4d planeB_coeffs = planeB.asVec4();
	
	/*// Conversione da vec4d to vector<double>
	std::vector<double> planeCoeffA, planeCoeffB;
	planeCoeffA.push_back(planeA_coeffs[0]);	planeCoeffB.push_back(planeB_coeffs[0]);
	planeCoeffA.push_back(planeA_coeffs[1]);	planeCoeffB.push_back(planeB_coeffs[1]);
	planeCoeffA.push_back(planeA_coeffs[2]);	planeCoeffB.push_back(planeB_coeffs[2]);
	planeCoeffA.push_back(planeA_coeffs[3]);	planeCoeffB.push_back(planeB_coeffs[3]);
*/
	// Inizializzazione point cloud da riempire
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fase iterativa. Per ogni frame viene scansionata l'immagine e inseriti i punti nella PC

	

	for (float cameraY = confData.minY; cameraY <= confData.maxY; cameraY += space_between_frame) {

		cv::Vec3f position(cameraX, cameraY, cameraZ);
		confData.cameraPos = position;
		// Istruzioni per l'aggiornamento dello stato su console
		float progress = ++iter / num_iterations * 100;
		printf("\rScansionando la posizione y=%2.2f. Completamento al %2.2f%% ", cameraY, progress);
		std::cout<<std::flush;

		// Chiamata al metodo per ottenere le catture della macchina fotografica.
		cv::Mat screenshot = reproject(model, confData);

		// Conversione dall'immagine allo spazio 3D
		convert_to_3d(screenshot, confData, planeA_coeffs, planeB_coeffs, cloud);
	
	}

	// Visualizzazione finale della point cloud
	std::cout << "Punti nella cloud " << cloud->points.size() << std::endl;
	pcl::visualization::PCLVisualizer pcl_viewer("PCL Viewer");
	pcl_viewer.addCoordinateSystem(0.1);
	pcl_viewer.addPointCloud<pcl::PointXYZ>(cloud, "input_cloud");
	pcl_viewer.setBackgroundColor(0.2, 0.2, 0.2);
	pcl_viewer.spin();

	cout << "Exit program." << endl;
	//_waitKey();

	return 0;

}
