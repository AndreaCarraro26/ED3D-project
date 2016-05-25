
#include "laser_scanner.h"
#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loading configuration parameters
	Configuration confData;
	read_config(confData);
	double deg2rad = 2 * 3.1416 / 360;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loading STL
	std::cout << "Loading model \"" << confData.modelName << "\"...";
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(confData.modelName);
	if (!model)
	{
		std::cout << "No model found with this name." << std::endl;
		_waitKey();
		return 1;
	}
	std::cout << " OK" << std::endl;

	// Mesh optimization
	osgUtil::Optimizer optimizer;
	optimizer.optimize(model.get());

	// Mesh dimensions calculation
	osg::ComputeBoundsVisitor cbbv;
	model->accept(cbbv);
	osg::BoundingBox bb = cbbv.getBoundingBox();
	osg::Vec3 modelSize = bb._max - bb._min;

	std::cout << "Model dimensions:" << std::endl;
	std::cout << "  x_min: " << bb.xMin() << "\tx_max: " << bb.xMax() << "\t\tx: " << modelSize[0] << "mm" << std::endl;
	std::cout << "  y_min: " << bb.yMin() << "\t\ty_max: " << bb.yMax() << "\t\ty: " << modelSize[1] << "mm" << std::endl;
	std::cout << "  z_min: " << bb.zMin() << "\t\tz_max: " << bb.zMax() << "\t\tz: " << modelSize[2] << "mm" << std::endl;

	// It's possible to configure the first and last position of the scan. If they are not defined, the entire mesh will be scanned
	if (!confData.useBounds) {	
		confData.minY = bb.yMin() - confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) + confData.baseline;
		confData.maxY = bb.yMax() + confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) - confData.baseline;
	}

	confData.laserLength = confData.cameraHeight / sin(deg2rad*confData.alphaLaser);														

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration phase (will override conf file loaded data)

	bool isConfigOK = false;
	while (!isConfigOK) {
		confData.cameraPos[0] = (bb.xMax() + bb.xMin()) / 2;
		confData.cameraPos[1] = confData.minY;
		confData.cameraPos[2] = bb.zMin() + confData.cameraHeight;

		if (!confData.useBounds) {	
		confData.minY = bb.yMin() - confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) + confData.baseline;
		confData.maxY = bb.yMax() + confData.cameraHeight*tan(deg2rad*(90 - confData.alphaLaser)) - confData.baseline;
		}


		// height of intersection of the planes
		float z_intersection = confData.cameraPos[2] - confData.baseline*tan(deg2rad*confData.alphaLaser);
		if (z_intersection <= bb.zMax()) {
			std::cout << "WARNING: the height of the intersecition between the lasers is  " << std::endl;
			std::cout << "^^^^^^^  less then the loaded model height." << std::endl;
			std::cout << "^^^^^^^  It's recommended to raise the camera/laser position at least of "
				<< ceil(bb.zMax() - z_intersection) << "mm." << std::endl << std::flush;

			std::cout << "Continue anyway? (y/n) ";
			std::string answer;
			while (true) {
				std::cin >> answer;
				if (answer == "y" || answer == "Y") {
					std::cout << "Scan will be compleated ignoring the suggestion.\n";
					break;
				}

				else if (answer == "n" || answer == "N") {
					//editconfig
					isConfigOK = false;
					edit_conf(confData);
					std::cin.clear();
					std::cin.ignore(std::cin.rdbuf()->in_avail());
					break;
				}
				std::cin.clear();
				std::cin.ignore(std::cin.rdbuf()->in_avail());
				std::cout << "Input not valid. Retry.\nContinue anyway? (y/n)" << std::endl;
			}
		}

		std::cout << "\nVisualizing actual scene (laser sources located at start position)\n\n";
		draw_lasers(model, confData);
	
		std::cout << "Actual configuration parameters: \n";
		std::cout << "Camera/Laser height: " << confData.cameraHeight << "mm"<< std::endl;
		std::cout << "Baseline: " << confData.baseline << "mm" << std::endl;
		std::cout << "Angle from XY plane: " << confData.alphaLaser << "degrees"<< std::endl;
		std::cout << "Y position at start: " << confData.minY << "mm" << std::endl;
		std::cout << "Y position at the end: " << confData.maxY << "mm" << std::endl;
		std::cout << "Intersection method used: " ;
		if (confData.useMoller)
			std::cout << "Moller-Trumbore\n" << std::endl;
		else
			std::cout << "Naive - OSG \n" << std::endl;
	
		std::cout << "Scan the scene with these values?" << std::endl;
		std::cout << "Otherwise, it's possible to manually modify them." << std::endl;
		std::cout << "Press (Y) to begin the scan, (N) to enter the configuration menu. " << std::endl;
		while (true) {
			std::string answer;
			std::cin >> answer;
			if (answer == "y" || answer == "Y") {
				isConfigOK = true;
				break;
			}

			else if (answer == "n" || answer == "N") {
				//editconfig
				isConfigOK = false;
				edit_conf(confData);
				std::cin.clear();
				std::cin.ignore(std::cin.rdbuf()->in_avail());
				break;
			}
			std::cin.clear();
			std::cin.ignore(100,'\n');
			std::cout << "Input not valid. Retry.\nPress (Y) to begin the scan, (N) to enter the configuration menu. " << std::endl;
		}

	}

	////////////////////////////////////////////////////////////////////////////////////////////////////

	// Initialization of variables used for progress count
	float iter = 0;
	float total_space = confData.maxY - confData.minY;
	float space_between_frame = confData.scanSpeed / confData.fpsCam; // in millimetri
	int num_iterations = ceil(total_space / space_between_frame);

	////////////////////////////////////////////////////////////////////////////////////////////////////

	// Equation of the laser plane
	float planeA[4];
	float planeB[4];

	calculatePlan(confData, planeA, planeB);
	/*
	OpenCLparam CLparam;
	init_moller(confData, CLparam);
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Iterative phase. For every frame, an image will be scanned and used to extract point XYZ coordinates

	// PointCloud to fill
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (float cameraY = confData.minY; cameraY <= confData.maxY; cameraY += space_between_frame) {

		confData.cameraPos[1] = cameraY;

		// Progress on console
		float progress = iter++ / num_iterations * 100;
		printf("\rScansionando la posizione y=%2.2f. Completamento al %2.2f%% ", cameraY, progress);
		std::cout << std::flush;

		// Scan the scene
		cv::Mat screenshot;
		/*
		if (confData.useMoller) 
			screenshot = reprojectMoller(CLparam, confData, numPolygon, numDirections, cameraY);
		else
			*/
			screenshot = reproject(model, confData);
		// Convert to 3D points
		convert_to_3d(screenshot, confData, planeA, planeB, cloud);

	}

	// Visualization
	std::cout << "\npoint in cloud " << cloud->points.size() << std::endl;
	pcl::visualization::PCLVisualizer pcl_viewer("PCL Viewer");
	pcl_viewer.addCoordinateSystem(0.1);
	pcl_viewer.addPointCloud<pcl::PointXYZ>(cloud, "input_cloud");
	pcl_viewer.setBackgroundColor(0.2, 0.2, 0.2);
	pcl_viewer.spin();
	pcl_viewer.close();

	std::string answer;
	while (true) {
		std::cout << "Save this configuration for further scan? (Y/N) ";
		std::cin >> answer;
		if (answer == "y" || answer == "Y") {
			save_config(confData);
			break;
		}
		if (answer == "n" || answer == "N") {
			std::cout << "Configuration not saved. " << std::endl;
			break;
		}
		std::cout << "Input not valid! Please type Y or N " << std::endl;
	}

	//_waitKey();

	return 0;

}