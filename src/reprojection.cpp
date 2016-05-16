//#include "stdafx.h"

#include <math.h> 
#include <vector>  

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include <osgViewer/Viewer>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"


osg::Vec3 getInter(double angle, int laserLength, osg::Vec3 source, osg::Vec3 center, osg::ref_ptr<osg::Node> model) {

	float X;

	if (angle<0)
		X = center.x() - laserLength*tan(fabs(angle));
	else
		X = center.x() + laserLength*tan(fabs(angle));
	float Y = center.y();
	float Z = center.z();
	osg::Vec3 end(X, Y, Z);

	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(source, end);
	//intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.apply(*model.get());

	if (intersector->containsIntersections()) {
		osg::Vec3 point = intersector->getFirstIntersection().getLocalIntersectPoint();
		return point;
	}

	return osg::Vec3(0.0, 0.0, 0.0);
}


cv::Mat reproject(osg::ref_ptr<osg::Node> model, float positionY)	{ 

	////////////////////////////////////////////////////////////////
	// ottenimento dei dati dal file di configurazione
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	double deg2rad = 2 * 3.1416 / 360;

	int numLaser;	fs["numLaser"] >> numLaser;
	int alphaLaser; fs["alphaLaser"] >> alphaLaser;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	int cameraX = 0;
	float cameraY = positionY;
	int cameraZ;		fs["cameraHeight"] >> cameraZ;
	double laserLength;	fs["laserLength"] >> laserLength;
	
	int	laserDistance;	fs["laserDistance"] >> laserDistance;

	double f_x;			fs["sensor_f_x"] >> f_x;
	double f_y;			fs["sensor_f_y"] >> f_y;
	double x_0;			fs["sensor_x_0"] >> x_0;
	double y_0;			fs["sensor_y_0"] >> y_0;

	int width;			fs["sensor_width"] >> width;
	int height;			fs["sensor_height"] >> height;

	fs.release();

	cv::Mat intrinsic(3, 3, CV_64FC1);

	// Popolamento della matrice degli intrinsici
	intrinsic.at<double>(0, 0) = f_x;
	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(0, 2) = x_0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(1, 1) = f_y;
	intrinsic.at<double>(1, 2) = y_0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	float minAngle = fanLaser / numLaser;
		
	osg::ref_ptr<osg::Group> root;

	// allocazione dei punti di intersezione dei due laser
	osg::ref_ptr<osg::Vec3Array> inter_points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> inter_points2 = new osg::Vec3Array;

	osg::Vec3 point;
	cv::Vec3f pointCV;

	// array per i punti, ma std
	std::vector<cv::Vec3f> intersection_points;
	
	// angolo iniziale di disegno del laser
	float actualAngle = -fanLaser / 2;
	// definizione del punto di applicazione del fascio
	float laserX = cameraX, laserY = cameraY + laserDistance, laserZ = cameraZ;
	osg::Vec3 source(laserX, laserY, laserZ);
	// definizione del punto di arrivo del raggio centrale
	float centerX = laserX;
	float centerY = laserY - laserLength*sin(deg2rad*(90 - alphaLaser));
	float centerZ = laserZ - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center(centerX, centerY, centerZ);

	// stesso procedimento gi� visto, applicato al secondo laser
	float laser2X = cameraX, laser2Y = cameraY - laserDistance, laser2Z = cameraZ;
	osg::Vec3 source2(laser2X, laser2Y, laser2Z);						//punto di partenza del secondo laser
	float center2X = laser2X;
	float center2Y = laser2Y + laserLength*sin(deg2rad*(90 - alphaLaser));
	float center2Z = laser2Z - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center2(center2X, center2Y, center2Z);

	osg::Vec3 null(0.0, 0.0, 0.0);

	for (actualAngle; actualAngle <= fanLaser / 2; actualAngle += minAngle) {

		//std::cout << actualAngle << std::endl;
		// intersezione del primo laser
		point = getInter(deg2rad*actualAngle, laserLength, source, center, model);
		if (point != null) {
			pointCV[0] = point.x() - cameraX;
			pointCV[1] = point.y() - cameraY;
			pointCV[2] = point.z() - cameraZ;

			intersection_points.push_back(pointCV);
		}
		
		// intersezioni del secondo laser
		point = getInter(deg2rad*actualAngle, laserLength, source2, center2, model);
		if (point != null) {
			pointCV[0] = point.x() - cameraX;
			pointCV[1] = point.y() - cameraY;
			pointCV[2] = point.z() - cameraZ;

			intersection_points.push_back(pointCV);
		}

	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;	rVec.at<double>(1) = 0;	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
	distCoeffs.at<double>(0) = 0 ; 	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0; 	distCoeffs.at<double>(3) = 0;
	distCoeffs.at<double>(4) = 0;

	std::vector<cv::Vec2f> imagePoints;	
	cv::Mat image_to_return(height, width, CV_8UC1);
	image_to_return.setTo(0);

	if (intersection_points.size()!=0)
		cv::projectPoints(intersection_points, rVec, tVec, intrinsic, distCoeffs, imagePoints);

	//std::cout << imagePoints.size() << std::endl;

	float x, y;
	for (int i = 0; i < imagePoints.size(); i++) {
		x = imagePoints[i][0];
		y = imagePoints[i][1];
		if (0<((int)x) && ((int)x) < width && 0<((int)y) && ((int)y) < height) {
			//std::cout << (int)x << " " << (int)y << std::endl;
			image_to_return.at<uchar>((int)y, (int)x) = 255;
		}
	}

	return image_to_return;
}
