//#include "stdafx.h"


#include <math.h> 
#include <vector>  

#include <osgDB/ReadFile>

#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Matrixd>
#include <osg/Point>
#include <osg/MatrixTransform>

#include <osgUtil/Optimizer>
#include <osgUtil/Optimizer>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include <osgViewer/Viewer>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

osg::Vec3 getIntersection(double angle, int laserLength, osg::Vec3 source, osg::Vec3 center, osg::ref_ptr<osg::Node> model) {

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


int scan_scene(osg::ref_ptr<osg::Group> root, int model_index, float positionY, 
				osg::Vec4d* planeA_coeffs, osg::Vec4d* planeB_coeffs) {
	
	////////////////////////////////////////////////////////////////
	// ottenimento dei dati dal file di configurazione
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	
	int cameraX = 0;
	float cameraY = positionY;
	int cameraZ;		fs["cameraHeight"] >> cameraZ;
	int laserLength;	fs["laserLength"] >> laserLength;
	int	laserDistance;	fs["laserDistance"] >> laserDistance;

	int numLaser;	fs["numLaser"] >> numLaser;
	int alphaLaser; fs["alphaLaser"] >> alphaLaser;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	fs.release();

	float minAngle = fanLaser / numLaser;
	double deg2rad = 2 * 3.1416 / 360;

	osg::ref_ptr<osg::Node> model = root->getChild(model_index);

	// allocazione dei punti di intersezione dei due laser
	osg::ref_ptr<osg::Vec3Array> inter_points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> inter_points2 = new osg::Vec3Array;
	osg::Vec3 point;

	// angolo iniziale di disegno del laser
	double actualAngle = -fanLaser / 2;		
	// definizione del punto di applicazione del fascio
	float laserX = cameraX, laserY = cameraY + laserDistance, laserZ = cameraZ;
	osg::Vec3 source(laserX, laserY, laserZ);
	// definizione del punto di arrivo del raggio centrale
	float centerX = laserX;
	float centerY = laserY - laserLength*sin(deg2rad*(90 - alphaLaser));
	float centerZ = laserZ - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center(centerX, centerY, centerZ);

	// stesso procedimento già visto, applicato al secondo laser
	float laser2X = cameraX, laser2Y = cameraY - laserDistance, laser2Z = cameraZ;
	osg::Vec3 source2(laser2X, laser2Y, laser2Z);						//punto di partenza del secondo laser
	float center2X = laser2X;
	float center2Y = laser2Y + laserLength*sin(deg2rad*(90 - alphaLaser));
	float center2Z = laser2Z - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center2(center2X, center2Y, center2Z);

	osg::Vec3 null(0.0, 0.0, 0.0);

	for (actualAngle; actualAngle <= fanLaser / 2; actualAngle += minAngle) {

		std::cout << actualAngle << std::endl;
		// intersezione del primo laser
		point = getIntersection(deg2rad*actualAngle, laserLength, source, center, model);
		if (point != null)
			inter_points->push_back(point);
		// intersezioni del secondo laser
		point = getIntersection(deg2rad*actualAngle, laserLength, source2, center2, model);
		if (point != null)
			inter_points2->push_back(point);

	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Rappresentazione dei punti di intersezione nella scena
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1.0, .0, .0, 1.0));

	osg::ref_ptr<osg::Geode> intersection_geode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> intersection_geometry_1 = new osg::Geometry();	// punti generati dal primo laser
	osg::ref_ptr<osg::Geometry> intersection_geometry_2 = new osg::Geometry();	// punti generati dal secondo laser

	intersection_geometry_1->setVertexArray(inter_points.get());
	intersection_geometry_1->setColorArray(color.get());
	intersection_geometry_1->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	intersection_geometry_1->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, inter_points.get()->getNumElements()));
	intersection_geometry_1->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);

	intersection_geometry_2->setVertexArray(inter_points2.get());
	intersection_geometry_2->setColorArray(color.get());
	intersection_geometry_2->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	intersection_geometry_2->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, inter_points2.get()->getNumElements()));
	intersection_geometry_2->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);

	intersection_geode->addDrawable(intersection_geometry_1.get());
	intersection_geode->addDrawable(intersection_geometry_2.get());
	
	int draw_index = model_index + 1;
	root->insertChild(draw_index, intersection_geode.get());
	//root->addChild(intersection_geode.get());
	//int draw_index = root->getChildIndex(intersection_geode.get());

	//calcolo e restituzione coeff del piano
	osg::Plane planeA(source, center, osg::Vec3(centerX+100, centerY, centerZ));
	osg::Plane planeB(source2, center2, osg::Vec3(center2X+100, center2Y, center2Z));
	
	*planeA_coeffs = planeA.asVec4();
	*planeB_coeffs = planeB.asVec4();

	return draw_index;
}

