
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/Matrixd>

#include <math.h>       /* sin */
#include <vector>  //for std::vector

#include <osg/Point>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/TexGenNode>
#include <osgUtil/Optimizer>
#include <osgUtil/Optimizer>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgDB/Registry>

#include <opencv2/core/core.hpp>
osg::Vec3 getIntersection(double angle, int laserLength, osg::Vec3 source, osg::Vec3 center, osg::ref_ptr<osg::Node> model) {

	
	float X;
	if(angle<0) 
		X = center.x() - laserLength*tan(fabs(angle));
	else
		X = center.x() + laserLength*tan(fabs(angle));
	float Y = center.y();
	float Z = center.z();
	osg::Vec3 end(X, Y, Z);
	
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector= new osgUtil::LineSegmentIntersector(source,end);
	//intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.apply(*model.get());
	
	if(intersector->containsIntersections()) {
		osg::Vec3 point = intersector->getFirstIntersection().getLocalIntersectPoint();
		if((point[2]<500&&point[2]>200)) {			
			//std::cout << point[0] << " " << point[1] << " " << point[2] <<std::endl;
			return point;		
		}	
	}

	return osg::Vec3(0.0,0.0,0.0);	
}

int main(int, char **)
{
	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	int cameraX = 0;	// Posizione X e Y del setting non modificabili 
	int cameraY = 0;	// nel file di configurazione
	int cameraZ;		fs["cameraHeight"] >> cameraZ;
	int laserLength;	fs["laserLength"] >> laserLength;
	int	laserDistance;	fs["laserDistance"] >> laserDistance;


	int numLaser;	fs["numLaser"] >> numLaser;
	int scanSpeed;	fs["scanSpeed"] >> scanSpeed;
	int fpsCam;	fs["fpsCam"] >> fpsCam;
	int alphaLaser; fs["alphaLaser"] >> alphaLaser;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	float minAngle = fanLaser / numLaser;
	double deg2rad = 2*3.1416/360; 
	
	// Conversione di un modello [da rendere più flessibile in fase in configurazione]
	//system("osgconv ../data/bin1.stl ../data/bin1.osg");
		
	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;
	
	// Caricamento del modello 
	std::cout << "Loading Model from Disk." << std::endl;
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("../data/bin1.osg");
	std::cout << "Model Loaded. " << std::endl;
	root->addChild(model.get());
	osgUtil::Optimizer optimizer;  
	optimizer.optimize(model.get());
	
	osg::ref_ptr<osg::Vec3Array> inter_points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> inter_points2 = new osg::Vec3Array;	
	osg::Vec3 point;

	for(int i = 0; i<5; i++) {													//<<---------------------------prova ciclo

	double actualAngle=-fanLaser/2;
	cameraY = cameraY + 10;
	float laserX = cameraX, laserY = cameraY + laserDistance, laserZ = cameraZ;
	osg::Vec3 source(laserX, laserY, laserZ);					//punto di partenza del laser
	float centerX = laserX;
	float centerY = laserY - laserLength*sin(deg2rad*(90 - alphaLaser));
	float centerZ = laserZ - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center(centerX, centerY, centerZ);	

	float laser2X = cameraX, laser2Y = cameraY - laserDistance, laser2Z = cameraZ;
	osg::Vec3 source2(laser2X, laser2Y, laser2Z);						//punto di partenza del secondo laser
	float center2X = laser2X;
	float center2Y = laser2Y + laserLength*sin(deg2rad*(90 - alphaLaser));
	float center2Z = laser2Z - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::Vec3 center2(center2X, center2Y, center2Z);

	osg::Vec3 null(0.0,0.0,0.0);

	for (actualAngle; actualAngle <= fanLaser/2  ; actualAngle += minAngle) {
	
		point = getIntersection(deg2rad*actualAngle, laserLength, source, center, model);
		if (point!=null)
			inter_points->push_back(point);
		
		point = getIntersection(deg2rad*actualAngle, laserLength, source2, center2, model);
		if (point!=null)
			inter_points2->push_back(point);
			
	}
	
	osg::ref_ptr<osg::Geode> interGeode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> interGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	
	interGeometry->setVertexArray(inter_points.get());
	interGeometry->setColorArray(color.get());
	interGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	interGeometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, inter_points.get()->getNumElements()));

	interGeometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON ); 
	interGeode->addDrawable(interGeometry.get());
	root->addChild(interGeode.get());
	//2
	osg::ref_ptr<osg::Geode> inter2Geode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> inter2Geometry = new osg::Geometry();
	interGeode->addDrawable(inter2Geometry.get());
	root->addChild(inter2Geode.get());

	inter2Geometry->setVertexArray(inter_points2.get());
	inter2Geometry->setColorArray(color.get());
	inter2Geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	inter2Geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, inter_points2.get()->getNumElements()));
	inter2Geometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON ); 
}	
	
	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());

	viewer.run();
	return 0; 
}

