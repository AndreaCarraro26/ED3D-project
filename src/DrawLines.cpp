#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/Matrixd>

#include <osgUtil/Optimizer>


// include STD
#include <math.h>       /* sin */
#include <vector>  //for std::vector
#include <string>

// include OCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#define PI 3.1415926


int main()
{
	std::string confFile = "../data/Configuration.xml";
	
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	std::string modelName; fs["model"] >> modelName;
	int cameraX = 0;
	float cameraY = 0;
	int cameraZ;		fs["cameraHeight"] >> cameraZ;
	int laserLength;	fs["laserLength"] >> laserLength;
	int	laserDistance;	fs["laserDistance"] >> laserDistance;
	int minY; 			fs["minY"] >> minY;
	int maxY;			fs["maxY"] >> maxY;
	int numLaser;		fs["numLaser"] >> numLaser;
	int alphaLaser; 	fs["alphaLaser"] >> alphaLaser;
	float fanLaser;		fs["fanLaser"] >> fanLaser;

	int scanSpeed;		fs["scanSpeed"] >> scanSpeed;
	int fpsCam;			fs["fpsCam"] >> fpsCam;

	float time_between_frame = 1.0 / fpsCam;	// in secondi
	float space_between_frame = time_between_frame*scanSpeed; // in millimetri

	fs.release();
	float minAngle = fanLaser / numLaser;
	float deg2rad = 2 * 3.14 / 360;
	
	// Conversione di un modello [da rendere più flessibile in fase in configurazione]
	//system("osgconv ../data/bin1.stl ../data/bin1.osg");

	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;

	// Caricamento del modello 
	std::cout << "Loading Model from Disk." << std::endl;
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelName);
	if(model) std::cout << "Model Loaded. " << std::endl;
	root->addChild(model.get());

	std::vector<osg::ref_ptr<osg::Vec3Array> > bundle, bundle2;	// Vettore contenente tutti i punti necessari a disegnare le rette del fascio
	std::vector<osg::ref_ptr<osg::Geode> > bundleGeode, bundle2Geode;
	std::vector<osg::ref_ptr<osg::Geometry> > bundleGeometry, bundle2Geometry;

	osg::ref_ptr<osg::Group> bundleNode = new osg::Group, bundle2Node = new osg::Group;
	root->addChild(bundleNode.get());
	root->addChild(bundle2Node.get());

	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
	osgUtil::IntersectionVisitor iv;
	
	//for (float position = minY; position <= maxY; position += space_between_frame) {}
	

	int position=minY; //////overrrrride

	cameraY=position;

	osg::ref_ptr<osg::Vec3Array> inter_points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> inter_points2 = new osg::Vec3Array;
	osg::Vec3 point;

	float actualAngle = -fanLaser / 2;
	float laserX = cameraX, laserY = cameraY + laserDistance, laserZ = cameraZ;
	osg::Vec3 source(laserX, laserY, laserZ);					//punto di partenza del laser
	float centerX = laserX;
	float centerY = laserY - laserLength*sin(deg2rad*(90 - alphaLaser));
	float centerZ = laserZ - laserLength*cos(deg2rad*(90 - alphaLaser));

	float laser2X = cameraX, laser2Y = cameraY - laserDistance, laser2Z = cameraZ;
	osg::Vec3 source2(laser2X, laser2Y, laser2Z);						//punto di partenza del secondo laser
	float center2X = laser2X;
	float center2Y = laser2Y + laserLength*sin(deg2rad*(90 - alphaLaser));
	float center2Z = laser2Z - laserLength*cos(deg2rad*(90 - alphaLaser));
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));

	int i = 0;
	for (i; actualAngle <= fanLaser / 2; i++) {

		bundle.push_back(new osg::Vec3Array);
		bundleGeode.push_back(new osg::Geode());
		bundleGeometry.push_back(new osg::Geometry());

		// costruzione del fascio di rette
		bundle[i]->push_back(source);
		float X;
		if (actualAngle<0)
			X = centerX - laserLength*tan(deg2rad*fabs(actualAngle));
		else
			X = centerX + laserLength*tan(deg2rad*fabs(actualAngle));
		float Y = centerY;
		float Z = centerZ;
		osg::Vec3 end(X, Y, Z);
		bundle[i]->push_back(end);
		
		bundleGeometry[i]->setVertexArray(bundle[i].get());
		bundleGeometry[i]->setColorArray(color.get());
		bundleGeometry[i]->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		bundleGeometry[i]->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		bundleGeode[i]->addDrawable(bundleGeometry[i].get());
		bundleNode->addChild(bundleGeode[i].get());
		
		
/*
		// costruzione dell'intersector
		intersector = new osgUtil::LineSegmentIntersector(source, end);
		iv.setIntersector(intersector.get());
		iv.apply(*model.get());
		if (intersector->containsIntersections()) {
			point = intersector->getFirstIntersection().getLocalIntersectPoint();
			if (point[2]<500 && point[2]>200) {
				inter_points->push_back(point);
				std::cout << point[0] << " " << point[1] << " " << point[2] << " " << actualAngle << std::endl;
			}
		}
*/

		//2
		bundle2.push_back(new osg::Vec3Array);
		bundle2Geode.push_back(new osg::Geode());
		bundle2Geometry.push_back(new osg::Geometry());
		bundle2[i]->push_back(source2);

		if (actualAngle<0)
			X = center2X - laserLength*tan(deg2rad*fabs(actualAngle));
		else
			X = center2X + laserLength*tan(deg2rad*fabs(actualAngle));

		Y = center2Y;
		Z = center2Z;
		end.set(X, Y, Z);
		bundle2[i]->push_back(end);
		
		bundle2Geometry[i]->setVertexArray(bundle2[i].get());
		bundle2Geometry[i]->setColorArray(color.get());
		bundle2Geometry[i]->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		bundle2Geometry[i]->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		bundle2Geode[i]->addDrawable(bundle2Geometry[i].get());
		bundle2Node->addChild(bundle2Geode[i].get());
		/*
		intersector = new osgUtil::LineSegmentIntersector(source2, end);
		iv.setIntersector(intersector.get());
		iv.apply(*model.get());
		if (intersector->containsIntersections()) {
			point = intersector->getFirstIntersection().getWorldIntersectPoint();
			if (point[2]<500 && point[2]>200)
				inter_points2->push_back(point);
		}*/
		actualAngle += minAngle;

	}
/*
	osg::ref_ptr<osg::Geode> interGeode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> interGeometry = new osg::Geometry();

	interGeometry->setVertexArray(inter_points.get());
	interGeometry->setColorArray(color.get());
	interGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	interGeometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, inter_points.get()->getNumElements()));

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
	inter2Geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, inter_points2.get()->getNumElements()));
*/

	
	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.run();
	

	return 0;
}

