#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>

#include <osg/ComputeBoundsVisitor>
#include <osg/Point>
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

	int cameraHeight;		fs["cameraHeight"] >> cameraHeight;
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

	osg::ComputeBoundsVisitor cbbv;
	model->accept(cbbv);
	osg::BoundingBox bb = cbbv.getBoundingBox();
	osg::Vec3 ModelSize = bb._max - bb._min;
	std::cout << "Dimensioni del modello:" << std::endl;
	std::cout << "\tx_min: " << bb.xMin() << " x_max: " << bb.xMax() << "\tx: " << ModelSize[0] << "mm" << std::endl;
	std::cout << "\ty_min: " << bb.yMin() << " y_max: " << bb.yMax() << "\ty: " << ModelSize[1] << "mm" << std::endl;
	std::cout << "\tz_min: " << bb.zMin() << " z_max: " << bb.zMax() << "\tz: " << ModelSize[2] << "mm" << std::endl;

	int cameraX = (int) (bb.xMax() + bb.xMin())/2; std::cout<< "X=" << cameraX <<std::endl;
	int cameraZ = (int) bb.zMin() + cameraHeight; 

	root->addChild(model.get());

	std::vector<osg::ref_ptr<osg::Vec3Array> > bundle, bundle2;	// Vettore contenente tutti i punti necessari a disegnare le rette del fascio
	std::vector<osg::ref_ptr<osg::Geode> > bundleGeode, bundle2Geode;
	std::vector<osg::ref_ptr<osg::Geometry> > bundleGeometry, bundle2Geometry;

	osg::ref_ptr<osg::Group> bundleNode = new osg::Group, bundle2Node = new osg::Group;
	root->addChild(bundleNode.get());			//verso il meno
	//root->addChild(bundle2Node.get());		//verso il più

	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
	osgUtil::IntersectionVisitor iv;
	
	//for (float position = minY; position <= maxY; position += space_between_frame) {}
	

	int position=0; //////overrrrride

	int cameraY=position;

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
		
		

		// costruzione dell'intersector
		intersector = new osgUtil::LineSegmentIntersector(source, end);
		iv.setIntersector(intersector.get());
		iv.apply(*model.get());
		if (intersector->containsIntersections()) {
			point = intersector->getFirstIntersection().getLocalIntersectPoint();
			if (true) {
				inter_points->push_back(point);
				std::cout << point[0] << " " << point[1] << " " << point[2] << " " << actualAngle << std::endl;
			}
		}


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
		
		intersector = new osgUtil::LineSegmentIntersector(source2, end);
		iv.setIntersector(intersector.get());
		iv.apply(*model.get());
		if (intersector->containsIntersections()) {
			point = intersector->getFirstIntersection().getWorldIntersectPoint();
			if (true)
				inter_points2->push_back(point);
		}
		actualAngle += minAngle;

	}

	osg::ref_ptr<osg::Geode> interGeode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> interGeometry = new osg::Geometry();

	interGeometry->setVertexArray(inter_points.get());
	interGeometry->setColorArray(color.get());
	interGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	interGeometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, inter_points.get()->getNumElements()));
	interGeometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);

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
	inter2Geometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);

	
	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.run();
	

	return 0;
}

