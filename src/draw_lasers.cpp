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


int draw_lasers(osg::ref_ptr<osg::Node> model, osg::Vec3 cameraPos, int laserLength, int laserDistance)
{
	std::string confFile = "../data/Configuration.xml";
	
	float minAngle = fanLaser / numLaser;
	float deg2rad = 2 * PI / 360;
	
	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;

	root->addChild(model.get());

	float cameraX = cameraPos.x();
	float cameraY = cameraPos.y();
	float cameraZ = cameraPos.z();

	std::vector<osg::ref_ptr<osg::Vec3Array> > bundle, bundle2;	// Vettore contenente tutti i punti necessari a disegnare le rette del fascio
	std::vector<osg::ref_ptr<osg::Geode> > bundleGeode, bundle2Geode;
	std::vector<osg::ref_ptr<osg::Geometry> > bundleGeometry, bundle2Geometry;

	osg::ref_ptr<osg::Group> bundleNode = new osg::Group, bundle2Node = new osg::Group;
	root->addChild(bundleNode.get());			//verso il meno
	root->addChild(bundle2Node.get());		//verso il più

	int position=0; //////overrrrride

	//cameraY=position;

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
	}

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	viewer.run();
	

	return 0;
}