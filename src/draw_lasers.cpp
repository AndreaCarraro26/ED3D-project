#include "laser_scanner.h"

#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>

#include <osg/ComputeBoundsVisitor>
#include <osg/Point>
// include STD
#include <math.h>       /* sin */

// include OCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#define PI 3.1415926


int draw_lasers(osg::ref_ptr<osg::Node> model, Configuration params)
{
	
	float deg2rad = 2 * PI / 360;
	
	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;

	root->addChild(model.get());

	float cameraX = params.cameraPos[0];
	float cameraY = params.cameraPos[1];
	float cameraZ = params.cameraPos[2];

	std::vector<osg::ref_ptr<osg::Vec3Array> > bundle, bundle2;	// Vettore contenente tutti i punti necessari a disegnare le rette del fascio
	std::vector<osg::ref_ptr<osg::Geode> > bundleGeode, bundle2Geode;
	std::vector<osg::ref_ptr<osg::Geometry> > bundleGeometry, bundle2Geometry;

	osg::ref_ptr<osg::Group> bundleNode = new osg::Group, bundle2Node = new osg::Group;
	root->addChild(bundleNode.get());			//verso il meno
	root->addChild(bundle2Node.get());		//verso il più
	
	float laserX = cameraX, laserY = cameraY + params.laserDistance, laserZ = cameraZ;
	osg::Vec3 source(laserX, laserY, laserZ);					//punto di partenza del laser
	float centerX = laserX;
	float centerY = laserY - params.laserLength*cos(deg2rad*(params.alphaLaser));
	float centerZ = laserZ - params.laserLength*sin(deg2rad*(params.alphaLaser));

	float laser2X = cameraX, laser2Y = cameraY - params.laserDistance, laser2Z = cameraZ;
	osg::Vec3 source2(laser2X, laser2Y, laser2Z);						//punto di partenza del secondo laser
	float center2X = laser2X;
	float center2Y = laser2Y + params.laserLength*cos(deg2rad*(params.alphaLaser));
	float center2Z = laser2Z - params.laserLength*sin(deg2rad*(params.alphaLaser));
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));

	int i = 0;
	float actualAngle = -params.fanLaser / 2;
	float minAngle = params.fanLaser / params.numLaser;

	for (i; actualAngle <= params.fanLaser / 2; i++, actualAngle+=minAngle) {

		bundle.push_back(new osg::Vec3Array);
		bundleGeode.push_back(new osg::Geode());
		bundleGeometry.push_back(new osg::Geometry());

		// costruzione del fascio di rette
		bundle[i]->push_back(source);
		float X;
		if (actualAngle<0)
			X = centerX - params.laserLength*tan(deg2rad*fabs(actualAngle));
		else
			X = centerX + params.laserLength*tan(deg2rad*fabs(actualAngle));
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
			X = center2X - params.laserLength*tan(deg2rad*fabs(actualAngle));
		else
			X = center2X + params.laserLength*tan(deg2rad*fabs(actualAngle));

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

	viewer.setUpViewInWindow(0,0, 640, 480);
	viewer.realize();
	viewer.run();

	return 0;
}