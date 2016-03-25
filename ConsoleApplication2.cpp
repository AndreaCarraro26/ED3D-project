// include OSG
#include "stdafx.h"
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/Matrixd>

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/TexGenNode>
#include <osgUtil/Optimizer>
#include <osgDB/Registry>

// include STD
#include <math.h>       /* sin */
#include <vector>  //for std::vector
#include <string>

// include OCV
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>

/**
@brief basic function to produce an OpenGL projection matrix and associated viewport parameters
which match a given set of camera intrinsics. This is currently written for the Eigen linear
algebra library, however it should be straightforward to port to any 4x4 matrix class.
@param[out] frustum Eigen::Matrix4d projection matrix.  Eigen stores these matrices in column-major (i.e. OpenGL) order.
@param[out] viewport 4-component OpenGL viewport values, as might be retrieved by glGetIntegerv( GL_VIEWPORT, &viewport[0] )
@param[in]  alpha x-axis focal length, from camera intrinsic matrix
@param[in]  alpha y-axis focal length, from camera intrinsic matrix
@param[in]  skew  x and y axis skew, from camera intrinsic matrix
@param[in]  u0 image origin x-coordinate, from camera intrinsic matrix
@param[in]  v0 image origin y-coordinate, from camera intrinsic matrix
@param[in]  img_width image width, in pixels
@param[in]  img_height image height, in pixels
@param[in]  near_clip near clipping plane z-location, can be set arbitrarily > 0, controls the mapping of z-coordinates for OpenGL
@param[in]  far_clip  far clipping plane z-location, can be set arbitrarily > near_clip, controls the mapping of z-coordinate for OpenGL
*/ 

//void build_opengl_projection_for_intrinsics(Eigen::Matrix4d &frustum, int *viewport, double alpha, double beta, double skew, double u0, double v0, int img_width, int img_height, double near_clip, double far_clip) {
//
//	 These parameters define the final viewport that is rendered into by
//	 the camera.
//	double L = 0;
//	double R = img_width;
//	double B = 0;
//	double T = img_height;
//
//	 near and far clipping planes, these only matter for the mapping from
//	 world-space z-coordinate into the depth coordinate for OpenGL
//	double N = near_clip;
//	double F = far_clip;
//
//	 set the viewport parameters
//	viewport[0] = L;
//	viewport[1] = B;
//	viewport[2] = R - L;
//	viewport[3] = T - B;
//
//	 construct an orthographic matrix which maps from projected
//	 coordinates to normalized device coordinates in the range
//	 [-1, 1].  OpenGL then maps coordinates in NDC to the current
//	 viewport
//	Eigen::Matrix4d ortho = Eigen::Matrix4d::Zero();
//	ortho(0, 0) = 2.0 / (R - L); ortho(0, 3) = -(R + L) / (R - L);
//	ortho(1, 1) = 2.0 / (T - B); ortho(1, 3) = -(T + B) / (T - B);
//	ortho(2, 2) = -2.0 / (F - N); ortho(2, 3) = -(F + N) / (F - N);
//	ortho(3, 3) = 1.0;
//
//	 construct a projection matrix, this is identical to the 
//	 projection matrix computed for the intrinsicx, except an
//	 additional row is inserted to map the z-coordinate to
//	 OpenGL. 
//	Eigen::Matrix4d tproj = Eigen::Matrix4d::Zero();
//	tproj(0, 0) = alpha; tproj(0, 1) = skew; tproj(0, 2) = u0;
//	tproj(1, 1) = beta; tproj(1, 2) = v0;
//	tproj(2, 2) = -(N + F); tproj(2, 3) = -N*F;
//	tproj(3, 2) = 1.0;
//
//	 resulting OpenGL frustum is the product of the orthographic
//	 mapping to normalized device coordinates and the augmented
//	 camera intrinsic matrix
//	frustum = ortho*tproj;
//
//	 glMatrixMode(GL_PROJECTION);
//	 glLoadMatrixd(&frustum(0, 0));
//}

#define PI 3.1415926

void intrinsic2projection(osg::Matrixd &frustum, double alpha, double beta, double skew, double u0, double v0, int img_width, int img_height, double near_clip, double far_clip)	 {

	// These parameters define the final viewport that is rendered into by
	// the camera.
	double L = 0;
	double R = img_width;
	double B = 0;
	double T = img_height;

	// near and far clipping planes, these only matter for the mapping from
	// world-space z-coordinate into the depth coordinate for OpenGL
	double N = near_clip;
	double F = far_clip;

	// construct an orthographic matrix which maps from projected
	// coordinates to normalized device coordinates in the range
	// [-1, 1].  OpenGL then maps coordinates in NDC to the current
	// viewport
	osg::Matrixd ortho; 
	ortho(0, 0) = 2.0 / (R - L); ortho(0, 3) = -(R + L) / (R - L);
	ortho(1, 1) = 2.0 / (T - B); ortho(1, 3) = -(T + B) / (T - B);
	ortho(2, 2) = -2.0 / (F - N); ortho(2, 3) = -(F + N) / (F - N);
	ortho(3, 3) = 1.0;

	// construct a projection matrix, this is identical to the 
	// projection matrix computed for the intrinsicx, except an
	// additional row is inserted to map the z-coordinate to
	// OpenGL. 
	osg::Matrixd tproj;
	tproj(0, 0) = alpha; tproj(0, 1) = skew; tproj(0, 2) = u0;
	tproj(1, 1) = beta; tproj(1, 2) = v0;
	tproj(2, 2) = -(N + F); tproj(2, 3) = -N*F;
	tproj(3, 2) = 1.0;

	// resulting OpenGL frustum is the product of the orthographic
	// mapping to normalized device coordinates and the augmented
	// camera intrinsic matrix
	frustum = ortho*tproj;

	// glMatrixMode(GL_PROJECTION);
	// glLoadMatrixd(&frustum(0, 0));
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

	std::cout << cameraZ << std::endl;
	std::cout << laserLength << std::endl;
	std::cout << laserDistance << std::endl;

	int numLaser;	fs["numLaser"] >> numLaser;
	int scanSpeed;	fs["scanSpeed"] >> scanSpeed;
	int fpsCam;		fs["fpsCam"] >> fpsCam;
	int alphaLaser; fs["alphaLaser"] >> alphaLaser;
	float fanLaser;	fs["fanLaser"] >> fanLaser;

	float minAngle = fanLaser / numLaser;
	float deg2rad = 2 * 3.14 / 360;
	
	// Conversione di un modello [da rendere più flessibile in fase in configurazione]
	//system("osgconv ../data/bin1.stl ../data/bin1.osg");

	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;

	// Caricamento del modello 
	std::cout << "Loading Model from Disk." << std::endl;
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("../data/bin1.osg");
	std::cout << "Model Loaded. " << std::endl;
	root->addChild(model.get());

	std::vector<osg::ref_ptr<osg::Vec3Array> > bundle, bundle2;	// Vettore contenente tutti i punti necessari a disegnare le rette del fascio
	std::vector<osg::ref_ptr<osg::Geode> > bundleGeode, bundle2Geode;
	std::vector<osg::ref_ptr<osg::Geometry> > bundleGeometry, bundle2Geometry;

	osg::ref_ptr<osg::Group> bundleNode = new osg::Group, bundle2Node = new osg::Group;
	//root->addChild(bundleNode.get());
	//root->addChild(bundle2Node.get());

	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
	osgUtil::IntersectionVisitor iv;


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
		/*
		bundleGeometry[i]->setVertexArray(bundle[i].get());
		bundleGeometry[i]->setColorArray(color.get());
		bundleGeometry[i]->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		bundleGeometry[i]->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		bundleGeode[i]->addDrawable(bundleGeometry[i].get());
		bundleNode->addChild(bundleGeode[i].get());
		*/

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
		/*
		bundle2Geometry[i]->setVertexArray(bundle2[i].get());
		bundle2Geometry[i]->setColorArray(color.get());
		bundle2Geometry[i]->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		bundle2Geometry[i]->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		bundle2Geode[i]->addDrawable(bundle2Geometry[i].get());
		bundle2Node->addChild(bundle2Geode[i].get());
		*/
		intersector = new osgUtil::LineSegmentIntersector(source2, end);
		iv.setIntersector(intersector.get());
		iv.apply(*model.get());
		if (intersector->containsIntersections()) {
			point = intersector->getFirstIntersection().getWorldIntersectPoint();
			if (point[2]<500 && point[2]>200)
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


	//osg::ref_ptr<osg::Matrixd> frustum = new osg::Matrixd;
	//	|a	g	u| |4615.04		0	1113.41	|
	//	|0	B	v| |0		4615.51	480.016	|
	//	|0	0	1| |0			0		1	|
	osg::Matrixd frustum;
	float alpha = 4615.04; int img_width = 2024;
	float beta = 4615.51;	int img_height = 1088;
	float skew = 0;		float near_clip = 100;
	float u0 = 1113.41;	float far_clip = 1000;
	float v0 = 480.016;
	intrinsic2projection(frustum, alpha, beta, skew, u0, v0, img_width, img_height, near_clip, far_clip);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	//viewer.getCamera()->setProjectionMatrix(frustum); 

	// Create a matrix to specify a distance from the viewpoint.
	osg::Matrix trans;
	trans.makeTranslate(0., 0., -cameraZ);
	// Rotation angle (in radians)
	float angle(0.);

	viewer.getCamera()->setViewMatrix(trans);
	viewer.run();

	//while (!viewer.done())
	//{	// Create the rotation matrix.
	//	osg::Matrix rot;
	//	rot.makeRotate(angle, osg::Vec3(1., 0., 0.));
	//	angle += 0.01;
	//	// Set the view matrix (the concatenation of the rotation and
	//	// translation matrices).
	//	viewer.getCamera()->setViewMatrix(trans);
	//	// Draw the next frame.
	//	viewer.frame();
	//}	

	return 0;
}

