
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osg/Matrixd>

#include <math.h>       /* sin */
#include <vector>  //for std::vector

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/TexGenNode>
#include <osgUtil/Optimizer>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgDB/Registry>

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
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.apply(*model.get());
	
	if(intersector->containsIntersections()) {
		osg::Vec3 point = intersector->getFirstIntersection().getLocalIntersectPoint();
		if((point[2]<500&&point[2]>200)||true) {			
			std::cout << point[0] << " " << point[1] << " " << point[2] <<std::endl;
			return point;		
		}	
	}

	return osg::Vec3(0.0,0.0,0.0);	
}

int main(int, char **)
{
	int cameraX = 0;
	int cameraY = 0;
	int cameraZ = 2000;
	int laserLength = 3000;
	int	laserDistance = 500; // Da 500 a 800mm

	int numLaser = 700;		// numero di linee che compongono il fascio
	int scanSpeed;	//  Valori 100 mm/s – 1000 mm/s
	int fpsCam;		// Valori 100 fps – 500 fps
	double alphaLaser = 70;	// Valori 60° - 70°
	double fanLaser = 45;		// Valori 30° - 45°
	
	double minAngle = fanLaser / numLaser;
	double deg2rad = 2*M_PI/360; 
	// Conversione di un modello [da rendere più flessibile in fase in configurazione]
	//system("osgconv ../data/bin1.stl ../data/bin1.osg");
		
	// Il nodo root è un group, un cui figlio è il modello 
	osg::ref_ptr<osg::Group> root = new osg::Group;
	
	// Caricamento del modello 
	std::cout << "Loading Model from Disk." << std::endl;
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("../data/bin1.osg");
	std::cout << "Model Loaded. " << std::endl;
	//root->addChild(model.get());

	osg::ref_ptr<osg::Vec3Array> inter_points = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec3Array> inter_points2 = new osg::Vec3Array;	
	osg::Vec3 point;
	
	double actualAngle=-fanLaser/2;

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
	int i = 0;
	for (i; actualAngle <= fanLaser/2  ; i++) {
	
		point = getIntersection(deg2rad*actualAngle, laserLength, source, center, model);
		if (point!=null)
			inter_points->push_back(point);
		
		point = getIntersection(deg2rad*actualAngle, laserLength, source2, center2, model);
		if (point!=null)
			inter_points2->push_back(point);
		
		actualAngle += minAngle;
		
	}

	osg::ref_ptr<osg::Geode> interGeode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> interGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
	
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
	double alpha = 4615.04; int img_width = 2024;
	double beta = 4615.51;	int img_height = 1088;
	double skew = 0;		double near_clip = 100;
	double u0 = 1113.41;	double far_clip = 1000;
	double v0 = 480.016;
	intrinsic2projection(frustum, alpha, beta, skew, u0, v0, img_width, img_height, near_clip, far_clip);

	osgViewer::Viewer viewer;
	viewer.setSceneData(root.get());
	//viewer.getCamera()->setProjectionMatrix(frustum); 
		
	// Create a matrix to specify a distance from the viewpoint.
	osg::Matrix trans;
	trans.makeTranslate(0., 0., -cameraZ);
	// Rotation angle (in radians)
	double angle(0.);

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

