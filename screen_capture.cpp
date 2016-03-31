// include OSG
#include "stdafx.h"	
#include <osgDB/ReadFile>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osgDB/WriteFile> 

// include STD
#include <math.h>       /* sin */
#include <vector>  //for std::vector
#include <string>

// include OCV
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>

class SnapImageDrawCallback : public osg::Camera::DrawCallback
{
public:

	SnapImageDrawCallback()
	{
		_snapImageOnNextFrame = false;
	}

	void setFileName(const std::string& filename) { _filename = filename; }
	const std::string& getFileName() const { return _filename; }

	void setSnapImageOnNextFrame(bool flag) { _snapImageOnNextFrame = flag; }
	bool getSnapImageOnNextFrame() const { return _snapImageOnNextFrame; }

	virtual void operator () (const osg::Camera& camera) const
	{
		if (!_snapImageOnNextFrame) return;

		int x = -329, y = -160, width = 2024, height = 1088;
		
		// bisognerebbe dargli in pasto all' operator la camera del viewer e anche un riferimento per portare fuori l'immagine che ricava
		/*x = camera.getViewport()->x();
		y = camera.getViewport()->y();
		width = camera.getViewport()->width();
		height = camera.getViewport()->height();*/
		

		osg::ref_ptr<osg::Image> image = new osg::Image;
		image->readPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE);

		/*if (osgDB::writeImageFile(*image, _filename))
		{
			std::cout << "Saved screen image to `" << _filename << "`" << std::endl;
		}*/

		_snapImageOnNextFrame = false;
	}

protected:

	std::string _filename;
	mutable bool _snapImageOnNextFrame;


};



void takeScreenshot(osg::Node* pRoot, std::string filename)
{
	osgViewer::Viewer viewer;
	viewer.setSceneData(pRoot);

	osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new SnapImageDrawCallback();
	viewer.getCamera()->setPostDrawCallback(snapImageDrawCallback.get());
	
	if (snapImageDrawCallback.get())
	{
		std::cout << "make screenshot" << std::endl;
		snapImageDrawCallback->setFileName(filename);
		snapImageDrawCallback->setSnapImageOnNextFrame(true);
	}
	else
	{
		std::cout << "Warning: no make screenshot" << std::endl;
	}


	// Create a matrix to specify a distance from the viewpoint.
	osg::Matrix trans;
	trans.makeTranslate(0., 0., -500);
	viewer.getCamera()->setViewMatrix(trans);
	viewer.frame();
}

// metodo main per un veloce debug
//int main(int, char **)
//{
//	
//	// Il nodo root è un group, un cui figlio è il modello 
//	osg::ref_ptr<osg::Group> root = new osg::Group;
//
//	// Caricamento del modello 
//	std::cout << "Loading Model from Disk." << std::endl;
//	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("cessna.osg");
//	std::cout << "Model Loaded. " << std::endl;
//	root->addChild(model.get());
//	
//	takeScreenshot(root, "../data/lollo.bmp");
//
//	return 0;
//}

