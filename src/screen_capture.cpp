//#include "stdafx.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <iostream>
#include <sstream>
#include <string.h>

#include <opencv2/core/core.hpp>


cv::Mat mat_to_return;
int has_read = 0;

	
class WindowCaptureCallback : public osg::Camera::DrawCallback
{
public:

	struct ContextData : public osg::Referenced
	{

		osg::ref_ptr<osg::Image> image;

		osg::GraphicsContext*   _gc;
		GLenum                  _readBuffer;

		GLenum                  _pixelFormat;
		GLenum                  _type;
		int                     _width;
		int                     _height;


		ContextData(osg::GraphicsContext* gc, GLenum readBuffer) :
			_gc(gc),
			_readBuffer(readBuffer),
			_pixelFormat(GL_BGRA),
			_type(GL_UNSIGNED_BYTE),
			_width(0),
			_height(0)
		{
			if (gc->getTraits())
			{
				if (gc->getTraits()->alpha)
				{
					osg::notify(osg::NOTICE) << "Select GL_BGRA read back format" << std::endl;
					_pixelFormat = GL_BGRA;
				}
				else
				{
					osg::notify(osg::NOTICE) << "Select GL_BGR read back format" << std::endl;
					_pixelFormat = GL_BGR;
				}
			}

			if (gc->getTraits())
			{
				_width = gc->getTraits()->width;
				_height = gc->getTraits()->height;
			}

			std::cout << "Window size " << _width << ", " << _height << std::endl;

			// single buffered image
			image = new osg::Image;
		}

		void read()
		{
		
			image->readPixels(0, 0, _width, _height,
				_pixelFormat, _type);

			cv::Mat illo(image->t(), image->s(), CV_8UC4);
			illo.data = (uchar*)image->data();
			cv::flip(illo, illo, 0);

			mat_to_return = illo.clone();
			//mat_to_return.create(_width, _height, CV_8UC4);
		//	mat_to_return.data = (uchar*)image->data();
			//cv::flip(mat_to_return, mat_to_return, 0);

			has_read = 1;
		}

	};

	typedef std::map<osg::GraphicsContext*, osg::ref_ptr<ContextData> > ContextDataMap;

	GLenum                      _readBuffer;
	mutable OpenThreads::Mutex  _mutex;
	mutable ContextDataMap      _contextDataMap;

	WindowCaptureCallback(GLenum readBuffer) :
		_readBuffer(readBuffer)
	{
	}

	ContextData* createContextData(osg::GraphicsContext* gc) const
	{
		return new ContextData(gc, _readBuffer);
	}

	ContextData* getContextData(osg::GraphicsContext* gc) const
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		osg::ref_ptr<ContextData>& data = _contextDataMap[gc];
		if (!data) data = createContextData(gc);

		return data.get();
	}

	virtual void operator () (osg::RenderInfo& renderInfo) const
	{
		glReadBuffer(_readBuffer);

		osg::GraphicsContext* gc = renderInfo.getState()->getGraphicsContext();
		osg::ref_ptr<ContextData> cd = getContextData(gc);
		cd->read();
	}


};


cv::Mat get_pic(osg::ref_ptr<osg::Geode> _model, osg::Matrix &_trans)
{	

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);
	
	double width;		fs["sensor_width"] >> width;
	int height;			fs["sensor_height"] >> height;
	
	double f_x;			fs["sensor_f_x"] >> f_x;
	double f_y;			fs["sensor_f_y"] >> f_y;
	double x_0;			fs["sensor_x_0"] >> x_0;
	double y_0;			fs["sensor_y_0"] >> y_0;

	fs.release();
	
	int zNear = 3;
	int zFar = 1000;

	GLenum readBuffer = GL_BACK;

	osg::ref_ptr<osg::Group> model = _model;
	osgViewer::Viewer viewer;
	viewer.setSceneData(model.get());

	viewer.getCamera()->setProjectionMatrixAsPerspective(30.0f, width / height, 1.0f, 10000.0f);

	osg::Matrix view = _trans;
	viewer.getCamera()->setViewMatrix(view);
	viewer.getCamera()->setProjectionMatrixAsFrustum(-zNear*x_0 / f_x, zNear*(width - x_0) / f_x,
		-zNear*y_0 / f_y, zNear*(height - y_0) / f_y, zNear, zFar);
	
	osg::ref_ptr<osg::GraphicsContext> pbuffer;

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = width;
	traits->height = height;
	traits->red = 8;
	traits->green = 8;
	traits->blue = 8;
	traits->alpha = 8;
	traits->windowDecoration = false;
	traits->pbuffer = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;

	pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
	/*if (pbuffer.valid())
	{
	osg::notify(osg::NOTICE) << "Pixel buffer has been created successfully." << std::endl;
	}
	else
	{
	osg::notify(osg::NOTICE) << "Pixel buffer has not been created successfully." << std::endl;
	}*/

	if (pbuffer.valid())
	{
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(pbuffer.get());
		camera->setViewport(new osg::Viewport(0, 0, width, height));
		camera->setFinalDrawCallback(new WindowCaptureCallback(readBuffer));

		viewer.addSlave(camera.get());

		viewer.realize();

		viewer.stopThreading();
		pbuffer->releaseContext();
		viewer.startThreading();
	}

	viewer.frame();

	while (!has_read) 
	{}

	return mat_to_return;

}