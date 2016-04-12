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
#include <opencv2/highgui/highgui.hpp>

cv::Mat img_to_return;
int cond = 0;

	
class WindowCaptureCallback : public osg::Camera::DrawCallback
{
public:

	enum Mode
	{
		READ_PIXELS
	};

	enum FramePosition
	{
		END_FRAME
	};

	struct ContextData : public osg::Referenced
	{

		ContextData(osg::GraphicsContext* gc, Mode mode, GLenum readBuffer, const std::string& name) :
			_gc(gc),
			_mode(mode),
			_readBuffer(readBuffer),
			_fileName(name),
			_pixelFormat(GL_BGRA),
			_type(GL_UNSIGNED_BYTE),
			_width(0),
			_height(0),
			_currentImageIndex(0)
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

			getSize(gc, _width, _height);

			std::cout << "Window size " << _width << ", " << _height << std::endl;

			// single buffered image
			_imageBuffer.push_back(new osg::Image);
		}

		void getSize(osg::GraphicsContext* gc, int& width, int& height)
		{
			if (gc->getTraits())
			{
				width = gc->getTraits()->width;
				height = gc->getTraits()->height;
			}
		}

		void read()
		{
			osg::GLExtensions* ext = osg::GLExtensions::Get(_gc->getState()->getContextID(), true);
			readPixels();

		}

		void readPixels()
		{
			// std::cout<<"readPixels("<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;

			unsigned int nextImageIndex = (_currentImageIndex + 1) % _imageBuffer.size();

			int width = 0, height = 0;
			getSize(_gc, width, height);
			if (width != _width || _height != height)
			{
				std::cout << "   Window resized " << width << ", " << height << std::endl;
				_width = width;
				_height = height;
			}

			// osg::Image* imagen = _imageBuffer[_currentImageIndex].get();

			osg::Image * to_return = _imageBuffer[_currentImageIndex].get();

#if 1
			to_return->readPixels(0, 0, _width, _height,
				_pixelFormat, _type);
#endif

			//if (!_fileName.empty())
			//{
			//	// osgDB::writeImageFile(*to_return, _fileName); 
			//	osgDB::writeImageFile(*to_return, "lollo.bmp");
			//}

			cv::Mat illo(to_return->t(), to_return->s(), CV_8UC4);
			illo.data = (uchar*)to_return->data();
			cv::flip(illo, illo, 0);

			img_to_return = illo.clone();
			cond = 1;
			//_currentImageIndex = nextImageIndex;
		}




		typedef std::vector< osg::ref_ptr<osg::Image> > ImageBuffer;

		osg::GraphicsContext*   _gc;
		Mode                    _mode;
		GLenum                  _readBuffer;
		std::string             _fileName;

		GLenum                  _pixelFormat;
		GLenum                  _type;
		int                     _width;
		int                     _height;

		unsigned int            _currentImageIndex;
		ImageBuffer             _imageBuffer;

	};

	WindowCaptureCallback(Mode mode, FramePosition position, GLenum readBuffer) :
		_mode(mode),
		_position(position),
		_readBuffer(readBuffer)
	{
	}

	FramePosition getFramePosition() const { return _position; }

	ContextData* createContextData(osg::GraphicsContext* gc) const
	{
		std::stringstream filename;
		filename << "test_" << _contextDataMap.size() << ".bmp";
		return new ContextData(gc, _mode, _readBuffer, filename.str());
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

	typedef std::map<osg::GraphicsContext*, osg::ref_ptr<ContextData> > ContextDataMap;

	Mode                        _mode;
	FramePosition               _position;
	GLenum                      _readBuffer;
	mutable OpenThreads::Mutex  _mutex;
	mutable ContextDataMap      _contextDataMap;


};

void addCallbackToViewer(osgViewer::ViewerBase& viewer, WindowCaptureCallback* callback)
{
	osgViewer::ViewerBase::Windows windows;
	viewer.getWindows(windows);
	for (osgViewer::ViewerBase::Windows::iterator itr = windows.begin();
	itr != windows.end();
		++itr)
	{
		osgViewer::GraphicsWindow* window = *itr;
		osg::GraphicsContext::Cameras& cameras = window->getCameras();
		osg::Camera* lastCamera = 0;
		for (osg::GraphicsContext::Cameras::iterator cam_itr = cameras.begin();
		cam_itr != cameras.end();
			++cam_itr)
		{
			if (lastCamera)
			{
				if ((*cam_itr)->getRenderOrder() > lastCamera->getRenderOrder())
				{
					lastCamera = (*cam_itr);
				}
				if ((*cam_itr)->getRenderOrder() == lastCamera->getRenderOrder() &&
					(*cam_itr)->getRenderOrderNum() >= lastCamera->getRenderOrderNum())
				{
					lastCamera = (*cam_itr);
				}
			}
			else
			{
				lastCamera = *cam_itr;
			}
		}

		if (lastCamera)
		{
			osg::notify(osg::NOTICE) << "Last camera " << lastCamera << std::endl;

			lastCamera->setFinalDrawCallback(callback);
		}
		else
		{
			osg::notify(osg::NOTICE) << "No camera found" << std::endl;
		}
	}
}

cv::Mat get_pic(osg::ref_ptr<osg::Node> &_model, 
		osg::Matrix &_trans, 
		double _width, double _height, 
		double f_x, double f_y,
		double x_0, double y_0 )
{	
	int zNear = 3;
	int zFar = 1000;

	GLenum readBuffer = GL_BACK;
	WindowCaptureCallback::FramePosition position = WindowCaptureCallback::END_FRAME;
	WindowCaptureCallback::Mode mode = WindowCaptureCallback::READ_PIXELS;

	osg::ref_ptr<osg::Node> model = _model;
	osgViewer::Viewer viewer;
	viewer.setSceneData(model.get());

	double width = _width;
	int height = _height;
	viewer.getCamera()->setProjectionMatrixAsPerspective(30.0f, width / height, 1.0f, 10000.0f);

	osg::Matrix trans = _trans;
	trans.makeTranslate(0., 0., -500);
	viewer.getCamera()->setViewMatrix(trans);
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
		// GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
		// camera->setDrawBuffer(buffer);
		//camera->setReadBuffer(buffer);
		camera->setFinalDrawCallback(new WindowCaptureCallback(mode, position, readBuffer));

		viewer.addSlave(camera.get());

		viewer.realize();

		viewer.stopThreading();
		pbuffer->releaseContext();
		viewer.startThreading();
	}

	viewer.frame();

	std::string s = ".";
	while (cond == 0) 
	{
		s + ".";
	}
	cv::Mat img = img_to_return.clone();

	return img;

}



int main(int argc, char** argv)
{
	int width = 2024;
	int height = 1088;

	double f_x = 4615.04;
	double f_y = 4615.51;
	double x_0 = 1113.41;
	double y_0 = 480.016;
	
	osg::Matrix trans;
	trans.makeTranslate(0., 0., -500);
	
	// load the data
	osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("../data/cessna.osg");
	if (!loadedModel)
	{
		std::cout << "No data loaded" << std::endl;
		return 1;
	}
	else std::cout << "loaded cessna" << std::endl;

	//osg::Image* image = get_pic(loadedModel, trans, (double)width, (double)height );
	cv::Mat pippo = get_pic(loadedModel, trans, (double)width, (double)height, f_x, f_y, x_0, y_0);

	cv::imwrite("../data/Mat_debug.bmp", pippo);

	return 0; 

}
