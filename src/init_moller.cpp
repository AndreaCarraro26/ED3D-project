#include "laser_scanner.h"
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS
#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>   // Khronos C++ Wrapper API
#include <pcl/io/pcd_io.h>


void init_moller(Configuration& confData, OpenCLparam& CLparam) {
	// Calculation of the direction vector used for moller-trumbore
	int numDirections;
	if (confData.numLaser % 2 == 0)
		numDirections = confData.numLaser + 1;
	else
		numDirections = confData.numLaser;

	float * directions = (float *)malloc(numDirections * 3 * 2 * sizeof(float));

	// Point of application of first laser plane
	float laserAx = confData.cameraPos[0];
	float laserAy = confData.cameraPos[1] + confData.baseline;
	float laserAz = confData.cameraPos[2];
	float sourceA[3] = { laserAx, laserAy, laserAz };

	// Arriving point of the firt laser plane
	float centerX = laserAx;

	float centerY = laserAy - confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser));
	float centerZ = laserAz - confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser));
	float centerA[3] = { centerX, centerY, centerZ };

	// minumum angle between two discrete laser 
	float minAngle = confData.fanLaser / confData.numLaser;
	// First angle in the for loop
	float actualAngle = -confData.fanLaser / 2;

	// Buinlding vector directionsA
	for (int i = 0; i < numDirections * 3; i = i + 3) {
		float x;
		if (deg2rad*actualAngle < 0)
			x = centerA[0] - confData.laserLength*tan(fabs((deg2rad*actualAngle)));
		else
			x = centerA[0] + confData.laserLength*tan(fabs((deg2rad*actualAngle)));
		float y = centerA[1];
		float z = centerA[2];
		cv::Vec3f direction_temp = { x - sourceA[0] , y - sourceA[1] , z - sourceA[2] };
		direction_temp = direction_temp / norm(direction_temp);
		directions[i] = direction_temp[0];
		directions[i + 1] = direction_temp[1];
		directions[i + 2] = direction_temp[2];
		actualAngle += minAngle;
	}

	// Same algorithm used for the second plane

	float laserBx = confData.cameraPos[0];
	float laserBy = confData.cameraPos[1] - confData.baseline;
	float laserBz = confData.cameraPos[2];
	float sourceB[3] = { laserBx, laserBy, laserBz };					
	float center2X = laserBx;
	float center2Y = laserBy + confData.laserLength*sin(deg2rad*(90 - confData.alphaLaser));
	float center2Z = laserBz - confData.laserLength*cos(deg2rad*(90 - confData.alphaLaser));
	float centerB[3] = { center2X, center2Y, center2Z };

	// Buinlding vector directionsA
	actualAngle = -confData.fanLaser / 2;
	for (int i = 0; i < numDirections * 3; i = i + 3) {
		float x;
		if (deg2rad*actualAngle < 0)
			x = centerB[0] - confData.laserLength*tan(fabs((deg2rad*actualAngle)));
		else
			x = centerB[0] + confData.laserLength*tan(fabs((deg2rad*actualAngle)));
		float y = centerB[1];
		float z = centerB[2];
		cv::Vec3f direction_temp = { x - sourceB[0] , y - sourceB[1] , z - sourceB[2] };
		direction_temp = direction_temp / norm(direction_temp);
		directions[i + numDirections * 3] = direction_temp[0];
		directions[i + 1 + numDirections * 3] = direction_temp[1];
		directions[i + 2 + numDirections * 3] = direction_temp[2];
		actualAngle += minAngle;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// This portion of code is used only if moller-trumbore method is chosen on config file

	
	int numPolygon;

	if (confData.useMoller) {
		

		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFileSTL(confData.modelName, mesh) == 0)	{
			std::cout << "No model found with this name." << std::endl;
			_waitKey();
			return 1;
		}
		
		// mesh to cloud convertion		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVertices(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(mesh.cloud, *cloudVertices);

		// v1 represents all the first verteces of every face of the mesh. 
		// x, y, z are stored in at position [j], [j+1], [j+2]
		numPolygon = mesh.polygons.size();
		float * v1 = (float *)malloc(numPolygon * 3 * sizeof(float));
		float * v2 = (float *)malloc(numPolygon * 3 * sizeof(float));
		float * v3 = (float *)malloc(numPolygon * 3 * sizeof(float));

		pcl::Vertices triangle;
		int j = 0;
		int i = 0;
		for (i = 0; i < numPolygon; i++) {
			pcl::PointXYZ vertex1, vertex2, vertex3;
			triangle = mesh.polygons.at(i);
			vertex1 = cloudVertices->points[triangle.vertices[0]];
			vertex2 = cloudVertices->points[triangle.vertices[1]];
			vertex3 = cloudVertices->points[triangle.vertices[2]];

			v1[j] = vertex1.x;
			v1[j + 1] = vertex1.y;
			v1[j + 2] = vertex1.z;

			v2[j] = vertex2.x;
			v2[j + 1] = vertex2.y;
			v2[j + 2] = vertex2.z;

			v3[j] = vertex3.x;
			v3[j + 1] = vertex3.y;
			v3[j + 2] = vertex3.z;

			j = j + 3;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////

		// calculating mesh position to set the camera/laser on the right position
		float pclMinX = FLT_MAX, pclMaxX = -FLT_MAX, pclMinY = FLT_MAX, pclMaxY = -FLT_MAX, pclMinZ = FLT_MAX, pclMaxZ = -FLT_MAX;
			for (int i = 0; i < cloudVertices->points.size(); i++ ) {
			pcl::PointXYZ point = cloudVertices->points[i];
			if (point.x < pclMinX)
				pclMinX = point.x;
			if (point.x > pclMaxX)
				pclMaxX = point.x;

			if (point.y < pclMinY)
				pclMinY = point.y;
			if (point.y > pclMaxY)
				pclMaxY = point.y;

			if (point.z < pclMinZ)
				pclMinZ = point.z;
			if (point.z > pclMaxZ)
				pclMaxZ = point.z;
		}
		confData.cameraPos[0] = (pclMaxX + pclMinX) / 2;
		confData.cameraPos[2] = pclMinZ + confData.cameraHeight;


		////////////////////////////////////////////////////////////////////////////////////////////////////

		// Initializing OpenCL

		// Get all platforms (drivers)
		std::vector<cl::Platform> all_platforms;
		cl::Platform::get(&all_platforms);
		if (all_platforms.size() == 0) {
			std::cout << " No platforms found. Check OpenCL installation!\n";
			exit(1);
		}
		std::cout << "Choose the platform: " << std::endl;
		for (int i = 0; i < all_platforms.size(); i++) {
			std::cout << i << ": " << all_platforms[i].getInfo<CL_PLATFORM_NAME>() << std::endl;
		}
		char * st;	std::cin >> st;
		int num = atoi(st);
		cl::Platform default_platform = all_platforms[num];
		std::cout << "Using platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << "\n";

		std::vector<cl::Device> all_devices;
		default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
		if (all_devices.size() == 0) {
			std::cout << " No devices found. Check OpenCL installation!\n";
			exit(1);
		}
		std::cout << "Choose the device: " << std::endl;
		for (int i = 0; i < all_devices.size(); i++) {
			std::cout << i << ": " << all_devices[i].getInfo<CL_DEVICE_NAME>() << std::endl;
		}
		st;	std::cin >> st;
		num = atoi(st);
		cl::Device default_device = all_devices[num];
		std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>() << "\n";

		cl::Context context({ default_device });

		cl::CommandQueue queue(context, default_device);

		ifstream cl_file("./moller.cl");
		std::string cl_string(std::istreambuf_iterator<char>(cl_file), (std::istreambuf_iterator<char>()));
		cl::Program::Sources source(1, std::make_pair(cl_string.c_str(), cl_string.length() + 1));
		cl::Program  program(context, source);

		try {
			program.build({ default_device });
		}
		catch (cl::Error error) {
			if (error.err() == CL_BUILD_PROGRAM_FAILURE) {
				std::cout << "Build log:" << std::endl << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << std::endl;
			}
			throw error;
		}

		// Buffer allocation
		cl::Buffer d_v1, d_v2, d_v3, d_sourceCamera, d_directions, d_results, d_new_directions;
		d_v1 = cl::Buffer::Buffer(context, CL_MEM_READ_ONLY, sizeof(float) * numPolygon * 3);
		d_v2 = cl::Buffer::Buffer(context, CL_MEM_READ_ONLY, sizeof(float) * numPolygon * 3);
		d_v3 = cl::Buffer::Buffer(context, CL_MEM_READ_ONLY, sizeof(float) * numPolygon * 3);
		d_sourceCamera = cl::Buffer::Buffer(context, CL_MEM_READ_WRITE, sizeof(float) * 3);
		d_directions = cl::Buffer::Buffer(context, CL_MEM_READ_ONLY, sizeof(float) * numDirections * 3 * 2);
		d_results = cl::Buffer::Buffer(context, CL_MEM_READ_WRITE, sizeof(float) * numDirections * 3 * 2);
		d_new_directions = cl::Buffer::Buffer(context, CL_MEM_READ_WRITE, sizeof(float) * numDirections * 3 * 2);

		queue.enqueueWriteBuffer(d_v1, CL_TRUE, 0, sizeof(float) * numPolygon * 3, v1);
		queue.enqueueWriteBuffer(d_v2, CL_TRUE, 0, sizeof(float) * numPolygon * 3, v2);
		queue.enqueueWriteBuffer(d_v3, CL_TRUE, 0, sizeof(float) * numPolygon * 3, v3);
		queue.enqueueWriteBuffer(d_directions, CL_TRUE, 0, sizeof(float) * numDirections * 3 *2, directions);

		CLparam = {
			default_platform, default_device, context, queue, program, d_v1, d_v2, d_v3, d_sourceCamera, d_directions, d_results, d_new_directions
		};
	}
}