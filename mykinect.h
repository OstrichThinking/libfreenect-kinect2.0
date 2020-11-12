#pragma once
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include < libfreenect2/libfreenect2.hpp>
#include < libfreenect2/frame_listener_impl.h>
#include < libfreenect2/registration.h>
#include < libfreenect2/packet_pipeline.h>
#include <fstream>
#include "json/json.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace libfreenect2;

class MyKinect
{
public:
	MyKinect();
	~MyKinect();

	//about device
	int getDeviceNumber();
	string getDeviceSerialNumber();
	Freenect2Device* open(string serial);
	void setDepthRange(float minRange, float maxRange);
	void setListener();
	void startDevice();
	void registerListener();
	libfreenect2::FrameMap getMultiFrames();
	void releaseFrame(FrameMap frames);
	void releaseDevice();
	void geyDeviceIntrinsic();
	Registration* getRegistration();

	//about frame
	void setFrames(FrameMap frames);
	Frame* getOriginRGBFrame();
	Frame* getOriginIRFrame();
	Frame* getOriginDEPTHFrame();
	cv::Mat getRGBMat();
	cv::Mat getIRMat();
	cv::Mat getDEPTHMat();
	cv::Mat getConterFromRGB(cv::Mat rgb);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZPCL(Frame* undistored);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getXYZPCLRGB(Frame* undistorted, Frame* registered);


private:
	Freenect2Device* dev;
	Freenect2 freenect2;
	SyncMultiFrameListener* listener;
	FrameMap frames;
	Frame* originRGBFrame;
	Frame* originIRFrame;
	Frame* originDepthFrame;
	Registration* registration;
};
