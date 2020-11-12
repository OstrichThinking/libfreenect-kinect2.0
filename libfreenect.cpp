#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include"mykinect.h"

using namespace std;
using namespace libfreenect2;

int main() {

    MyKinect* myKinect = new MyKinect();

    if (myKinect->getDeviceNumber() == 0) {
        cout << "no device connected!" << endl;
        return -1;
    }
    string serial = myKinect->getDeviceSerialNumber();
    //open devicei
    if (myKinect->open(serial) == 0) {
        cout << "failure opening device!" << endl;
        return -1;
    }
    myKinect->setDepthRange(0.5, 1.2);
    myKinect->setListener();
    myKinect->startDevice();
    myKinect->registerListener();
    myKinect->geyDeviceIntrinsic();

    FrameMap frames;
    //cv::VideoWriter rgbwriter("./video/rgb.mp4", cv::CAP_IMAGES, 30.0, cv::Size(512, 424),true);

    int number = 0;
    //pcl::visualization::CloudViewer viewerRGBXYZ("ViewerRGBXYZ");
    //pcl::visualization::CloudViewer viewerXYZ("ViewerXYZ");

    while (true) {

        frames = myKinect->getMultiFrames();
        myKinect->setFrames(frames);

        cv::Mat depth_color, rgb, ir;
        // mapping deepth to color  COLORMAP_JET
        applyColorMap(myKinect->getDEPTHMat(), depth_color, cv::COLORMAP_JET);
        rgb = myKinect->getConterFromRGB(myKinect->getRGBMat());
        ir = myKinect->getIRMat();
        
        ostringstream osrgb, osir, osdepth, osPCLXYZRGB, osPCLXYZ;
        osrgb << "./image/rgb/rgb_" << number << ".jpg";
        osir << "./image/ir/ir_" << number << ".png";
        osdepth << "./image/depth/depth_" << number << ".png";
        osPCLXYZRGB << "./PCL/pclxyzrgb_" << number << ".pcd";
        osPCLXYZ << "./PCL/pclxyz_" << number << ".pcd";


        Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

        myKinect->getRegistration()->apply(myKinect->getOriginRGBFrame(),
            myKinect->getOriginDEPTHFrame(), &undistorted, &registered, true, &depth2rgb);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB = myKinect->getXYZPCLRGB(frames[Frame::Depth], &registered);
        //viewerRGBXYZ.showCloud(cloudXYZRGB);
        //viewerXYZ.showCloud(myKinect->getXYZPCL(frames[Frame::Depth]));

        // save data
        if (number < 600) {

            imwrite(osrgb.str(), rgb);
            imwrite(osir.str(), ir);
            imwrite(osdepth.str(), depth_color);

            cloudXYZRGB->height = 1;
            cloudXYZRGB->width = cloudXYZRGB->points.size();
            cout << "point cloud size = " << cloudXYZRGB->points.size() << endl;
            cloudXYZRGB->is_dense = false;
            pcl::io::savePCDFile(osPCLXYZRGB.str(), *cloudXYZRGB);
            number++;
            cout << number << endl;
        }
        else{
            break;
        }

        cv::imshow("rgb-center", rgb);
        //cv::imshow("ir", ir);
        //cv::imshow("depth", depth_color);

        if (cv::waitKey(1) == 0x1B) {
            break;
        }
        myKinect->releaseFrame(frames);
    }
    cv::destroyAllWindows();
    myKinect->releaseDevice();
    //rgbwriter.release();
    return 0;
}
