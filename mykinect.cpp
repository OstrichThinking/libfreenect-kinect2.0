#include "mykinect.h"

MyKinect::MyKinect() {}
MyKinect::~MyKinect() {}

/// <summary>
/// Must be called before doing anything else.
/// </summary>
/// <returns>
/// Number of devices, 0 if none
/// </returns>
int MyKinect::getDeviceNumber() {
	return freenect2.enumerateDevices();
}

/// <summary>
/// Open device by serial number with default pipeline.
/// </summary>
/// <param name="serial">number of device</param>
/// <returns>
/// New device object, or NULL on failure
/// </returns>
Freenect2Device* MyKinect::open(string serial) {
	dev = freenect2.openDevice(serial);
	return dev;
}

/// <summary>
/// get device serial number
/// </summary>
/// <returns>
/// serial number or enpty if the device is invalid
/// </returns>
string MyKinect::getDeviceSerialNumber() {
	return freenect2.getDefaultDeviceSerialNumber();
}


/// <summary>
/// set the depth range of device
/// </summary>
/// <param name="minRange">the min range(meter), the min range of kinect2.0 is 0.5 meter</param>
/// <param name="maxRange">the max range(meter), the max range of kinect is 8 meters</param>
void MyKinect::setDepthRange(float minRange, float maxRange) {
	Freenect2Device::Config config;
	config.MinDepth = 0.5;
	config.MaxDepth = 1.2;
	dev->setConfiguration(config);
}

/// <summary>
/// set listener to get frame
/// </summary>
void MyKinect::setListener() {
	listener = new SyncMultiFrameListener(Frame::Color | Frame::Depth | Frame::Ir);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
}

/// <summary>
/// registe listener
/// </summary>
void MyKinect::registerListener() {
	this->registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

/// <summary>
/// Start data processing with both RGB and depth streams, once device started all config is invalid
/// </summary>
void MyKinect::startDevice() {
	dev->start();
}

/// <summary>
/// to get multiple frames
/// </summary>
/// <returns>
/// return multiple frames
/// </returns>
libfreenect2::FrameMap MyKinect::getMultiFrames() {
	FrameMap frames;
	listener->waitForNewFrame(frames);
	return frames;
}

/// <summary>
/// release the frames
/// </summary>
/// <param name="frames">frames which will be released</param>
void MyKinect::releaseFrame(FrameMap frames) {
	listener->release(frames);
}

/// <summary>
/// to release the device
/// </summary>
void MyKinect::releaseDevice() {
	dev->stop();
	dev->close();
}


void MyKinect::geyDeviceIntrinsic() {
	
	Freenect2Device::ColorCameraParams colorParams = dev->getColorCameraParams();
	Freenect2Device::IrCameraParams IrParams = dev->getIrCameraParams();

	Json::Value root;
	Json::Value ColorIntrinsic;
	Json::Value DepthIntrinsic;
	Json::Value Distortion;
	// get Color Camera 
	ColorIntrinsic["fx"] = Json::Value(colorParams.fx);//Focal length x (pixel)
	ColorIntrinsic["fy"] = Json::Value(colorParams.fy);//Focal length y (pixel)
	ColorIntrinsic["cx"] = Json::Value(colorParams.cx);//Principal point x (pixel)
	ColorIntrinsic["cy"] = Json::Value(colorParams.cy);//Principal point y (pixel)
	root["ColorIntrinsic"] = Json::Value(ColorIntrinsic);

	DepthIntrinsic["fx"] = Json::Value(IrParams.fx);//Focal length x (pixel)
	DepthIntrinsic["fy"] = Json::Value(IrParams.fy);//Focal length y (pixel)
	DepthIntrinsic["cx"] = Json::Value(IrParams.cx);//Principal point x (pixel)
	DepthIntrinsic["cy"] = Json::Value(IrParams.cy);//Principal point y (pixel)
	
	root["DepthIntrinsic"] = Json::Value(DepthIntrinsic);

	//cout << "************color  "<< colorParams.fx <<"     IR "<< IrParams.fx <<"     **********" << endl;

	Distortion["k1"] = Json::Value(IrParams.k1);//Radial distortion coefficient, 1st-order
	Distortion["k2"] = Json::Value(IrParams.k2);//Radial distortion coefficient, 2st-order
	Distortion["k3"] = Json::Value(IrParams.k3);//Radial distortion coefficient, 3st-order
	Distortion["p1"] = Json::Value(IrParams.p1);//Tangential distortion coefficient
	Distortion["p2"] = Json::Value(IrParams.p2);//Tangential distortion coefficient

	//cout << ColorIntrinsic << "*******" << DepthIntrinsic << endl;

	root["Distortion"] = Json::Value(Distortion);

	Json::StyledWriter sw;
	cout << sw.write(root) << endl << endl;

	//write to json
	ofstream os;
	os.open("./intrinsic/intrinsic.json");
	os << sw.write(root);
	os.close();
	
}

Registration* MyKinect::getRegistration() {
	return registration;
}

/// <summary>
/// setter
/// </summary>
void MyKinect::setFrames(FrameMap frameMap) {
	frames = frameMap;
}

/// <summary>
/// get origin RGB frame
/// </summary>
/// <returns>
/// 1920x1080. BGRX or RGBX 
/// </returns>
Frame* MyKinect::getOriginRGBFrame() {
	originRGBFrame = frames[Frame::Color];
	return originRGBFrame;
}

/// <summary>
/// get origin IR frames
/// </summary>
/// <returns>
/// 512x424 float. Range is [0.0, 65535.0].
/// </returns>
Frame* MyKinect::getOriginIRFrame() {
	originIRFrame = frames[Frame::Ir];
	return originIRFrame;
}

/// <summary>
/// get origin depth frame
/// </summary>
/// <returns>
/// 512x424 float, unit: millimeter. Non-positive, NaN, and infinity are invalid or missing data.
/// </returns>
Frame* MyKinect::getOriginDEPTHFrame() {
	originDepthFrame = frames[Frame::Depth];
	return originDepthFrame;
}

/// <summary>
/// get the processed mat
/// </summary>
/// <returns>
/// return the Processed Fram from origin
/// </returns>
cv::Mat MyKinect::getRGBMat() {
	cv::Mat rgbmat;
	Frame* rgb = frames[Frame::Color];
	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
	return rgbmat;
}

cv::Mat MyKinect::getConterFromRGB(cv::Mat rgb) {
	cv::Mat RGBMat = rgb;
	cv::Mat depthFromRgb;
	cv::Size size(512, 424);
	float matrix[2][3] = { 
		{3.55251142e-1, -2.92237443e-2,-1.23270320e+2},
		{-1.03500761e-2, 3.49528158e-1, 3.31646880e+1} };
	cv::Mat affineMatrix = cv::Mat(2,3, CV_32F, matrix);
	warpAffine(RGBMat, depthFromRgb, affineMatrix, size, cv::INTER_AREA, cv::BORDER_CONSTANT, 0);
	return depthFromRgb;
}


/// <summary>
/// get the processed mat
/// </summary>
/// <returns>
/// return the processed frame which CV_32FC1 converted to CV_16UC1
/// </returns>
cv::Mat MyKinect::getIRMat() {
	cv::Mat irmat, ir_converted;
	Frame* ir = frames[Frame::Ir];
	cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
	normalize(irmat, ir_converted, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
	return ir_converted;
}

/// <summary>
/// get the processed mat
/// </summary>
/// <returns>
/// return the processed frame which CV_32FC1 convered to CV_16UC1 and convered to CV_8UC1
/// </returns>
cv::Mat MyKinect::getDEPTHMat() {

	cv::Mat depthmat, depth_converted;
	Frame* depth = frames[Frame::Depth];
	cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
	normalize(depthmat, depth_converted, 0, 65535, cv::NORM_MINMAX, CV_16UC1);

	// 16 to 8
	int width = depth_converted.cols;
	int height = depth_converted.rows;
	cv::Mat dst_8 = cv::Mat::zeros(height, width, CV_8UC1);//create a enpty mat
	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;
	minMaxIdx(depth_converted, minp, maxp);  //get the max and min of pixel

	//cout << "*********** min of pixel:  " << minv << "  max of piexl:  " << maxv << "**********" << endl;

	ushort* p_img;
	uchar* p_dst;
	for (int i = 0; i < height; i++) {

		p_img = depth_converted.ptr<ushort>(i);//get i row head pointer in source_img
		p_dst = dst_8.ptr<uchar>(i);//get i row head pointer in des_img

		for (int j = 0; j < width; ++j){
			p_dst[j] = (p_img[j] - minv) / (maxv - minv) * 255;
		}
	}

	return dst_8;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr MyKinect::getXYZPCL(Frame* undistored) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);

	float xx = 0.0, yy = 0.0, zz = 0.0;
	for (int m = 0; m < 512; m++) {
		for (int n = 0; n < 424; n++) {
			pcl::PointXYZ p;
			registration->getPointXYZ(undistored, n, m, xx, yy, zz);
			if (zz < 1.2 && yy < 0.2) {
				p.z = -zz;
				p.x = xx;
				p.y = -yy;
			}
			cloudXYZ->points.push_back(p);
		}
	}
	return cloudXYZ;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MyKinect::getXYZPCLRGB(Frame* undistorted, Frame* registered) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	float x = 0.0, y = 0.0, z = 0.0, color = 0.0;

	for (int m = 0; m < 512; m++){
		for (int n = 0; n < 424; n++){
			pcl::PointXYZRGB p;
			registration->getPointXYZRGB(undistorted, registered, n, m, x, y, z, color);
			const uint8_t* c = reinterpret_cast<uint8_t*>(&color);
			uint8_t b = c[0];
			uint8_t g = c[1];
			uint8_t r = c[2];
			if (z < 1.2 && y < 0.2)
			{
				p.z = -z;
				p.x = x;
				p.y = -y;
				p.b = b;
				p.g = g;
				p.r = r;
			}
			cloudXYZRGB->points.push_back(p);
		}
	}

	return cloudXYZRGB;
}