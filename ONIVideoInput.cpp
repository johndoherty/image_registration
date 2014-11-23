/*
 * ONIVideoInput.cpp
 *
 *  Created on: Nov 19, 2014
 *      Author: john
 */
#include "ONIVideoInput.h"

using namespace std;
using namespace cv;

#define THROW_IF_FAILED(retVal) {if (retVal != XN_STATUS_OK) {cout << xnGetStatusString(retVal); throw xnGetStatusString(retVal);}}


ONIVideoInput::ONIVideoInput(std::string filename, int startFrame) {
	THROW_IF_FAILED(context.Init());

	THROW_IF_FAILED(context.OpenFileRecording(filename.c_str(), player));
	THROW_IF_FAILED(player.SetRepeat(false));

	THROW_IF_FAILED(imageGen.Create(context));
	XnPixelFormat pixelFormat = imageGen.GetPixelFormat();
	if (pixelFormat != XN_PIXEL_FORMAT_RGB24) {
		THROW_IF_FAILED(imageGen.SetPixelFormat(XN_PIXEL_FORMAT_RGB24));
	}
	xn::ImageMetaData xImageMap2;
	imageGen.GetMetaData(xImageMap2);
	frame_height = xImageMap2.YRes();
	frame_width = xImageMap2.XRes();

	depthGen.Create(context);
	XnUInt32 nframes;
	player.GetNumFrames(depthGen.GetName(), nframes);
	totalFrameCount = nframes;

	xn::NodeInfo nin = depthGen.GetInfo();
	if ((XnNodeInfo*)nin == 0)
		throw "Could not read DepthGenerator info. Probably, the input file is corrupted";

	XnProductionNodeDescription description = nin.GetDescription();
	XnFieldOfView fov;
	depthGen.GetFieldOfView(fov);
	cout << "Field of view: " << fov.fHFOV << ", "<< fov.fVFOV << endl;

	THROW_IF_FAILED(context.StartGeneratingAll());

	currentFrameCount = 0;

	firstImageFrame = Mat();
	firstDepthFrame = Mat();
	getNextImageFrame(firstImageFrame);
	getNextDepthFrame(firstDepthFrame);
	Mat test = Mat();
	for (int i = 0; i < startFrame; i++) {
		getNextImageFrame(test);
		getNextDepthFrame(test);
	}
}

ONIVideoInput::~ONIVideoInput() {
	context.StopGeneratingAll();
	context.Release();
}

void ONIVideoInput::getFirstImageFrame(Mat &output) {
	firstImageFrame.copyTo(output);
}

void ONIVideoInput::getFirstDepthFrame(Mat &output) {
	firstDepthFrame.copyTo(output);
}

bool ONIVideoInput::getNextImageFrame(Mat &output) {
	if (currentFrameCount >= totalFrameCount) {
		return false;
	}
	THROW_IF_FAILED(imageGen.WaitAndUpdateData());
	xn::ImageMetaData xImageMap;
	imageGen.GetMetaData(xImageMap);
	XnRGB24Pixel* imgData = const_cast<XnRGB24Pixel*>(xImageMap.RGB24Data());
	cv::Mat image(frame_height, frame_width, CV_8UC3, reinterpret_cast<void*>(imgData));
	cv::cvtColor(image, image, CV_BGR2RGB); // opencv image format is BGR

	if (!image.empty()) {
		image.copyTo(output);
		currentFrameCount++;
		return true;
	}
	return false;
}

bool ONIVideoInput::getNextDepthFrame(Mat &output) {
	// save depth
	THROW_IF_FAILED(depthGen.WaitAndUpdateData());
	xn::DepthMetaData xDepthMap;
	depthGen.GetMetaData(xDepthMap);
	XnDepthPixel* depthData = const_cast<XnDepthPixel*>(xDepthMap.Data());
	cv::Mat depth(frame_height, frame_width, CV_16U, reinterpret_cast<void*>(depthData));
	if (!depth.empty()) {
		depth.copyTo(output);
		return true;
	}
	return false;
}

int ONIVideoInput::getCurrentFrameCount() {
	return currentFrameCount;
}

Point2f ONIVideoInput::getFocalLength() const{
	// TODO: finish this
	return Point2f(0, 0);
}

Point2f ONIVideoInput::getCenterOfProjection() const{
	// TODO
	return Point2f(0, 0);
}


