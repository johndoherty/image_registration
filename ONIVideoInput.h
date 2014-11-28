#ifndef ONI_VIDEO_INPUT_H_
#define ONI_VIDEO_INPUT_H_

#include <iostream> // for standard I/O
#include <string>   // for strings
#include "ImageInput.h"
#include "DepthInput.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <XnCppWrapper.h>

class ONIVideoInput: public ImageInput, public DepthInput {
public:
	ONIVideoInput(std::string filename, int startFrame = 0);
	~ONIVideoInput();

	int getCurrentFrameCount();
	void getFirstImageFrame(cv::Mat &firstFrame);
	void getFirstDepthFrame(cv::Mat &firstFrame);
	bool getNextImageFrame(cv::Mat &frame);
	bool getNextDepthFrame(cv::Mat &frame);
	bool getNextUserHeadLocation(cv::Point3f &headLocation);
	void depthImageCoordsToWorldCoords(cv::Mat &depthImage, std::vector<cv::Point2f> imageCoords, std::vector<cv::Point3f> &worldCoords);

	//XnChar strPose[20];
	//bool needPose;
	void newUser(XnUserID nId);
	void lostUser(XnUserID nId);
	void calibrationComplete(XnUserID nId, XnCalibrationStatus eStatus);
	void calibrationStart(XnUserID nId);
	void poseDetected(const XnChar* strPose, XnUserID nId);

private:
	cv::Mat firstImageFrame;
	cv::Mat firstDepthFrame;

	int currentFrameCount;
	int totalFrameCount;

	xn::Context context;
	xn::Player player;
	xn::ImageGenerator imageGen;
	xn::DepthGenerator depthGen;
	xn::UserGenerator userGen;
	bool supportsUserTracking;
	bool needPose;
	XnChar strPose[20];
	XnUInt32 frame_height;
	XnUInt32 frame_width;

};

#endif /* ONI_VIDEO_INPUT_H_ */

