/*
 * Device.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include "CVVideoInput.h"

class Device {
public:
	Device(ImageInput &imageInput, cv::FeatureDetector &detector);
	bool advanceFrame();
	void getCurrentImage(cv::Mat &image);
	void getCurrentBwImage(cv::Mat &bwImage);
	void getCurrentKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
	void getCameraMatrix(cv::Mat &cameraMatrix); // TODO: use matx
private:
	ImageInput* imageInput;
	cv::FeatureDetector* featureDetector;
	cv::Mat currentImage;
	cv::Mat currentBwImage;
	cv::Mat cameraMatrix;
};



#endif /* DEVICE_H_ */
