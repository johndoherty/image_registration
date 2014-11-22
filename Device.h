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
private:
	ImageInput* inputVideo;
	cv::FeatureDetector* featureDetector;
	cv::Mat currentImage;
	cv::Mat currentBwImage;
};



#endif /* DEVICE_H_ */
