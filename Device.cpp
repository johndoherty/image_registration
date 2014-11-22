/*
 * Device.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Device.h"

using namespace std;
using namespace cv;

Device::Device(ImageInput &imageInput, FeatureDetector &detector) {
	inputVideo = imageInput;
	featureDetector = detector;
	inputVideo->getNextImageFrame(currentImage);
}

bool Device::advanceFrame() {
	Mat nextFrame;
	bool valid = inputVideo->getNextImageFrame(nextFrame);
	if (valid) {
		nextFrame.copyTo(currentImage);
		cvtColor(currentImage, currentBwImage, CV_RGB2GRAY);
	}
	return valid;
}

void Device::getCurrentImage(Mat &image) {
	currentImage.copyTo(image);
}

void Device::getCurrentKeyPoints(vector<KeyPoint> &keyPoints) {
	featureDetector->detect(currentImage, keyPoints);
}
