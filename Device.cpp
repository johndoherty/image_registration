/*
 * Device.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Device.h"

using namespace std;
using namespace cv;

Device::Device(ImageInput &image, FeatureDetector &detector) {
	imageInput = &image;
	featureDetector = &detector;
	imageInput->getNextImageFrame(currentImage);
	//cameraMatrix = Mat::eye(3,4, CV_64F); //TODO: init camera
	cameraMatrix = (Mat_<float>(3, 3) << 658.46, 0, 399.5, 0, 658.46, 239.5, 0, 0, 1);
	//[658.4605250182128, 0, 399.5;
	//0, 658.4605250182128, 239.5;
	//0, 0, 1]

}

bool Device::advanceFrame() {
	Mat nextFrame;
	bool valid = imageInput->getNextImageFrame(nextFrame);
	if (valid) {
		nextFrame.copyTo(currentImage);
		cvtColor(currentImage, currentBwImage, CV_RGB2GRAY);
	}
	return valid;
}

void Device::getCurrentImage(Mat &image) {
	currentImage.copyTo(image);
}

void Device::getCurrentBwImage(Mat &bwImage) {
	currentBwImage.copyTo(bwImage);
}

void Device::getCurrentKeyPoints(vector<KeyPoint> &keyPoints) {
	featureDetector->detect(currentBwImage, keyPoints);
}

void Device::getCameraMatrix(Mat &camera) {
	cameraMatrix.copyTo(camera);
}
