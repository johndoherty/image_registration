/*
 * External.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "External.h"

using namespace std;
using namespace cv;
using namespace pcl;


External::External(ImageInput &image, DepthInput &depth, cv::FeatureDetector &detector) {
	imageInput = image;
	depthInput = depth;
	featureDetector = detector;
	imageInput->getNextImageFrame(firstImage);
	depthInput->getNextDepthFrame(firstDepth);

	rangeImage = new RangeImagePlanar();
	pointCloud = new PointCloud();
	Point2f focalLength = depthInput->getFocalLength();
	Point2f centerOfProjection = depthInput->getCenterOfProjection();
	rangeImage->setDepthImage(
			firstDepth.ptr<float>(0, 0),
			firstDepth.cols,
			firstDepth.rows,
			centerOfProjection.x,
			centerOfProjection.y,
			focalLength.x,
			focalLength.y
	);

	getCurrentKeyPoints(firstKeyPoints);

	for (int row = 0; row < firstDepth.rows; row++) {
		for (int col = 0; col < firstDepth.cols; col++) {
			Eigen::Vector3f v;
			rangeImage->calculate3DPoint((float)col, (float)row, firstDepth.at<float>(row, col), v);
			PointXYZ point;
			point.x = v[0];
			point.y = v[1];
			point.z = v[2];
			pointCloud->push_back(point);

		}
	}

	for (int i = 0; i < firstKeyPoints.size(); i++) {
		Eigen::Vector3f v;
		rangeImage->calculate3DPoint((float)firstKeyPoints[i].pt.x, (float)firstKeyPoints[i].pt.y, firstDepth.at<float>(firstKeyPoints[i].pt), v);
		PointXYZ point;
		point.x = v[0];
		point.y = v[1];
		point.z = v[2];
		keyWorldPoints.push_back(point);
	}
}

bool External::advanceFrame() {
	Mat nextImage;
	Mat nextDepth;
	bool imageValid = imageInput->getNextImageFrame(nextImage);
	bool depthValid = depthInput->getNextDepthFrame(nextDepth);
	if (imageValid) {
		nextImage.copyTo(currentImage);
		cvtColor(currentImage, currentBwImage, CV_RGB2GRAY);
	}
	if (depthValid) {
		nextDepth.copyTo(currentDepthImage);
	}
	return imageValid && depthValid;
}

void External::getCurrentImage(Mat &image) {
	currentImage.copyTo(image);
}

void External::getCurrentKeyPoints(vector<KeyPoint> &keyPoints) {
	featureDetector->detect(currentBwImage, keyPoints);
}

void External::getCurrentDepthImage(cv::Mat &depthImage) {
	currentDepthImage.copyTo(depthImage);
}
