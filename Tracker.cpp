/*
 * Tracker.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Tracker.h"

using namespace std;
using namespace cv;

Tracker::Tracker(Device &d, External &e) {
	device = d;
	external = e;
}

bool Tracker::advanceFrame() {
	bool validDevice = device->advanceFrame();
	bool validExternal = external->advanceFrame();
	return validDevice && validExternal;
}

void Tracker::computePosePnP(cv::Mat &R, cv::Mat &t) {
	Mat externalBwImage, deviceBwImage, deviceCamera;
	vector<KeyPoint> deviceKeyPoints, externalKeyPoints;
	device->getCameraMatrix(deviceCamera);
	device->getCurrentBwImage(deviceBwImage);
	external->getCurrentBwImage(externalBwImage);
	device->getCurrentKeyPoints(deviceKeyPoints);
	external->getCurrentKeyPoints(externalKeyPoints);
	vector<DMatch> matches;
	keyPointMatches(externalBwImage, externalKeyPoints, deviceBwImage, deviceKeyPoints, matches);

	vector<Point2f> devicePoints;
	vector<Point3f> alignedWorldPoints, worldPoints;
	external->getCurrentKeyWorldPoints(worldPoints);

	for (int i = 0; i < matches.size(); i++) {
		devicePoints.push_back(deviceKeyPoints[matches[i].queryIdx].pt);
		alignedWorldPoints.push_back(worldPoints[matches[i].trainIdx]);
	}

	solvePnPRansac(alignedWorldPoints, devicePoints, deviceCamera, Mat(), R, t);
}

void Tracker::keyPointMatches(Mat &externalImage, vector<KeyPoint> &externalKeyPoints, Mat &deviceImage, vector<KeyPoint> &deviceKeyPoints, vector<DMatch> &matches) {
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	Mat trainDescriptors, queryDescriptors;
	extractor.compute(externalImage, externalKeyPoints, trainDescriptors);
	extractor.compute(deviceImage, deviceKeyPoints, queryDescriptors);

	BFMatcher matcher;
	//FlannBasedMatcher matcher;
	vector<DMatch> initialMatches;
	matches.clear();
	//matcher.knnMatch(descriptors1, descriptors2, initialMatches, 2);
	matcher.match(queryDescriptors, trainDescriptors, matches);

	return;

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for(int i = 0; i < trainDescriptors.rows; i++) {
		double dist = initialMatches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.

	matches.clear();
	for(int i = 0; i < trainDescriptors.rows; i++) {
		if(initialMatches[i].distance <= max(1.5*min_dist, 0.02)) {
			matches.push_back(initialMatches[i]);
		}
	}
}



