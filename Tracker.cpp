/*
 * Tracker.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Tracker.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
using namespace pcl;

#define FAST_THRESHOLD 40

Tracker::Tracker(cv::Mat &startImage, cv::Mat &startDepth, cv::Mat &deviceCamera, float depthFocal) {
	depthFocalLength = depthFocal;
	startImage.copyTo(roomImage);
	startDepth.copyTo(roomDepth);
	deviceCamera.copyTo(deviceCameraMatrix);
	cvtColor(roomImage, roomBwImage, CV_RGB2GRAY);
	extractKeyPoints(roomBwImage, roomKeyPoints);

	PointCloud<PointXYZRGB> p;
	roomPointCloud = p.makeShared();
	buildRoomPointCloud();
	buildRoomKeyLocations();
}

Point3f Tracker::depthImagePointToRoomLocation(Point2f imageCoord) {
	Point3f location;
	unsigned short z = roomDepth.at<unsigned short>(imageCoord.y, imageCoord.x);
	location.x = (imageCoord.x - (.5 * roomDepth.cols)) * (z / depthFocalLength);
	location.y = (imageCoord.y - (.5 * roomDepth.rows)) * (z / depthFocalLength);
	location.z = z;
	return location;
}

void Tracker::buildRoomKeyLocations() {
	for (int i = 0; i < roomKeyPoints.size(); i++) {
		Eigen::Vector3f v;
		Point3f location = depthImagePointToRoomLocation(roomKeyPoints[i].pt);
		roomKeyLocation.push_back(location);
	}
}

void Tracker::buildRoomPointCloud() {
	cout << "Number of keypoints: " << roomKeyPoints.size() << endl;
	for (int row = 0; row < roomDepth.rows; row++) {
		for (int col = 0; col < roomDepth.cols; col++) {
			Eigen::Vector3f v;
			Point3f location = depthImagePointToRoomLocation(Point2f(col, row));
			PointXYZRGB point;
			point.x = location.x;
			point.y = location.y;
			point.z = location.z;
			point.r = roomImage.at<Vec3b>(row,col)[2];
			point.g = roomImage.at<Vec3b>(row,col)[1];
			point.b = roomImage.at<Vec3b>(row,col)[0];
			roomPointCloud->push_back(point);
		}
	}
}


void Tracker::extractKeyPoints(Mat &image, vector<KeyPoint> &keyPoints) {
	FastFeatureDetector f = FastFeatureDetector(FAST_THRESHOLD);
	f.detect(image, keyPoints);
}

vector<DMatch>* Tracker::getMatches() {
	return &matches;
}

PointCloud<PointXYZRGB>::Ptr Tracker::getRoomPointCloud() {
	return roomPointCloud;
}

void Tracker::computePosePnP(cv::Mat &deviceImage, cv::Mat &R, cv::Mat &t) {
	Mat Rvec;
	deviceKeyPoints.clear();
	cvtColor(deviceImage, deviceBwImage, CV_RGB2GRAY);
	extractKeyPoints(deviceBwImage, deviceKeyPoints);
	cout << "Matching" << endl;
	keyPointMatches(roomBwImage, roomKeyPoints, deviceBwImage, deviceKeyPoints, matches);

	vector<Point2f> devicePoints;
	vector<Point3f> alignedWorldPoints;
	cout << "World points: " << roomKeyLocation.size() << endl;
	cout << "External key points: " << roomKeyPoints.size() << endl;
	cout << "Device key points: " << deviceKeyPoints.size() << endl;
	for (int i = 0; i < matches.size(); i++) {
		devicePoints.push_back(deviceKeyPoints[matches[i].queryIdx].pt);
		alignedWorldPoints.push_back(roomKeyLocation[matches[i].trainIdx]);
	}

	cout << "Solving ransac..." << endl;
	solvePnPRansac(alignedWorldPoints, devicePoints, deviceCameraMatrix, Mat(), Rvec, t);
	Rodrigues(Rvec, R);
}

void Tracker::keyPointMatches(Mat &externalImage, vector<KeyPoint> &externalKeyPoints, Mat &deviceImage, vector<KeyPoint> &deviceKeyPoints, vector<DMatch> &matches) {
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	Mat trainDescriptors, queryDescriptors;
	extractor.compute(externalImage, externalKeyPoints, trainDescriptors);
	extractor.compute(deviceImage, deviceKeyPoints, queryDescriptors);
	cout << "Matching" << endl;
	//BFMatcher matcher;
	FlannBasedMatcher matcher;
	initialMatches.clear();
	//matches.clear();
	//matcher.knnMatch(descriptors1, descriptors2, initialMatches, 2);
	matcher.match(queryDescriptors, trainDescriptors, initialMatches);
	//return;
	double max_dist = 0; double min_dist = 1000;
	//-- Quick calculation of max and min distances between keypoints
	for(int i = 0; i < trainDescriptors.rows; i++) {
		double dist = initialMatches[i].distance;
		cout << dist << endl;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	cout << "Min dist: " << min_dist << endl;
	matches.clear();
	for(int i = 0; i < trainDescriptors.rows; i++) {
		if(initialMatches[i].distance <= max(1.6*min_dist, 250.0)) {
			matches.push_back(initialMatches[i]);
		}
	}
	cout << "Number of matches: " << matches.size() << endl;
}



