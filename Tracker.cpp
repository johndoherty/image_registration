/*
 * Tracker.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Tracker.h"

using namespace std;
using namespace cv;
using namespace pcl;

#define FAST_THRESHOLD 40

Tracker::Tracker(cv::Mat &startImage, cv::Mat &startDepth, cv::Mat &deviceCamera, boost::shared_ptr<PointCloudWrapper> wrapper) {
	pointCloudWrapper = wrapper;
	startImage.copyTo(roomImage);
	startDepth.copyTo(roomDepth);
	deviceCamera.copyTo(deviceCameraMatrix);
	cvtColor(roomImage, roomBwImage, CV_RGB2GRAY);
	extractKeyPoints(roomBwImage, roomKeyPoints);

	PointCloud<PointXYZRGB> p;
	roomPointCloud = p.makeShared();
	PointCloud<PointXYZRGB> m;
	segmentedPointCloud = m.makeShared();

	pointCloudWrapper->pointCloudForDepthImage(roomDepth, roomImage, roomPointCloud);
	pointCloudWrapper->segmentPointCloud(roomPointCloud, segmentedPointCloud);
	buildRoomKeyLocations();
}

void Tracker::buildRoomKeyLocations() {
	vector<Point2f> imageCoords;
	imageCoords.clear();
	for (int i = 0; i < roomKeyPoints.size(); i++) {
		imageCoords.push_back(roomKeyPoints[i].pt);
	}
	pointCloudWrapper->depthImageCoordsToWorldCoords(roomDepth, imageCoords, roomKeyLocation);
}

void Tracker::extractKeyPoints(Mat &image, vector<KeyPoint> &keyPoints) {
	//FastFeatureDetector f = FastFeatureDetector(FAST_THRESHOLD);
	GoodFeaturesToTrackDetector f(
			500,	// maxCorners
			0.05,	// quality level
			5,		// min distance
			3,		// block size
			true,	// use harris
			0.04	// k
	);
	f.detect(image, keyPoints);
}

bool Tracker::computePosePnP(Mat &deviceImage, Mat& depth, Point3f headLocation, Mat &R, Mat &t) {
	Mat Rvec;
	deviceKeyPoints.clear();
	depth.copyTo(currentDepthImage);
	currentHeadLocation = Point3f(headLocation);
	cvtColor(deviceImage, deviceBwImage, CV_RGB2GRAY);
	resizeDeviceImage(deviceBwImage, deviceBwImage, roomBwImage.size());
	//resize(deviceBwImage, deviceBwImage, roomBwImage.size());
	extractKeyPoints(deviceBwImage, deviceKeyPoints);
	keyPointMatches(roomBwImage, roomKeyPoints, deviceBwImage, deviceKeyPoints, matches);

	vector<Point2f> devicePoints;
	vector<Point3f> alignedWorldPoints;
	if (matches.size() < 3) {
		cout << "Not enough matches" << endl;
		return false;
	}

	for (int i = 0; i < matches.size(); i++) {
		devicePoints.push_back(deviceKeyPoints[matches[i].queryIdx].pt);
		alignedWorldPoints.push_back(roomKeyLocation[matches[i].trainIdx]);
	}
	Mat inliers;
	cout << "Solving pnp..." << endl;
	solvePnPRansac(
			alignedWorldPoints,	// object points
			devicePoints,		// image points
			deviceCameraMatrix,	// camera matrix
			Mat(),				// distortion coeffs
			Rvec,				// rotation matrix
			t,					// translation matrix
			false,				// use initial guess
			1000,				// interation count
			5.0,				// inlier threshold
			100,				// number of inliers to stop
			inliers,			// inlier indexes
			ITERATIVE			// method
	);
	cout << "Number of inliers: " << inliers.size() << endl;
	Rodrigues(Rvec, R);
	return true;
}

void Tracker::resizeDeviceImage(Mat &input, Mat &output, Size targetSize) {
	Size inputSize = input.size();
	float widthOverHeight = ((float)inputSize.width) / ((float)inputSize.height);
	if (targetSize.width > targetSize.height) {
		targetSize.width = targetSize.height * widthOverHeight;
	} else {
		targetSize.height = targetSize.width / widthOverHeight;
	}
	resize(input, output, targetSize);
}

void Tracker::keyPointMatches(Mat &externalImage, vector<KeyPoint> &externalKeyPoints, Mat &deviceImage, vector<KeyPoint> &deviceKeyPoints, vector<DMatch> &matches1) {
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	vector<DMatch> initialMatches;
	Mat trainDescriptors, queryDescriptors;
	extractor.compute(externalImage, externalKeyPoints, trainDescriptors);
	extractor.compute(deviceImage, deviceKeyPoints, queryDescriptors);
	BFMatcher matcher;
	//FlannBasedMatcher matcher;
	initialMatches.clear();
	//matches1.reserve(queryDescriptors.rows);
	//matches1.clear();
	initialMatches.reserve(queryDescriptors.rows);
	//matcher.knnMatch(descriptors1, descriptors2, initialMatches, 2);
	matcher.match(queryDescriptors, trainDescriptors, initialMatches);
	//return;
	double max_dist = 0; double min_dist = 1000;
	//-- Quick calculation of max and min distances between keypoints
	for(int i = 0; i < trainDescriptors.rows; i++) {
		double dist = initialMatches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	cout << "Min dist: " << min_dist << endl;
	matches.clear();
	cout << "Initial matches: " << initialMatches.size() << endl;
	for(int i = 0; i < initialMatches.size(); i++) {
		if(initialMatches[i].distance <= max(1.7*min_dist, 350.0)) {
			matches.push_back(initialMatches[i]);
		}
		if (initialMatches[i].queryIdx >= deviceKeyPoints.size())
			cout << "Query index too large: " << initialMatches[i].queryIdx << ", match: " << i << endl;
		if (initialMatches[i].trainIdx >= roomKeyPoints.size())
			cout << "Train index too large: " << initialMatches[i].trainIdx << ", match: " << i << endl;
	}
	cout << "Final matches: " << matches.size() << endl;
}



