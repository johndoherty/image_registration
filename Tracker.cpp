/*
 * Tracker.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Tracker.h"
#include <pcl/filters/extract_indices.h>

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
	PointCloud<PointXYZRGB> m;
	segmentedPointCloud = m.makeShared();

	buildRoomPointCloud();
	buildRoomKeyLocations();
}

Point3f Tracker::depthImagePointToRoomLocation(Point2f imageCoord) {
	Point3f location;
	unsigned short z = roomDepth.at<unsigned short>(imageCoord.y, imageCoord.x);
	float z_float = ((float)z) * 0.001f;

	location.x = (imageCoord.x - (.5 * roomDepth.cols)) * (z_float / depthFocalLength);
	location.y = (imageCoord.y - (.5 * roomDepth.rows)) * (z_float / depthFocalLength);
	location.z = z_float;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	float minX, minY, minZ = 100;
	float maxX, maxY, maxZ = 0;
	for (int row = 0; row < roomDepth.rows; row++) {
		for (int col = 0; col < roomDepth.cols; col++) {
			Eigen::Vector3f v;
			Point3f location = depthImagePointToRoomLocation(Point2f(col, row));
			PointXYZRGB point;
			point.x = location.x;
			point.y = location.y;
			point.z = location.z;
			if (point.x > maxX) maxX = point.x;
			if (point.x < minX) minX = point.x;
			if (point.y > maxY) maxY = point.y;
			if (point.y < minY) minY = point.y;
			if (point.z > maxZ) maxZ = point.z;
			if (point.z < minZ) minZ = point.z;
			point.r = roomImage.at<Vec3b>(row,col)[2];
			point.g = roomImage.at<Vec3b>(row,col)[1];
			point.b = roomImage.at<Vec3b>(row,col)[0];
			roomPointCloud->push_back(point);
			tempPointCloud->push_back(point);
		}
	}
	cout << "X range: " << minX << ", " << maxX << endl;
	cout << "Y range: " << minY << ", " << maxY << endl;
	cout << "Z range: " << minZ << ", " << maxZ << endl;

	coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(.05);
	seg.setMaxIterations(1000);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	for (int i = 0; i < 5; i ++) {
		seg.setInputCloud(tempPointCloud);
		seg.segment(*inliers, *coefficients);
		cout << "Inliers: " << inliers->indices.size() << endl;
		cout << "Total points: " << roomPointCloud->points.size() << endl;
		if (inliers->indices.size() == 0) {
			PCL_ERROR("Could not estimate a planar model for the given dataset.");
			break;
		}
		for (int j = 0; j < inliers->indices.size(); j++) {
			PointXYZRGB n = tempPointCloud->points[inliers->indices[j]];
			switch (i) {
			case 0:
				n.r = 255; n.g = 0; n.b = 0;
				break;
			case 1:
				n.r = 0; n.g = 255; n.b = 0;
				break;
			case 2:
				n.r = 0; n.g = 0; n.b = 255;
				break;
			case 3:
				n.r = 0; n.g = 200; n.b = 200;
				break;
			case 4:
				n.r = 200; n.g = 200; n.b = 0;
				break;
			default:
				n.r = 200; n.g = 200; n.b = 200;
				break;
			}
			segmentedPointCloud->push_back(n);
		}
		extract.setInputCloud(tempPointCloud);
		extract.setIndices(inliers);
		extract.setNegative (true);
		extract.filter(*tempPointCloud1);
		tempPointCloud.swap (tempPointCloud1);
	}
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
	resizeDeviceImage(deviceBwImage, deviceBwImage, roomBwImage.size());
	//resize(deviceBwImage, deviceBwImage, roomBwImage.size());
	extractKeyPoints(deviceBwImage, deviceKeyPoints);
	keyPointMatches(roomBwImage, roomKeyPoints, deviceBwImage, deviceKeyPoints, matches);

	vector<Point2f> devicePoints;
	vector<Point3f> alignedWorldPoints;
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



