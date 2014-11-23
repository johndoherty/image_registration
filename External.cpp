/*
 * External.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "External.h"
#include "opencv2/highgui/highgui.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;
using namespace cv;
using namespace pcl;


External::External(ImageInput &image, DepthInput &depth, cv::FeatureDetector &detector) {
	imageInput = &image;
	depthInput = &depth;
	featureDetector = &detector;
	keyWorldPoints = new vector<Point3f>();
	advanceFrame();
	getCurrentImage(firstImage);
	getCurrentDepthImage(firstDepth);
	getCurrentBwImage(firstBwImage);
	//imageInput->getNextImageFrame(firstImage);
	//depthInput->getNextDepthFrame(firstDepth);

	cout << "Building range image..." << endl;
	rangeImage = new RangeImagePlanar();

	PointCloud<PointXYZRGB> p;
	pointCloud = p.makeShared();
	Point2f focalLength = depthInput->getFocalLength();
	Point2f centerOfProjection = depthInput->getCenterOfProjection();
	//firstDepth.convertTo(firstDepth, CV_64F);
	/*float *data = (float*)firstDepth.data;
	rangeImage->setDepthImage(
			firstDepth.ptr<float>(0, 0),
			firstDepth.cols,
			firstDepth.rows,
			centerOfProjection.x,
			centerOfProjection.y,
			focalLength.x,
			focalLength.y
	);*/

	cout << "Built range image" << endl;

	getCurrentKeyPoints(firstKeyPoints);
	cout << "Number of keypoints: " << firstKeyPoints.size() << endl;
	int focal = 500;

	for (int row = 0; row < firstDepth.rows; row++) {
		for (int col = 0; col < firstDepth.cols; col++) {
			Eigen::Vector3f v;
			/*rangeImage->calculate3DPoint((float)col, (float)row, firstDepth.at<float>(row, col), v);
			PointXYZ point;
			point.x = v[0];
			point.y = v[1];
			point.z = v[2];
			*/
			unsigned short z = firstDepth.at<unsigned short>(row, col)/5;
			PointXYZRGB point;
			point.x = (col - (.5 * firstDepth.cols)) * (z / focal);
			point.y = (row - (.5 * firstDepth.rows)) * (z / focal);
			point.z = z;
			point.r = firstImage.at<Vec3b>(row,col)[2];
			point.g = firstImage.at<Vec3b>(row,col)[1];
			point.b = firstImage.at<Vec3b>(row,col)[0];
			//cout << point.x << ", " << point.y << ", " << point.z << endl;
			pointCloud->push_back(point);

		}
	}

	for (int i = 0; i < firstKeyPoints.size(); i++) {
		Eigen::Vector3f v;
		//rangeImage->calculate3DPoint((float)firstKeyPoints[i].pt.x, (float)firstKeyPoints[i].pt.y, firstDepth.at<float>(firstKeyPoints[i].pt), v);
		Point2f p = firstKeyPoints[i].pt;
		Point3f point;
		unsigned short z = firstDepth.at<unsigned short>(p)/5;
		point.x = (p.x - (.5 * firstDepth.cols)) * (z / focal);
		point.y = (p.y - (.5 * firstDepth.rows)) * (z / focal);
		point.z = z;
		//point.x = v[0];
		//point.y = v[1];
		//point.z = v[2];
		keyWorldPoints->push_back(point);
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

vector<KeyPoint>* External::getFirstKeyPoints() {
	return &firstKeyPoints;
}

void External::getCurrentDepthImage(cv::Mat &depthImage) {
	currentDepthImage.copyTo(depthImage);
}

void External::getCurrentBwImage(Mat &bwImage) {
	currentBwImage.copyTo(bwImage);
}

vector<Point3f>* External::getCurrentKeyWorldPoints() {
	return keyWorldPoints;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr External::getPointCloud() {
	return pointCloud;
}

void External::getFirstBWImage(cv::Mat &bw) {
	firstBwImage.copyTo(bw);
}
