/*
 * PointCloudWrapper.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: john
 */

#include "PointCloudWrapper.h"

using namespace std;
using namespace cv;
using namespace pcl;

PointCloudWrapper::PointCloudWrapper(boost::shared_ptr<DepthInput> input) {
	depthInput = input;
}

// Return a point cloud given a depth image
void PointCloudWrapper::pointCloudForDepthImage(Mat &depthImage, Mat &image, PointCloud<PointXYZRGB>::Ptr pointCloud) {
	float minX, minY, minZ = 100;
	float maxX, maxY, maxZ = 0;
	for (int row = 0; row < depthImage.rows; row++) {
		for (int col = 0; col < depthImage.cols; col++) {
			Eigen::Vector3f v;
			Point3f location = depthImageCoordToWorldCoord(depthImage, Point2f(col, row));
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
			point.r = image.at<Vec3b>(row,col)[2];
			point.g = image.at<Vec3b>(row,col)[1];
			point.b = image.at<Vec3b>(row,col)[0];
			pointCloud->push_back(point);
		}
	}
	cout << "X range: " << minX << ", " << maxX << endl;
	cout << "Y range: " << minY << ", " << maxY << endl;
	cout << "Z range: " << minZ << ", " << maxZ << endl;

}

// A helper function to segment the point cloud and color points appropriatly.
void PointCloudWrapper::segmentPointCloud(PointCloud<PointXYZRGB>::Ptr pointCloud, PointCloud<PointXYZRGB>::Ptr segmentedPointCloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*pointCloud));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(50);
	seg.setMaxIterations(1000);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	for (int i = 0; i < 4; i ++) {
		seg.setInputCloud(tempPointCloud);
		seg.segment(*inliers, *coefficients);
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


void PointCloudWrapper::depthImageCoordsToWorldCoords(Mat &depthImage, std::vector<cv::Point2f> imageCoords, std::vector<cv::Point3f> &worldCoords) {
	return depthInput->depthImageCoordsToWorldCoords(depthImage, imageCoords, worldCoords);
}

// Helper function for converting a single depth image coord into a world coord.
Point3f PointCloudWrapper::depthImageCoordToWorldCoord(Mat &depthImage, Point2f imageCoord) {
	vector<Point2f> imageCoords;
	vector<Point3f> worldCoords;
	worldCoords.clear();
	imageCoords.clear();
	imageCoords.push_back(imageCoord);
	depthImageCoordsToWorldCoords(depthImage, imageCoords, worldCoords);
	return worldCoords[0];
}

bool PointCloudWrapper::validWorldCoord(cv::Point3f worldCoord) {
	return (worldCoord.x != 0) || (worldCoord.y != 0) || (worldCoord.z != 0);
}
