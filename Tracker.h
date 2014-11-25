/*
 * Tracker.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef TRACKER_H_
#define TRACKER_H_

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class Tracker {
public:
	Tracker(cv::Mat &startImage, cv::Mat &startDepth, cv::Mat &deviceCamera, float depthFocal);
	void computePosePnP(cv::Mat &deviceImage, cv::Mat &R, cv::Mat &t);
	std::vector<cv::DMatch>* getMatches();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRoomPointCloud();
	cv::Point3f depthImagePointToRoomLocation(cv::Point2f imageCoord);
	friend class Viewer;

private:
	float depthFocalLength;
	cv::Mat deviceCameraMatrix;
	cv::Mat currentDepthImage;
	cv::Mat currentExternalImage;
	cv::Mat currentExternalBwImage;
	cv::Mat roomDepth;
	cv::Mat roomImage;
	cv::Mat roomBwImage;
	cv::Mat deviceBwImage;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr roomPointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedPointCloud;
	std::vector<cv::KeyPoint> roomKeyPoints;
	std::vector<cv::KeyPoint> deviceKeyPoints;
	std::vector<cv::Point3f> roomKeyLocation;
	std::vector<cv::DMatch> matches;
	pcl::ModelCoefficients::Ptr coefficients;

	void keyPointMatches(cv::Mat &trainImage, std::vector<cv::KeyPoint> &trainKeypoints, cv::Mat &queryImage, std::vector<cv::KeyPoint> &queryKeypoints, std::vector<cv::DMatch> &matches);
	void extractKeyPoints(cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints);
	void buildRoomPointCloud();
	void buildRoomKeyLocations();
	void resizeDeviceImage(cv::Mat &input, cv::Mat &output, cv::Size targetSize);
};



#endif /* TRACKER_H_ */
