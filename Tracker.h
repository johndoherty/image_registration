/*
 * Tracker.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef TRACKER_H_
#define TRACKER_H_

#include "PointCloudWrapper.h"

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
	Tracker(cv::Mat &startImage, cv::Mat &startDepth, cv::Mat &deviceCamera, cv::Mat &distortion, boost::shared_ptr<PointCloudWrapper> wrapper);
	bool computePosePnP(cv::Mat &deviceImage, cv::Mat &depth, cv::Mat &external, cv::Point3f headLocation, cv::Mat &R, cv::Mat &t, bool usePrevious=false);
	friend class Viewer;

private:
	cv::Mat deviceCameraMatrix;
	cv::Mat currentDepthImage;
	cv::Mat currentExternalImage;
	cv::Mat currentExternalBwImage;
	cv::Mat roomDepth;
	cv::Mat roomImage;
	cv::Mat roomBwImage;
	cv::Mat deviceBwImage;
	cv::Mat distortionCoeff;
	cv::Point3f currentHeadLocation;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr roomPointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedPointCloud;
	std::vector<cv::KeyPoint> roomKeyPoints;
	std::vector<cv::KeyPoint> deviceKeyPoints;
	std::vector<cv::Point3f> roomKeyLocation;
	std::vector<cv::Point2f> alignedDevicePoints;
	std::vector<cv::Point3f> alignedWorldPoints;
	std::vector<int> inlierIndexes;

	std::vector<cv::DMatch> matches;
	pcl::ModelCoefficients::Ptr coefficients;
	boost::shared_ptr<PointCloudWrapper> pointCloudWrapper;

	void keyPointMatches(cv::Mat &trainImage, std::vector<cv::KeyPoint> &trainKeypoints, cv::Mat &queryImage, std::vector<cv::KeyPoint> &queryKeypoints, std::vector<cv::DMatch> &matches);
	void extractKeyPoints(cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints);
	void buildRoomPointCloud();
	void buildRoomKeyLocations();
	void resizeDeviceImage(cv::Mat &input, cv::Mat &output, cv::Size targetSize);
};



#endif /* TRACKER_H_ */
