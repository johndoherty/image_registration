/*
 * PointCloudWrapper.h
 *
 * This class is meant to make it easier to communicate between conversion between
 * depth images and 3D points using OpenNi, and then accessing those points as a
 * PCL point cloud.
 *
 *  Created on: Nov 28, 2014
 *      Author: john
 */

#ifndef POINTCLOUDWRAPPER_H_
#define POINTCLOUDWRAPPER_H_

#include "DepthInput.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PointCloudWrapper {
public:
	PointCloudWrapper(boost::shared_ptr<DepthInput> input);
	void pointCloudForDepthImage(cv::Mat &depthImage, cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	void segmentPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedPointCloud);
	void depthImageCoordsToWorldCoords(cv::Mat &depthImage, std::vector<cv::Point2f> imageCoords, std::vector<cv::Point3f> &worldCoords);
	cv::Point3f depthImageCoordToWorldCoord(cv::Mat &depthImage, cv::Point2f imageCoord);
	bool validWorldCoord(cv::Point3f worldCoord);
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
	boost::shared_ptr<DepthInput> depthInput;
};


#endif /* POINTCLOUDWRAPPER_H_ */
