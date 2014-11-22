/*
 * External.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef EXTERNAL_H_
#define EXTERNAL_H_

#include "pcl/point_cloud.h"
#include "pcl/range_image/range_image_planar.h"

class External {
public:
	External(ImageInput &image, DepthInput &depth, cv::FeatureDetector &detector);
	bool advanceFrame();
	void getCurrentImage(cv::Mat &image);
	void getCurrentBwImage(cv::Mat &bwImage);
	void getCurrentDepthImage(cv::Mat &depthImage);
	void getCurrentKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
	void getCurrentKeyWorldPoints(std::vector<cv::Point3f>);
private:
	ImageInput *imageInput;
	DepthInput *depthInput;
	cv::Mat currentDepthImage;
	cv::Mat currentImage;
	cv::Mat currentBwImage;
	cv::Mat firstDepth;
	cv::Mat firstImage;
	cv::FeatureDetector *featureDetector;
	pcl::RangeImagePlanar *rangeImage;
	pcl::PointCloud<pcl::PointXYZ> *pointCloud;
	std::vector<cv::KeyPoint> firstKeyPoints;
	std::vector<cv::Point3f> keyWorldPoints;
};



#endif /* EXTERNAL_H_ */
