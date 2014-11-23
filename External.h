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
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ImageInput.h"
#include "DepthInput.h"


class External {
public:
	External(ImageInput &image, DepthInput &depth, cv::FeatureDetector &detector);
	bool advanceFrame();
	void getCurrentImage(cv::Mat &image);
	void getCurrentBwImage(cv::Mat &bwImage);
	void getCurrentDepthImage(cv::Mat &depthImage);
	void getCurrentKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
	std::vector<cv::KeyPoint>* getFirstKeyPoints();
	void getFirstBWImage(cv::Mat &bw);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();
	std::vector<cv::Point3f>* getCurrentKeyWorldPoints();
private:
	ImageInput *imageInput;
	DepthInput *depthInput;
	cv::Mat currentDepthImage;
	cv::Mat currentImage;
	cv::Mat currentBwImage;
	cv::Mat firstDepth;
	cv::Mat firstImage;
	cv::Mat firstBwImage;
	cv::FeatureDetector *featureDetector;
	pcl::RangeImagePlanar *rangeImage;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
	std::vector<cv::KeyPoint> firstKeyPoints;
	std::vector<cv::Point3f> *keyWorldPoints;
};



#endif /* EXTERNAL_H_ */
