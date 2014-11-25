/*
 * Viewer.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef VIEWER_H_
#define VIEWER_H_

#include "Tracker.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/common/common.h"

class Viewer {
public:
	Viewer(Tracker &t);
	void updateDisplay(cv::Mat R, cv::Mat t);
private:
	int v0;
	int v1;
	int frame;
	Tracker* tracker;
	std::vector<cv::KeyPoint> deviceKeyPoints;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	void makeViewableDepthImage(cv::Mat &input, cv::Mat &output);
	void augmentImage(cv::Mat &image, cv::Mat &output, std::vector<cv::KeyPoint>& keypoints, std::string text = "");
};



#endif /* VIEWER_H_ */
