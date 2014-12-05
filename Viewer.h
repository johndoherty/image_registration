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
#include <GLUT/glut.h>

#define NUMBER_OF_COLORS 100

class Viewer {
public:
	Viewer(Tracker &t);
	void updateDisplay(cv::Mat R, cv::Mat t);
	void glInit ();
private:
	int v0;
	int v1;
	int frame;
	int numSpheres;
	GLuint programID;
	float cameraPose[16];
	Tracker* tracker;
	cv::Mat roomImage;
	std::vector<cv::KeyPoint> deviceKeyPoints;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	cv::Scalar colors[100];
	void makeViewableDepthImage(cv::Mat &input, cv::Mat &output);
	void augmentImage(cv::Mat &image, cv::Mat &output, std::vector<cv::KeyPoint>& keypoints, std::string text = "");
	void augmentImage(cv::Mat &image, cv::Mat &output, std::vector<cv::Point2f>& keypoints, std::string text = "");
	void draw();

};



#endif /* VIEWER_H_ */
