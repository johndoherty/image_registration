/*
 * DepthInput.h
 *
 *  Created on: Nov 20, 2014
 *      Author: john
 */

#ifndef DEPTHINPUT_H_
#define DEPTHINPUT_H_

#include "pcl/point_cloud.h"

class DepthInput {
public:
	virtual bool getNextDepthFrame(cv::Mat &frame) = 0;
	//static void getDepthImageAsPointCloud(cv::Mat &depthImage, pcl::PointCloud &pointCloud);
};



#endif /* DEPTHINPUT_H_ */
