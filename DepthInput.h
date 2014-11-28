/*
 * DepthInput.h
 *
 *  Created on: Nov 20, 2014
 *      Author: john
 */

#ifndef DEPTHINPUT_H_
#define DEPTHINPUT_H_

#include "opencv2/core/core.hpp"
#include "pcl/point_cloud.h"

class DepthInput {
public:
	virtual bool getNextDepthFrame(cv::Mat &frame) = 0;
	virtual void depthImageCoordsToWorldCoords(cv::Mat &depthImage, std::vector<cv::Point2f> imageCoords, std::vector<cv::Point3f> &worldCoords) = 0;
};



#endif /* DEPTHINPUT_H_ */
