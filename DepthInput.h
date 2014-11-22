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
	virtual cv::Point2f getFocalLength() const = 0;
	virtual cv::Point2f getCenterOfProjection() const = 0;
};



#endif /* DEPTHINPUT_H_ */
