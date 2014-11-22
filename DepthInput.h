/*
 * DepthInput.h
 *
 *  Created on: Nov 20, 2014
 *      Author: john
 */

#ifndef DEPTHINPUT_H_
#define DEPTHINPUT_H_

class DepthInput {
public:
	virtual bool getNextDepthFrame(cv::Mat &frame) = 0;
};



#endif /* DEPTHINPUT_H_ */
