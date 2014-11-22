/*
 * Tracker.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef TRACKER_H_
#define TRACKER_H_

class Tracker {
public:
	Tracker(Device &d, External &e);
	bool advanceFrame();
	void computePosePnP(cv::Mat &R, cv::Mat &t);
private:
	Device *device;
	External *external;
};



#endif /* TRACKER_H_ */
