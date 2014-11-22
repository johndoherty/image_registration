/*
 * Viewer.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef VIEWER_H_
#define VIEWER_H_

#include "Tracker.h"

class Viewer {
public:
	Viewer(Tracker &t);
	void updateDisplay();
private:
	Tracker* tracker;
};



#endif /* VIEWER_H_ */
