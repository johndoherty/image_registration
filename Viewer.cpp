/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

void augmentImage(Mat &input, Mat &output, vector<KeyPoint>& keypoints, std::string text = "") {
	//drawKeypoints(input, keypoints, output);
	input.copyTo(output);
	putText(output, text, Point(10, output.rows - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255));
	for (int i = 0; i < keypoints.size(); i++) {
		circle(output, keypoints[i].pt, 5, Scalar(255, 0, 0), 1);
	}
}


void makeViewableDepthImage(Mat &input, Mat &output) {
	const float scaleFactor = 0.05f;
	input.convertTo(output, CV_8UC1, scaleFactor);
}
