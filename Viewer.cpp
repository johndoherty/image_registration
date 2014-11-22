/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

using namespace std;
using namespace cv;

#define FRAME_BY_FRAME false;

void augmentImage(Mat &input, Mat &output, vector<KeyPoint>& keypoints, std::string text = "") {
	//drawKeypoints(input, keypoints, output);
	input.copyTo(output);
	putText(output, text, Point(10, output.rows - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255));
	for (int i = 0; i < keypoints.size(); i++) {
		circle(output, keypoints[i].pt, 5, Scalar(255, 0, 0), 1);
	}
}

void Viewer::updateDisplay() {
	if (FRAME_BY_FRAME) {
		waitKey(0);
	} else {
		waitKey(1);
	}
}

void makeViewableDepthImage(Mat &input, Mat &output) {
	const float scaleFactor = 0.05f;
	input.convertTo(output, CV_8UC1, scaleFactor);
}


transImg = Scalar(0);
circle(transImg, Point(300, 300), 5, Scalar(255), 2);
circle(transImg, Point(((T.at<float>(0)/5) + 300), (T.at<float>(1)/5) + 300), 5, Scalar(255), 2);

augmentImage(deviceImage, augmentedDeviceImage, deviceKeyPoints);
drawMatches(deviceImageBw, deviceKeyPoints, externalImageBw, externalKeyPoints,
		matches, imgMatches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
