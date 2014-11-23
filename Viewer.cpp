/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

using namespace std;
using namespace cv;

#define FRAME_BY_FRAME false


Viewer::Viewer(Tracker &t) {
	Mat augmentedRoomImage, viewableDepth;
	tracker = &t;
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->addPointCloud(tracker->roomPointCloud, "Point Cloud");
	viewer->addCoordinateSystem(100.0);
	//viewer->setCameraPosition(0, 0, 0, 1, 0, 0);

	augmentImage(tracker->roomBwImage, augmentedRoomImage, tracker->roomKeyPoints);
	makeViewableDepthImage(tracker->roomDepth, viewableDepth);
	imshow("Room", augmentedRoomImage);
	waitKey(1);
}

void Viewer::augmentImage(Mat &input, Mat &output, vector<KeyPoint>& keypoints, std::string text) {
	input.copyTo(output);
	putText(input, text, Point(10, input.rows - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255));
	for (int i = 0; i < keypoints.size(); i++) {
		circle(input, keypoints[i].pt, 5, Scalar(255, 0, 0), 1);
	}
}

void Viewer::makeViewableDepthImage(Mat &input, Mat &output) {
	const float scaleFactor = 0.05f;
	input.convertTo(output, CV_8UC1, scaleFactor);
}

void Viewer::updateDisplay(Mat R, Mat t) {
	Mat imageMatches, augmentedDeviceImage;

	drawMatches(tracker->deviceBwImage, tracker->deviceKeyPoints, tracker->roomBwImage, tracker->roomKeyPoints,
			tracker->matches, imageMatches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	augmentImage(tracker->deviceBwImage, augmentedDeviceImage, deviceKeyPoints);
	imshow("Device Video", augmentedDeviceImage);
	imshow("Matches", imageMatches);
	if (FRAME_BY_FRAME) {
		waitKey(0);
	} else {
		waitKey(1);
	}
	/*
	transImg = Scalar(0);
	circle(transImg, Point(300, 300), 5, Scalar(255), 2);
	circle(transImg, Point(((T.at<float>(0)/5) + 300), (T.at<float>(1)/5) + 300), 5, Scalar(255), 2);
	*/
}







