/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

using namespace std;
using namespace cv;


void mouseEventOccured (const pcl::visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	std::cout << "s was pressed => saving camera angle as camera_params" << std::endl;
	viewer->saveCameraParameters("camera_params");
}

Viewer::Viewer(Tracker &t) {
	Mat augmentedRoomImage;
	tracker = &t;
	frame = 0;
	v0 = 0; v1 = 1;
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
	viewer->addPointCloud(tracker->roomPointCloud, "Room Point Cloud", v0);
	viewer->addPointCloud(tracker->segmentedPointCloud, "Segmented Room Point Cloud", v1);
	viewer->addCoordinateSystem(0.1);
	//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	//viewer->registerMouseCallback(mouseEventOccured, (void*)&viewer);
	viewer->loadCameraParameters("camera_params");

	for (int i = 0; i < tracker->roomKeyLocation.size(); i++) {
		Point3f p = tracker->roomKeyLocation[i];
		viewer->addSphere(pcl::PointXYZ(p.x, p.y, p.z), 0.1, 255, 0, 0, to_string(i), v0);
	}

	Point3f head = tracker->currentHeadLocation;
	viewer->addSphere(pcl::PointXYZ(head.x, head.y, head.z), 0.1, 0, 0, 255, "Head", v0);
	augmentImage(tracker->roomBwImage, augmentedRoomImage, tracker->roomKeyPoints);

	pcl::PointXYZ p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	viewer->addSphere(p, 0.1, "camera");
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
	Mat imageMatches, augmentedDeviceImage, viewableDepth;
	Mat center = -1 * R.t() * t;
	cout << "Device key points: " << tracker->deviceKeyPoints.size() << endl;
	cout << "Room key points: " << tracker->roomKeyPoints.size() << endl;
	cout << "Number of matches: " << tracker->matches.size() << endl;
	pcl::PointXYZ p;
	cout << center.at<double>(0) << ", " << center.at<double>(1) << ", " << center.at<double>(2) << endl;
	p.x = center.at<double>(0);
	p.y = center.at<double>(1);
	p.z = center.at<double>(2);
	Point3f head = tracker->currentHeadLocation;
	cout << "Head location: " << head << endl;
	viewer->updateSphere(pcl::PointXYZ(head.x, head.y, head.z), 0.1, 0, 0, 255, "Head");
	viewer->updateSphere(p, 0.1, 0, 255, 0, "camera");
	//viewer->loadCameraParameters("camera_params");
	for (int i = 0; i < tracker->matches.size(); i++) {
		/*if (tracker->matches[i].queryIdx >= tracker->deviceKeyPoints.size())
			cout << "Query index too large: " << tracker->matches[i].queryIdx << ", match: " << i << endl;
		if (tracker->matches[i].trainIdx >= tracker->roomKeyPoints.size())
			cout << "Train index too large: " << tracker->matches[i].trainIdx << ", match: " << i << endl;*/
	}

	if (!tracker->currentDepthImage.empty()) {
		makeViewableDepthImage(tracker->currentDepthImage, viewableDepth);
		imshow("Depth", viewableDepth);
	}

	drawMatches(tracker->deviceBwImage, tracker->deviceKeyPoints, tracker->roomBwImage, tracker->roomKeyPoints,
			tracker->matches, imageMatches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	augmentImage(tracker->deviceBwImage, augmentedDeviceImage, deviceKeyPoints);
	imshow("Device Video", augmentedDeviceImage);
	imshow("Matches", imageMatches);
	waitKey(1);
	frame++;
	/*
	transImg = Scalar(0);
	circle(transImg, Point(300, 300), 5, Scalar(255), 2);
	circle(transImg, Point(((T.at<float>(0)/5) + 300), (T.at<float>(1)/5) + 300), 5, Scalar(255), 2);
	 */
}







