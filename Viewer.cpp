/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

using namespace std;
using namespace cv;

#define ONLY_INLIERS true


void mouseEventOccured (const pcl::visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	std::cout << "s was pressed => saving camera angle as camera_params" << std::endl;
	viewer->saveCameraParameters("camera_params");
}

Viewer::Viewer(Tracker &t) {
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
		Scalar color(((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX));
		//Scalar color(.5, .3, .7);
		if (i < NUMBER_OF_COLORS) {
			colors[i] = color;
		}

		Point3f p = tracker->roomKeyLocation[i];
		viewer->addSphere(pcl::PointXYZ(p.x, p.y, p.z), 0.1, (double)color[0], (double)color[1], (double)color[2], to_string(i), v0);
	}
	numSpheres = tracker->roomKeyLocation.size();

	Point3f head = tracker->currentHeadLocation;
	viewer->addSphere(pcl::PointXYZ(head.x, head.y, head.z), 0.1, 0, 0, 255, "Head", v0);
	cvtColor(tracker->roomBwImage, roomImage, CV_GRAY2BGR);

	pcl::PointXYZ p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	viewer->addSphere(p, 0.1, "camera");
	waitKey(1);
}



void Viewer::augmentImage(Mat &input, Mat &output, vector<KeyPoint>& keypoints, std::string text) {
	input.copyTo(output);
	vector<Point2f> points;
	points.clear();
	for (int i = 0; i < keypoints.size(); i++) {
		points.push_back(keypoints[i].pt);
	}
	augmentImage(input, output, points, text);
}

void Viewer::augmentImage(Mat &input, Mat &output, vector<Point2f>& keypoints, std::string text) {
	input.copyTo(output);
	putText(output, text, Point(10, input.rows - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255));
	for (int i = 0; i < keypoints.size(); i++) {
		Scalar color(255, 0, 0);
		if (i < NUMBER_OF_COLORS) {
			color = colors[i];
			float r = color[2] * 255; float g = color[1] * 255; float b = color[0] *255;
			color[0] = r; color[1] = g; color[2] = b;
		}
		circle(output, keypoints[i], 7, color, 3);
	}
}

void Viewer::makeViewableDepthImage(Mat &input, Mat &output) {
	const float scaleFactor = 0.05f;
	input.convertTo(output, CV_8UC1, scaleFactor);
}

void Viewer::updateDisplay(Mat R, Mat t) {
	Mat imageMatches, augmentedDeviceImage, viewableDepth, augmentedRoomImage;
	Mat center = R.t() * t;
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

	for (int i = 0; i < numSpheres; i++) {
		viewer->removeShape(to_string(i));
	}
	numSpheres = 0;

	if (ONLY_INLIERS) {
		for (int i = 0; i < tracker->inlierIndexes.size(); i++) {
			Scalar color(255, 0, 0);
			if (i < NUMBER_OF_COLORS) {
				color = colors[i];
			}
			Point3f point = tracker->alignedWorldPoints[tracker->inlierIndexes[i]];
			viewer->addSphere(pcl::PointXYZ(point.x, point.y, point.z), 0.1, color[0], color[1], color[2], to_string(i), v0);
		}
		numSpheres = tracker->inlierIndexes.size();
	} else {
		for (int i = 0; i < tracker->alignedWorldPoints.size(); i++) {
			Scalar color(255, 0, 0);
			if (i < NUMBER_OF_COLORS) {
				color = colors[i];
			}
			Point3f point = tracker->alignedWorldPoints[i];
			viewer->addSphere(pcl::PointXYZ(point.x, point.y, point.z), 0.1, color[0], color[1], color[2], to_string(i), v0);
		}
		numSpheres = tracker->alignedDevicePoints.size();
	}

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
	//augmentImage(tracker->deviceBwImage, augmentedDeviceImage, deviceKeyPoints);

	cvtColor(tracker->deviceBwImage, augmentedDeviceImage, CV_GRAY2RGB);
	if (ONLY_INLIERS) {
		vector<Point2f> augmentPoints;
		for (int i = 0; i < tracker->inlierIndexes.size(); i++) {
			augmentPoints.push_back(tracker->alignedDevicePoints[tracker->inlierIndexes[i]]);
		}
		augmentImage(augmentedDeviceImage, augmentedDeviceImage, augmentPoints);
	} else {
		augmentImage(augmentedDeviceImage, augmentedDeviceImage, tracker->alignedDevicePoints);
	}



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







