#include <iostream> // for standard I/O
#include <string>   // for strings
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "pcl/range_image/range_image_planar.h"
#include "CVVideoInput.h"
#include "ONIVideoInput.h"

using namespace std;
using namespace cv;

#define FRAME_BY_FRAME false

void extractKeyPoints(Mat &input, vector<KeyPoint> &keypoints) {
	FAST(input, keypoints, 40);
}

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

void keyPointMatches(Mat &trainImage, vector<KeyPoint> &trainKeypoints, Mat &queryImage, vector<KeyPoint> &queryKeypoints, vector<DMatch> &matches) {
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	Mat trainDescriptors, queryDescriptors;
	extractor.compute(trainImage, trainKeypoints, trainDescriptors);
	extractor.compute(queryImage, queryKeypoints, queryDescriptors);

	BFMatcher matcher;
	//FlannBasedMatcher matcher;
	vector<DMatch> initialMatches;
	matches.clear();
	//matcher.knnMatch(descriptors1, descriptors2, initialMatches, 2);
	matcher.match(queryDescriptors, trainDescriptors, matches);

	return;

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for(int i = 0; i < trainDescriptors.rows; i++) {
		double dist = initialMatches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.

	matches.clear();
	for(int i = 0; i < trainDescriptors.rows; i++) {
		if(initialMatches[i].distance <= max(1.5*min_dist, 0.02)) {
			matches.push_back(initialMatches[i]);
		}
	}
}

void findDepthKeyPoints(vector<KeyPoint> &depthKeypoints, Mat &depth, vector<Point3f> &objectPoints) {
	objectPoints.clear();
	for (int i = 0; i < depthKeypoints.size(); i++) {
		int x = (int)depthKeypoints[i].pt.x;
		int y = (int)depthKeypoints[i].pt.y;
		Point3f newPoint = Point3f(depthKeypoints[i].pt.x, depthKeypoints[i].pt.y, depth.at<float>(y, x));
		objectPoints.push_back(newPoint);
	}
}

void alignDepthVectors(vector<Point3f> &trainKeypoints, vector<KeyPoint> &queryKeypoints, vector<DMatch> &matches, vector<Point3f> &alignedTrainKeypoints, vector<Point2f> &alignedQueryKeypoints) {
	alignedTrainKeypoints.clear();
	alignedQueryKeypoints.clear();
	for (int i = 0; i < matches.size(); i++) {
		alignedTrainKeypoints.push_back(trainKeypoints[matches[i].trainIdx]);
		alignedQueryKeypoints.push_back(queryKeypoints[matches[i].queryIdx].pt);
	}
}

void projectDepthMap(vector<Point3f> &depthKeypoints, vector<KeyPoint> &imageKeypoints, Mat &T, Mat &R, Mat &projectedDepthImage) {
	// Compute center from T and R (-Rt?)
	// compute distance from center to each depth keypoint
	// store that as pixel value in new image for image keypoint and surrounding pixels
	// Gaussian blur the image
}

int main() {
	cout << "Initializing camera inputs..." << endl;
	CVVideoInput deviceVideo = CVVideoInput("/Users/john/Dropbox/School/Research/videos/video2.mp4");
	ONIVideoInput externalVideo = ONIVideoInput("/Users/john/Dropbox/School/Research/videos/record1.oni", 400);
	cout << "Camera inputs initialized" << endl;

	Mat deviceImage, deviceImageBw, externalImage, externalImageBw, externalRangeImage;
	Mat augmentedDeviceImage, augmentedExternalImage, viewableDepthOutput;
	Mat imgMatches;
	Mat objectPoints;
	Mat R, Rmat, T, K;
	Mat transImg = Mat(600, 600, CV_8UC1);

	// TODO: Set k as camera matrix
	K = Mat::eye(3, 3, CV_64F);

	vector<KeyPoint> deviceKeyPoints;
	vector<Point2f> matchedDevicePoints;
	vector<KeyPoint> externalKeyPoints;
	vector<Point3f> objectKeyPoints;
	vector<Point3f> matchedObjectPoints;
	vector<DMatch> matches;

	cout << "Getting initial frame from external camera..." << endl;
	externalVideo.getFirstImageFrame(externalImage);
	externalVideo.getFirstDepthFrame(externalRangeImage);
	cvtColor(externalImage, externalImageBw, CV_RGB2GRAY);
	extractKeyPoints(externalImageBw,  externalKeyPoints);
	findDepthKeyPoints(externalKeyPoints, externalRangeImage, objectKeyPoints);
	augmentImage(externalImage, augmentedExternalImage, externalKeyPoints);
	imshow("External Key Points", augmentedExternalImage);
	cout << "Frame received" << endl;

	bool validDeviceImage = deviceVideo.getNextImageFrame(deviceImage);
	bool validExternalImage = externalVideo.getNextImageFrame(externalImage);
	bool validDepthImage = externalVideo.getNextDepthFrame(externalRangeImage);

	while (validDeviceImage && validExternalImage && validDepthImage) {
		cvtColor(deviceImage, deviceImageBw, CV_RGB2GRAY);
		extractKeyPoints(deviceImageBw, deviceKeyPoints);
		keyPointMatches(externalImageBw, externalKeyPoints, deviceImageBw, deviceKeyPoints, matches);
		//keyPointMatches(externalImageFrameBw, externalKeyPoints, externalImageFrameBw, externalKeyPoints, matches);
		alignDepthVectors(objectKeyPoints, deviceKeyPoints, matches, matchedObjectPoints, matchedDevicePoints);
		//alignDepthVectors(objectKeyPoints, externalKeyPoints, matches, matchedObjectPoints, matchedDevicePoints);
		solvePnPRansac(matchedObjectPoints, matchedDevicePoints, K, Mat(), R, T);
		Rodrigues(R, Rmat);
		cout << "R: " << Rmat << endl;
		cout << "T: " << T << endl;

		transImg = Scalar(0);
		circle(transImg, Point(300, 300), 5, Scalar(255), 2);
		circle(transImg, Point(((T.at<float>(0)/5) + 300), (T.at<float>(1)/5) + 300), 5, Scalar(255), 2);

		augmentImage(deviceImage, augmentedDeviceImage, deviceKeyPoints);
		drawMatches(deviceImageBw, deviceKeyPoints, externalImageBw, externalKeyPoints,
				matches, imgMatches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		/*drawMatches(externalImageFrameBw, externalKeyPoints, externalImageFrameBw, externalKeyPoints,
						matches, imgMatches, Scalar::all(-1), Scalar::all(-1),
						vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );*/

		makeViewableDepthImage(externalRangeImage, viewableDepthOutput);
		imshow("Device Video", augmentedDeviceImage);
		imshow("External Video", externalImage);
		imshow("Matches", imgMatches);
		imshow("Depth image", viewableDepthOutput);
		imshow("Translation", transImg);

		validDeviceImage = deviceVideo.getNextImageFrame(deviceImage);
		validExternalImage = externalVideo.getNextImageFrame(externalImage);
		validDepthImage = externalVideo.getNextDepthFrame(externalRangeImage);
		if (FRAME_BY_FRAME) {
			waitKey(0);
		} else {
			waitKey(1);
		}
	}

	/*
	outputVideo.open(NAME, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);

	if (!outputVideo.isOpened()) {
		cout << "Could not open the output video for write: " << source << endl;
		return -1;
	}
    string::size_type pAt = filename.find_last_of('.');    // Find extension point
	const string NAME = "results/" + destination + ".mp4"; // Form the new name with container

	int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC)); // Get Codec Type- Int form

	cout << "Codec int: " << ex << endl;

	// Transform from int to char via Bitwise operators
	char EXT[] = { (char) (ex & 0XFF), (char) ((ex & 0XFF00) >> 8), (char) ((ex
			& 0XFF0000) >> 16), (char) ((ex & 0XFF000000) >> 24), 0 };

	Size S = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH), // Acquire input size
	(int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));

    cout << "Input frame resolution: Width=" << S.width << "  Height="
			<< S.height << " of nr#: "
			<< inputVideo.get(CV_CAP_PROP_FRAME_COUNT) << endl;
	cout << "Input codec type: " << EXT << endl;


	 */
	//outputVideo.write(dst); //save or


	return 0;
}
