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
#define FAST_THRESHOLD 40


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

	FastFeatureDetector f = FastFeatureDetector(FAST_THRESHOLD);
	Device d(deviceVideo, f);
	External e(externalVideo);
	Tracker t(d, e);
	Viewer v(t);

	while (t.advanceFrame()) {
		v.updateDisplay();
	}






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

	pcl::RangeImagePlanar rangeImage;

	cout << "Getting initial frame from external camera..." << endl;
	externalVideo.getFirstImageFrame(externalImage);
	externalVideo.getFirstDepthFrame(externalRangeImage);

	makeRangeImagePlanar(rangeImage, externalRangeImage, externalVideo);

	cvtColor(externalImage, externalImageBw, CV_RGB2GRAY);
	extractKeyPoints(externalImageBw,  externalKeyPoints);
	findDepthKeyPoints(externalKeyPoints, externalRangeImage, objectKeyPoints, rangeImage);
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
