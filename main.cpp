#include <iostream> // for standard I/O
#include <string>   // for strings
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include "CVVideoInput.h"
#include "ONIVideoInput.h"
#include "Tracker.h"
#include "Viewer.h"
#include "PointCloudWrapper.h"

using namespace std;
using namespace cv;

#define FRAME_BY_FRAME false

Mat roomDepth, roomImage, currentDepth, currentExternalImage, deviceImage, R, t;
Mat cameraMatrix, distortionCoeff;
Mat viewableRoomDepth, viewableRoomImage, viewableCurrentDepth, viewableCurrentExternalImage, viewableDeviceImage;
Point3f headLocation;

int main() {
	float focal = 200;
	cameraMatrix = (Mat_<float>(3, 3) << 300.0, 0, 200.5, 0, 300.0, 100.5, 0, 0, 1);
	//cameraMatrix = (Mat_<float>(3, 3) << 460.6865232099297, 0, 399.5, 0, 460.6865232099297, 239.5, 0, 0, 1);
	//cameraMatrix = (Mat_<float>(3, 3) << 460.6865232099297, 0, 320, 0, 460.6865232099297, 240, 0, 0, 1);
	distortionCoeff = Mat();
	//cameraMatrix = (Mat_<float>(3, 3) << 654.4783072499766, 0, 399.5, 0, 654.4783072499766, 239.5, 0, 0, 1);
	//cameraMatrix = (Mat_<float>(3, 3) << 654.4783072499766, 0, 399.5, 0, 654.4783072499766, 239.5, 0, 0, 1);
	//distortionCoeff = (Mat_<float>(5,1) << 0.06774779384748693, -0.2183090452862961, 0, 0, -0.04145724673841295);
	cout << cameraMatrix << endl;
	headLocation = Point3f(0.0, 0.0, 0.0);

	cout << "Initializing camera inputs..." << endl;
	boost::shared_ptr<CVVideoInput> deviceVideo = boost::shared_ptr<CVVideoInput>(new CVVideoInput("/Users/john/Documents/Old\ School/2014\ Fall/Research/videos/video2.mp4"));
	boost::shared_ptr<ONIVideoInput> externalVideo = boost::shared_ptr<ONIVideoInput>(new ONIVideoInput("/Users/john/Documents/Old\ School/2014\ Fall/Research/videos/record1.oni", 400));
	//boost::shared_ptr<CVVideoInput> deviceVideo = boost::shared_ptr<CVVideoInput>(new CVVideoInput("video4.mp4", 0));
	//boost::shared_ptr<ONIVideoInput> externalVideo = boost::shared_ptr<ONIVideoInput>(new ONIVideoInput("first_person.oni", 0));
	//boost::shared_ptr<CVVideoInput> deviceVideo = boost::shared_ptr<CVVideoInput>(new CVVideoInput("video4.mp4", 0));
	//boost::shared_ptr<ONIVideoInput> externalVideo = boost::shared_ptr<ONIVideoInput>(new ONIVideoInput("Recording4.oni", 0));
	cout << "Camera inputs initialized" << endl;

	boost::shared_ptr<PointCloudWrapper> wrapper = boost::shared_ptr<PointCloudWrapper>(new PointCloudWrapper(externalVideo));

	// Initialize pose tracker
	externalVideo->getFirstDepthFrame(roomDepth);
	externalVideo->getFirstImageFrame(roomImage);
	Tracker tracker(roomImage, roomDepth, cameraMatrix, distortionCoeff, wrapper);
	cout << "Tracker initialized" << endl;

	// Initialized viewer
	Viewer viewer(tracker, "output.mp4", deviceVideo->getCodec(), false);

	// Iterate through frames in the two videos
	int frameCount = 0;
	while (deviceVideo->getNextImageFrame(deviceImage) && externalVideo->getNextDepthFrame(currentDepth) && externalVideo->getNextImageFrame(currentExternalImage)) {
		externalVideo->getNextUserHeadLocation(headLocation);
		tracker.computePosePnP(deviceImage, currentDepth, currentExternalImage, headLocation, R, t, (frameCount > 0));
		viewer.updateDisplay(R, t);
		cout << R << endl;
		cout << t << endl;
		frameCount++;
		char pressed;
		if (FRAME_BY_FRAME || frameCount == 1) {
			pressed = (char)waitKey(0);
		} else {
			pressed = (char)waitKey(10);
		}
		if (pressed == 'q' || pressed == 'Q') {
			return 0;
		}
	}


	return 0;
}
