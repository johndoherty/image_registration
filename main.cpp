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

using namespace std;
using namespace cv;

#define FRAME_BY_FRAME true

Mat roomDepth, roomImage, currentDepth, currentExternalImage, deviceImage, R, t;
Mat cameraMatrix;
Mat viewableRoomDepth, viewableRoomImage, viewableCurrentDepth, viewableCurrentExternalImage, viewableDeviceImage;

int main() {
	float focal = 200;
	cameraMatrix = (Mat_<float>(3, 3) << 658.46, 0, 399.5, 0, 658.46, 239.5, 0, 0, 1);

	cout << "Initializing camera inputs..." << endl;
	CVVideoInput deviceVideo = CVVideoInput("/Users/john/Dropbox/School/Research/videos/video2.mp4");
	ONIVideoInput externalVideo = ONIVideoInput("/Users/john/Dropbox/School/Research/videos/record1.oni", 0);
	cout << "Camera inputs initialized" << endl;

	externalVideo.getFirstDepthFrame(roomDepth);
	externalVideo.getFirstImageFrame(roomImage);

	Tracker tracker(roomImage, roomDepth, cameraMatrix, focal);
	cout << "Tracker initialized" << endl;

	Viewer viewer(tracker);

	waitKey(0);

	while (deviceVideo.getNextImageFrame(deviceImage) && externalVideo.getNextDepthFrame(currentDepth) && externalVideo.getNextImageFrame(currentExternalImage)) {
		tracker.computePosePnP(deviceImage, R, t);
		viewer.updateDisplay(R, t);
		cout << R << endl;
		cout << t << endl;
		char pressed;
		if (FRAME_BY_FRAME) {
			pressed = (char)waitKey(0);
		} else {
			pressed = (char)waitKey(10);
		}
		if (pressed == 'q' || pressed == 'Q') {
			return 0;
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
