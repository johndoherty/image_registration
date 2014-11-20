#include "CVVideoInput.h"

using namespace std;
using namespace cv;

CVVideoInput::CVVideoInput(std::string filename) {
	inputVideo = VideoCapture(filename);
	currentFrame = Mat();
	frameCount = 0;
    if (!inputVideo.isOpened()) {
		cout << "Could not open the input video: " << filename << endl;
	}
    inputVideo >> currentFrame;
    hasFrame = !currentFrame.empty();
	cout << filename << endl;
}

void CVVideoInput::getCurrentFrame(Mat &output) {
	currentFrame.copyTo(output);
	inputVideo >> currentFrame;
	if (currentFrame.empty()) {
		hasFrame = false;
	} else {
		hasFrame = true;
		resize(currentFrame, currentFrame, Size(0, 0), .5, .5);
		frameCount++;
	}
}

bool CVVideoInput::hasNextFrame() {
	return hasFrame;
}

int CVVideoInput::getCurrentFrameCount() {
	return frameCount;
}

CVVideoInput::~CVVideoInput() {
	currentFrame.release();
	inputVideo.release();
}


