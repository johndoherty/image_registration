#include "CVVideoInput.h"

using namespace std;
using namespace cv;

CVVideoInput::CVVideoInput(std::string filename, int startFrame) {
	inputVideo = VideoCapture(filename);
	frameCount = 0;
    if (!inputVideo.isOpened()) {
		cout << "Could not open the input video: " << filename << endl;
	}
	totalFrameCount = inputVideo.get(CV_CAP_PROP_FRAME_COUNT);
	Mat test = Mat();
	for (int i = 0; i < startFrame; i++) {
		getNextImageFrame(test);
	}
}

bool CVVideoInput::getNextImageFrame(Mat &output) {
	inputVideo >> output;
	if (!output.empty()) {
		resize(output, output, Size(0, 0), .5, .5);
		frameCount++;
		return true;
	}
	return false;
}

int CVVideoInput::getCurrentFrameCount() {
	return frameCount;
}

CVVideoInput::~CVVideoInput() {
	inputVideo.release();
}

int CVVideoInput::getCodec() {
	return static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
}


