#include <iostream> // for standard I/O
#include <string>   // for strings
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include "CVVideoInput.h"

using namespace std;
using namespace cv;

int main() {
	CVVideoInput deviceVideo = CVVideoInput("/Users/john/Dropbox/School/Research/videos/video1.mp4");
	Mat deviceFrame = Mat();


	while (deviceVideo.hasNextFrame()) {
		deviceVideo.getCurrentFrame(deviceFrame);
		imshow("Device Video", deviceFrame);
		waitKey(1);
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
