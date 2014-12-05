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
#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

Mat roomDepth, roomImage, currentDepth, currentExternalImage, deviceImage, R, t;
Mat cameraMatrix, distortionCoeff;
Mat viewableRoomDepth, viewableRoomImage, viewableCurrentDepth, viewableCurrentExternalImage, viewableDeviceImage;
boost::shared_ptr<CVVideoInput> deviceVideo;
boost::shared_ptr<ONIVideoInput> externalVideo;
Point3f headLocation;
Tracker* tracker;
Viewer* viewer;


//Called when the window is resized
void handleResize(int w, int h) {
	//Tell OpenGL how to convert from coordinates to pixel values
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION); //Switch to setting the camera perspective

	//Set the camera perspective
	glLoadIdentity(); //Reset the camera
	gluPerspective(70.0,				  //The camera angle
			(double)w / (double)h, //The width-to-height ratio
			1.0,				   //The near z clipping coordinate
			200.0);				//The far z clipping coordinate
}

void glutDisplay (void) {
	if (!(deviceVideo->getNextImageFrame(deviceImage) && externalVideo->getNextDepthFrame(currentDepth) && externalVideo->getNextImageFrame(currentExternalImage))) {
		exit(1);
	}
	externalVideo->getNextUserHeadLocation(headLocation);
	tracker->computePosePnP(deviceImage, currentDepth, headLocation, R, t);
	viewer->updateDisplay(R, t);
	cout << R << endl;
	cout << t << endl;
	char pressed;

	/*if (FRAME_BY_FRAME) {
		pressed = (char)waitKey(0);
	} else {
		pressed = (char)waitKey(10);
	}
	if (pressed == 'q' || pressed == 'Q') {
		exit(1);
	}*/

}

void glutIdle (void) {
	// Display the frame
	glutPostRedisplay();
}


void glutKeyboard (unsigned char key, int /*x*/, int /*y*/) {
	cout << "Pressed: " << key << endl;
}


int main(int argc, char **argv) {
	float focal = 200;
	//cameraMatrix = (Mat_<float>(3, 3) << 654.4783072499766, 0, 399.5, 0, 654.4783072499766, 239.5, 0, 0, 1);
	cameraMatrix = (Mat_<float>(3, 3) << 460.6865232099297, 0, 399.5, 0, 460.6865232099297, 239.5, 0, 0, 1);
	distortionCoeff = Mat();
	//distortionCoeff = (Mat_<float>(5,1) << 0.06774779384748693, -0.2183090452862961, 0, 0, -0.04145724673841295);
	//distortionCoeff = (Mat_<float>(5,1) << -0.3198154897068323, 0.1391814322821029, 0, 0, -0.01962299901470377);
	cout << cameraMatrix << endl;
	headLocation = Point3f(0.0, 0.0, 0.0);

	cout << "Initializing camera inputs..." << endl;
	deviceVideo = boost::shared_ptr<CVVideoInput>(new CVVideoInput("/Users/john/Dropbox/School/Research/videos/video2.mp4"));
	externalVideo = boost::shared_ptr<ONIVideoInput>(new ONIVideoInput("/Users/john/Dropbox/School/Research/videos/record1.oni", 0));
	cout << "Camera inputs initialized" << endl;

	boost::shared_ptr<PointCloudWrapper> wrapper = boost::shared_ptr<PointCloudWrapper>(new PointCloudWrapper(externalVideo));

	externalVideo->getFirstDepthFrame(roomDepth);
	externalVideo->getFirstImageFrame(roomImage);

	tracker = new Tracker(roomImage, roomDepth, cameraMatrix, distortionCoeff, wrapper);
	cout << "Tracker initialized" << endl;

	viewer = new Viewer(*tracker);
	cout << "Viewer initialized" << endl;


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow("User Tracker Viewer");
	glutReshapeFunc(handleResize);
	glEnable(GL_DEPTH_TEST);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	viewer->glInit();

	glutMainLoop();


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
