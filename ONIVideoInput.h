#ifndef ONI_VIDEO_INPUT_H_
#define ONI_VIDEO_INPUT_H_

#include <iostream> // for standard I/O
#include <string>   // for strings
#include "CameraInput.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <XnCppWrapper.h>


class ONIVideoInput: public CameraInput {
    public:
		ONIVideoInput(std::string filename);
        ~ONIVideoInput();
        bool hasNextFrame();
        void getCurrentFrame(cv::Mat &frame);
        int getCurrentFrameCount();
    private:
        cv::VideoCapture inputVideo;
        cv::Mat currentFrame;
        int frameCount;
        bool hasFrame;
};

#endif /* ONI_VIDEO_INPUT_H_ */

