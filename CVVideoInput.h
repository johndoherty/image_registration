#ifndef CV_VIDEO_INPUT_H_
#define CV_VIDEO_INPUT_H_

#include <iostream> // for standard I/O
#include <string>   // for strings
#include "CameraInput.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CVVideoInput: public CameraInput {
    public:
        CVVideoInput(std::string filename);
        ~CVVideoInput();
        bool hasNextFrame();
        void getCurrentFrame(cv::Mat &frame);
        int getCurrentFrameCount();
    private:
        cv::VideoCapture inputVideo;
        cv::Mat currentFrame;
        int frameCount;
        bool hasFrame;
};

#endif /* CV_VIDEO_INPUT_H_ */
