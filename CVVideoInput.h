#ifndef CV_VIDEO_INPUT_H_
#define CV_VIDEO_INPUT_H_

#include <iostream> // for standard I/O
#include <string>   // for strings
#include "ImageInput.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CVVideoInput: public ImageInput {
    public:
        CVVideoInput(std::string filename, int startFrame = 0);
        ~CVVideoInput();
        bool getNextImageFrame(cv::Mat &frame);
        int getCurrentFrameCount();
        int getCodec();
    private:
        cv::VideoCapture inputVideo;
        int totalFrameCount;
        int frameCount;
        bool hasFrame;
};

#endif /* CV_VIDEO_INPUT_H_ */
