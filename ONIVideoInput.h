#ifndef ONI_VIDEO_INPUT_H_
#define ONI_VIDEO_INPUT_H_

#include <iostream> // for standard I/O
#include <string>   // for strings
#include "ImageInput.h"
#include "DepthInput.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <XnCppWrapper.h>

class ONIVideoInput: public ImageInput, public DepthInput {
    public:
		ONIVideoInput(std::string filename, int startFrame = 0);
        ~ONIVideoInput();

        int getCurrentFrameCount();
        void getFirstImageFrame(cv::Mat &firstFrame);
        void getFirstDepthFrame(cv::Mat &firstFrame);
        bool getNextImageFrame(cv::Mat &frame);
        bool getNextDepthFrame(cv::Mat &frame);
    private:
        cv::Mat firstImageFrame;
        cv::Mat firstDepthFrame;

        int currentFrameCount;
        int totalFrameCount;

        xn::Context context;
        xn::Player player;
        xn::ImageGenerator imageGen;
        xn::DepthGenerator depthGen;
        XnUInt32 frame_height;
        XnUInt32 frame_width;
};

#endif /* ONI_VIDEO_INPUT_H_ */

