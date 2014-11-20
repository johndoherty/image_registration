#ifndef CAMERA_INPUT_H_
#define CAMERA_INPUT_H_

#include <opencv2/core/core.hpp>

class CameraInput {
public:
    virtual void getCurrentFrame(cv::Mat &frame) = 0;
    virtual int getCurrentFrameCount() = 0;
};

#endif /* CAMERA_INPUT_H_ */
