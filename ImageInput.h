#ifndef CAMERA_INPUT_H_
#define CAMERA_INPUT_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class ImageInput {
public:
	// Next frame is copied to frame. Return value is false if the output is not valid.
    virtual bool getNextImageFrame(cv::Mat &frame) = 0;
};

#endif /* CAMERA_INPUT_H_ */
