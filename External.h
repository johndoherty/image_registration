/*
 * External.h
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#ifndef EXTERNAL_H_
#define EXTERNAL_H_

class External {
public:
	External(ImageInput &image, DepthInput &depth, cv::FeatureDetector &detector);
	void getCurrentImage(cv::Mat image);
	void getCurrentBwImage(cv::Mat bwImage);
	void getCurrentKeyPoints(std::vector<cv::KeyPoint> &keyPoints);
private:
	ImageInput *imageInput;
	DepthInput *depthInput;
	cv::Mat currentImage;
	cv::Mat currentBwImage;
	cv::FeatureDetector *featureDetector;
};



#endif /* EXTERNAL_H_ */
