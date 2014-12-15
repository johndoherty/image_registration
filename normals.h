#ifndef NORMALS_H_
#define NORMALS_H_


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

void computeNormals(cv::Mat& src, cv::Mat& normals, bool cluster = false, int k = 5);

#endif /* NORMALS_H */
