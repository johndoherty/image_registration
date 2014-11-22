/*
 * External.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */


void findDepthKeyPoints(vector<KeyPoint> &depthKeypoints, Mat &depth, vector<Point3f> &objectPoints, pcl::RangeImagePlanar &rangeImage) {
	objectPoints.clear();
	for (int i = 0; i < depthKeypoints.size(); i++) {
		int x = (int)depthKeypoints[i].pt.x;
		int y = (int)depthKeypoints[i].pt.y;
		float range = depth.at<float>(y, x);
		Eigen::Vector3f worldPoint;
		rangeImage.calculate3DPoint((float)x, (float)y, range, worldPoint);
		Point3f newPoint = Point3f(worldPoint[0], worldPoint[1], worldPoint[2]);
		cout << worldPoint << endl;
		cout << newPoint << endl;
		cout << "X: " << x << ", Y: " << y << ", Range: " << range << endl;
		objectPoints.push_back(newPoint);
	}
}


void makeRangeImagePlanar(pcl::RangeImagePlanar &rangeImage, const Mat &depthImage, const ONIVideoInput &video) {
	Point2f focalLength = video.getFocalLength();
	Point2f centerOfProjection = video.getCenterOfProjection();
	rangeImage.setDepthImage(
			depthImage.ptr<float>(0, 0),
			depthImage.cols,
			depthImage.rows,
			centerOfProjection.x,
			centerOfProjection.y,
			focalLength.x,
			focalLength.y
	);
}
