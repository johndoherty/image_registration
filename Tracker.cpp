/*
 * Tracker.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */


Tracker::Tracker() {

}


void keyPointMatches(Mat &trainImage, vector<KeyPoint> &trainKeypoints, Mat &queryImage, vector<KeyPoint> &queryKeypoints, vector<DMatch> &matches) {
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	Mat trainDescriptors, queryDescriptors;
	extractor.compute(trainImage, trainKeypoints, trainDescriptors);
	extractor.compute(queryImage, queryKeypoints, queryDescriptors);

	BFMatcher matcher;
	//FlannBasedMatcher matcher;
	vector<DMatch> initialMatches;
	matches.clear();
	//matcher.knnMatch(descriptors1, descriptors2, initialMatches, 2);
	matcher.match(queryDescriptors, trainDescriptors, matches);

	return;

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for(int i = 0; i < trainDescriptors.rows; i++) {
		double dist = initialMatches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.

	matches.clear();
	for(int i = 0; i < trainDescriptors.rows; i++) {
		if(initialMatches[i].distance <= max(1.5*min_dist, 0.02)) {
			matches.push_back(initialMatches[i]);
		}
	}
}



