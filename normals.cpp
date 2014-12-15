#include "normals.h"
#include <iostream>

using namespace cv;
using namespace std;

bool hasPrevColors = false;
vector<Vec3f> prevCentroids(3);
vector<Vec3b> prevColors(3);


void cvtMat33ToVecs3(Mat& m, vector<Vec3f>& v) {
	v.resize(3);
		v[0][0] = m.at<float>(0, 0);
		v[0][1] = m.at<float>(0, 1);
		v[0][2] = m.at<float>(0, 2);
		v[1][0] = m.at<float>(1, 0);
		v[1][1] = m.at<float>(1, 1);
		v[1][2] = m.at<float>(1, 2);
		v[2][0] = m.at<float>(2, 0);
		v[2][1] = m.at<float>(2, 1);
		v[2][2] = m.at<float>(2, 2);
}


void cluster(Mat& normals, Mat& dst) {
	int k = 3;
	Mat img;
	normals.copyTo(img);

	Mat samples(img.rows * img.cols, 3, CV_32F);
	for (int y = 0; y < img.rows; y++)
		for (int x = 0; x < img.cols; x++)
			for (int z = 0; z < 3; z++)
				samples.at<float>(y + x * img.rows, z) = img.at<Vec3f>(y, x)[z];

	Mat labels;
	int attempts = 4;
	Mat centers;
	kmeans(samples, k, labels,
			TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 0.001),
			attempts, KMEANS_PP_CENTERS, centers);

	dst.create(img.size(), dst.type());
	Vec3b red, green, blue;
	red[0] = 0; red[1] = 0; red[2] = 255;
	green[0] = 0; green[1] = 255; green[2] = 0;
	blue[0] = 255; blue[1] = 0; blue[2] = 0;
	vector<Vec3b> colors(3);
	colors[0] = red; colors[1] = green; colors[2] = blue;

	vector<Vec3f> centroids;
	cvtMat33ToVecs3(centers, centroids);

	
	if (hasPrevColors) {
		vector<int> idxs(0);
		for (int i = 0; i < 3; i++) {
			int minIdx = 0;
			float minDist = 100;
			for (int j = 0; j < 3; j++) {
				bool valid = true;
				for (int k = 0; k < idxs.size(); k++) {
					if (idxs[k] ==j) {
						valid = false;
					}
				}
				if (!valid) continue;
				float dist = norm(centroids[i], prevCentroids[j]);
				if (dist < minDist) {
					minDist = dist;
					minIdx = j;
				}
			}

			colors[i] = prevColors[minIdx];
			idxs.push_back(minIdx);
		}

	} else {
		hasPrevColors = true;
	}
	prevCentroids = centroids;
	prevColors = colors;


	for (int y = 0; y < img.rows; y++)
		for (int x = 0; x < img.cols; x++) {
			int cluster_idx = labels.at<int>(y + x * img.rows, 0);
			dst.at<Vec3b> (y, x) = colors[cluster_idx];
		}

}

void computeNormals(Mat& src, Mat& dst, bool performClustering, int k) {
	//assert(src.channels() == 1);
	Mat normals;
	normals.create(src.rows, src.cols, CV_32FC3);
	dst.create(src.rows, src.cols, CV_8UC3);
	Vec3f vec1, vec2, crossAvg;
	vector<Vec3f> cross(5);
	vec1[0] = 2;
	vec1[1] = 2;
	vec2[0] = 2;
	vec2[1] = -2;
	for (int row = 1; row < normals.rows-1; row++) {
		for (int col = 1; col < normals.cols-1; col++) {
			int k = 0;
			for (int i = 0; i < cross.size(); i++) {
				int offset = i*2 +1;
				if (row - offset < 0 || row + offset > normals.rows || 
						col - offset < 0 || col + offset > normals.cols)
					continue;
				k++;
				vec1[2] = src.at<uint16_t>(row+offset, col+offset) - src.at<uint16_t>(row-offset, col-offset);
				vec2[2] = src.at<uint16_t>(row+offset, col-offset) - src.at<uint16_t>(row-offset, col+offset);
				vec1[2] /= 10;
				vec2[2] /= 10;
				cross[i] = vec1.cross(vec2);
				cross[i] = cross[i] / norm(cross);
				//cout << cross[i][0] << " " << cross[i][1] << " " << cross[i][2] << " "<< endl;
				
				//getColorForNorm(cross[i],color);
				//normals.at<Vec3b>(row, col) = color;

			}

			crossAvg = cross[0];
			for (int j = 1; j < k; j++) {
				crossAvg += cross[j];
			}

			crossAvg = crossAvg / k;
			//crossAvg[2] = .1;
			crossAvg = crossAvg / norm(crossAvg);
			//if (crossAvg[0] < .2 && crossAvg[1] < .2)
			//	crossAvg[2] = 1;
			//crossAvg[2] = 1 - crossAvg[2];
			//crossAvg[2] = 1 - norm(crossAvg);

			//cout << crossAvg[0] << " " << crossAvg[1] << " " << crossAvg[2] << endl;

			normals.at<Vec3f>(row, col) = crossAvg;
			//const float scaleFactor = 0.05f;
			//normals.at<uint8_t>(row, col) = src.at<uint16_t>(row, col) * scaleFactor;
		}
	}
	Mat img;
	//normals.convertTo(dst, CV_8UC3, 255, 0);
	if (performClustering) {
		cluster(normals, dst);
	}

	//resize(normals, normals, Size(0, 0), .3, .3);
	//pyrMeanShiftFiltering(normals, normals, 50, 30);
	//morphologyEx(normals, normals, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(20, 20)));
	//resize(normals, normals, src.size());
	//cvtColor(normals, normals, CV_BGR2GRAY);
	//equalizeHist(normals, normals);
}
