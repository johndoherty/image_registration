/*
 * Viewer.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: john
 */

#include "Viewer.h"

using namespace std;
using namespace cv;

#define ONLY_INLIERS true
#define DEBUG true



void mouseEventOccured (const pcl::visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	std::cout << "s was pressed => saving camera angle as camera_params" << std::endl;
	viewer->saveCameraParameters("camera_params");
}

GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path){

	// Create the shaders
	GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
	GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

	// Read the Vertex Shader code from the file
	std::string VertexShaderCode;
	std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
	if(VertexShaderStream.is_open()) {
		std::string Line = "";
		while(getline(VertexShaderStream, Line))
			VertexShaderCode += "\n" + Line;
		VertexShaderStream.close();
	}

	// Read the Fragment Shader code from the file
	std::string FragmentShaderCode;
	std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
	if(FragmentShaderStream.is_open()) {
		std::string Line = "";
		while(getline(FragmentShaderStream, Line))
			FragmentShaderCode += "\n" + Line;
		FragmentShaderStream.close();
	}

	GLint Result = GL_FALSE;
	int InfoLogLength;

	// Compile Vertex Shader
	printf("Compiling shader : %s\n", vertex_file_path);
	char const * VertexSourcePointer = VertexShaderCode.c_str();
	glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
	glCompileShader(VertexShaderID);

	// Check Vertex Shader
	glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	std::vector<char> VertexShaderErrorMessage(InfoLogLength);
	glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
	fprintf(stdout, "%s\n", &VertexShaderErrorMessage[0]);

	// Compile Fragment Shader
	printf("Compiling shader : %s\n", fragment_file_path);
	char const * FragmentSourcePointer = FragmentShaderCode.c_str();
	glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
	glCompileShader(FragmentShaderID);

	// Check Fragment Shader
	glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	std::vector<char> FragmentShaderErrorMessage(InfoLogLength);
	glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
	fprintf(stdout, "%s\n", &FragmentShaderErrorMessage[0]);

	// Link the program
	fprintf(stdout, "Linking program\n");
	GLuint ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	std::vector<char> ProgramErrorMessage( max(InfoLogLength, int(1)) );
	glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
	fprintf(stdout, "%s\n", &ProgramErrorMessage[0]);

	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);

	return ProgramID;
}

// this function is called each frame
void Viewer::draw() {
	cout << "Drawing" << endl;
	//glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(programID);
	GLint loc = glGetUniformLocation(programID, "cameraPose");
	glUniformMatrix4fv(loc, 1, false, cameraPose);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 0.0, 0.0, 1.0, 9.0, 0.0, -1.0, 0.0);
	//gluLookAt(-3.0, 3.0, 4.0, 0.0, -3.0, 5.0, 0.0, -1.0, 0.0);

	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < tracker->roomPointCloud->size(); i++) {
		//glColor3ub(pointCloud[i].r, pointCloud[i].g, pointCloud[i].b);
		glVertex3d((*tracker->roomPointCloud)[i].x, (*tracker->roomPointCloud)[i].y, (*tracker->roomPointCloud)[i].z);
	}

	glEnd();
	glFlush();

	glutSwapBuffers();
	glPopMatrix();
	//glPopAttrib();
	waitKey(10);
}


void Viewer::glInit () {
	programID = LoadShaders("default.vert", "phong.frag");
}

Viewer::Viewer(Tracker &t) {
	programID = 0;
	tracker = &t;
	frame = 0;

	//if (DEBUG) {
		// Create viewer
		v0 = 0; v1 = 1;
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
		viewer->addPointCloud(tracker->roomPointCloud, "Room Point Cloud", v0);
		viewer->addPointCloud(tracker->segmentedPointCloud, "Segmented Room Point Cloud", v1);
		viewer->addCoordinateSystem(0.1);
		//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
		//viewer->registerMouseCallback(mouseEventOccured, (void*)&viewer);
		viewer->loadCameraParameters("camera_params");

		for (int i = 0; i < tracker->roomKeyLocation.size(); i++) {
			Scalar color(((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX));
			//Scalar color(.5, .3, .7);
			if (i < NUMBER_OF_COLORS) {
				colors[i] = color;
			}

			Point3f p = tracker->roomKeyLocation[i];
			viewer->addSphere(pcl::PointXYZ(p.x, p.y, p.z), 0.1, (double)color[0], (double)color[1], (double)color[2], to_string(i), v0);
		}
		numSpheres = tracker->roomKeyLocation.size();

		Point3f head = tracker->currentHeadLocation;
		viewer->addSphere(pcl::PointXYZ(head.x, head.y, head.z), 0.1, 0, 0, 255, "Head", v0);

		pcl::PointXYZ p(0, 0 , 0);
		viewer->addSphere(p, 0.1, "camera");

		Point3f pointInFrontOfCamera(0, 0, 1);
		viewer->addArrow(p, pcl::PointXYZ(pointInFrontOfCamera.x, pointInFrontOfCamera.y, pointInFrontOfCamera.z), 255, 0, 255, false, "head_direction");
		viewer->addSphere(pcl::PointXYZ(pointInFrontOfCamera.x, pointInFrontOfCamera.y, pointInFrontOfCamera.z), 0.05, 255, 0, 255, "direction_point");

		cvtColor(tracker->roomBwImage, roomImage, CV_GRAY2BGR);
		waitKey(1);
	//} else {
		cameraPose[0] = 1.0; cameraPose[1] = 0.0; cameraPose[2] = 0.0; cameraPose[3] = 0.0;
		cameraPose[4] = 0.0; cameraPose[5] = 1.0; cameraPose[6] = 0.0; cameraPose[7] = 0.0;
		cameraPose[8] = 0.0; cameraPose[9] = 0.0; cameraPose[10] = 1.0; cameraPose[11] = 0.0;
		cameraPose[12] = 0.0; cameraPose[13] = 0.0; cameraPose[14] = 0.0; cameraPose[15] = 1.0;
	//}
}

void Viewer::augmentImage(Mat &input, Mat &output, vector<KeyPoint>& keypoints, std::string text) {
	input.copyTo(output);
	vector<Point2f> points;
	points.clear();
	for (int i = 0; i < keypoints.size(); i++) {
		points.push_back(keypoints[i].pt);
	}
	augmentImage(input, output, points, text);
}

void Viewer::augmentImage(Mat &input, Mat &output, vector<Point2f>& keypoints, std::string text) {
	input.copyTo(output);
	putText(output, text, Point(10, input.rows - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255));
	for (int i = 0; i < keypoints.size(); i++) {
		Scalar color(255, 0, 0);
		if (i < NUMBER_OF_COLORS) {
			color = colors[i];
			float r = color[2] * 255; float g = color[1] * 255; float b = color[0] *255;
			color[0] = r; color[1] = g; color[2] = b;
		}
		circle(output, keypoints[i], 7, color, 3);
	}
}

void Viewer::makeViewableDepthImage(Mat &input, Mat &output) {
	const float scaleFactor = 0.05f;
	input.convertTo(output, CV_8UC1, scaleFactor);
}

void Viewer::updateDisplay(Mat R, Mat t) {
	Mat imageMatches, augmentedDeviceImage, viewableDepth, augmentedRoomImage;
	Mat center = R.t() * t;

	//if (DEBUG) {
		cout << "Device key points: " << tracker->deviceKeyPoints.size() << endl;
		cout << "Room key points: " << tracker->roomKeyPoints.size() << endl;
		cout << "Number of matches: " << tracker->matches.size() << endl;
		pcl::PointXYZ p;
		cout << center.at<double>(0) << ", " << center.at<double>(1) << ", " << center.at<double>(2) << endl;
		p.x = center.at<double>(0);
		p.y = center.at<double>(1);
		p.z = center.at<double>(2);
		Point3f head = tracker->currentHeadLocation;
		cout << "Head location: " << head << endl;
		viewer->updateSphere(pcl::PointXYZ(head.x, head.y, head.z), 0.1, 0, 0, 255, "Head");
		viewer->updateSphere(p, 0.1, 0, 255, 0, "camera");

		Mat pointInFrontOfCamera = (Mat_<double>(3,1) << 0, 0, 1);
		Mat output = R.t() * pointInFrontOfCamera;
		output = output + center;
		cout << "New point in front of camera: " << output << endl;
		viewer->removeShape("head_direction");
		viewer->addArrow(pcl::PointXYZ(output.at<double>(0), output.at<double>(1), output.at<double>(2)), p, 255, 0, 0, false, "direction");
		viewer->updateSphere(pcl::PointXYZ(output.at<double>(0), output.at<double>(1), output.at<double>(2)), 0.05, 255, 0, 255, "direction_point");

		//trans.col(3) = t;
		//cout << trans << endl;
		//pointInFrontOfCamera =

		//viewer->updateSphere(pcl::PointXYZ(pointInFrontOfCamera[0], pointInFrontOfCamera[1], pointInFrontOfCamera[2]), 0.1, 255, 255, 0, "ref_point", v0);


		for (int i = 0; i < numSpheres; i++) {
			viewer->removeShape(to_string(i));
		}
		numSpheres = 0;

		if (ONLY_INLIERS) {
			for (int i = 0; i < tracker->inlierIndexes.size(); i++) {
				Scalar color(255, 0, 0);
				if (i < NUMBER_OF_COLORS) {
					color = colors[i];
				}
				Point3f point = tracker->alignedWorldPoints[tracker->inlierIndexes[i]];
				viewer->addSphere(pcl::PointXYZ(point.x, point.y, point.z), 0.1, color[0], color[1], color[2], to_string(i), v0);
			}
			numSpheres = tracker->inlierIndexes.size();
		} else {
			for (int i = 0; i < tracker->alignedWorldPoints.size(); i++) {
				Scalar color(255, 0, 0);
				if (i < NUMBER_OF_COLORS) {
					color = colors[i];
				}
				Point3f point = tracker->alignedWorldPoints[i];
				viewer->addSphere(pcl::PointXYZ(point.x, point.y, point.z), 0.1, color[0], color[1], color[2], to_string(i), v0);
			}
			numSpheres = tracker->alignedDevicePoints.size();
		}

		viewer->loadCameraParameters("camera_params");
		for (int i = 0; i < tracker->matches.size(); i++) {
			/*if (tracker->matches[i].queryIdx >= tracker->deviceKeyPoints.size())
					cout << "Query index too large: " << tracker->matches[i].queryIdx << ", match: " << i << endl;
				if (tracker->matches[i].trainIdx >= tracker->roomKeyPoints.size())
					cout << "Train index too large: " << tracker->matches[i].trainIdx << ", match: " << i << endl;*/
		}

		if (!tracker->currentDepthImage.empty()) {
			makeViewableDepthImage(tracker->currentDepthImage, viewableDepth);
			imshow("Depth", viewableDepth);
		}

		drawMatches(tracker->deviceBwImage, tracker->deviceKeyPoints, tracker->roomBwImage, tracker->roomKeyPoints,
				tracker->matches, imageMatches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		//augmentImage(tracker->deviceBwImage, augmentedDeviceImage, deviceKeyPoints);

		cvtColor(tracker->deviceBwImage, augmentedDeviceImage, CV_GRAY2RGB);
		if (ONLY_INLIERS) {
			vector<Point2f> augmentPoints;
			for (int i = 0; i < tracker->inlierIndexes.size(); i++) {
				augmentPoints.push_back(tracker->alignedDevicePoints[tracker->inlierIndexes[i]]);
			}
			augmentImage(augmentedDeviceImage, augmentedDeviceImage, augmentPoints);
		} else {
			augmentImage(augmentedDeviceImage, augmentedDeviceImage, tracker->alignedDevicePoints);
		}

		imshow("Device Video", augmentedDeviceImage);
		imshow("Matches", imageMatches);
		waitKey(1);
	//} else {
		cameraPose[0] = (float) R.at<double>(0, 0); cameraPose[1] = (float) R.at<double>(0, 1); cameraPose[2] = (float) R.at<double>(0, 2); cameraPose[3] = (float) t.at<double>(0);
		cameraPose[4] = (float) R.at<double>(1, 0); cameraPose[5] = (float) R.at<double>(1, 1); cameraPose[6] = (float) R.at<double>(1, 2); cameraPose[7] = (float) t.at<double>(0);
		cameraPose[8] = (float) R.at<double>(2, 0); cameraPose[9] = (float) R.at<double>(2, 1); cameraPose[10] = (float) R.at<double>(2, 2); cameraPose[11] = (float) t.at<double>(0);
		cameraPose[12] = 0.0; cameraPose[13] = 0.0; cameraPose[14] = 0.0; cameraPose[15] = 1.0;

		draw();
	//}


	frame++;
}







