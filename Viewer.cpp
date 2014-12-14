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
#define DEBUG false



void mouseEventOccured (const pcl::visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	std::cout << "s was pressed => saving camera angle as camera_params" << std::endl;
	viewer->saveCameraParameters("camera_params");
}

void windowResized(GLFWwindow *window, int width, int height) {
	cout << "Window resized: (" << width << ", " << height << ")" << endl;
	glfwMakeContextCurrent(window);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(50.0, (double)width / (double)height, 0.1, 400.0);
}

void error_callback(int error, const char* description) {
    cout << description << endl;
	//fputs(description, stderr);
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
void Viewer::draw(Mat &R, Mat &t) {
	if (glfwWindowShouldClose(window)) {
		glfwDestroyWindow(window);
		glfwTerminate();
		exit(EXIT_SUCCESS);
	}

	glfwMakeContextCurrent(window);
	//glPushAttrib(GL_ALL_ATTRIB_BITS);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(programID);

	//glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	//glEnable(GL_POINT_SMOOTH);
	// Setup the OpenGL viewpoint
	glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();
	//glLoadIdentity();
	//glTranslatef(0.0, -1.0, 0.0);
	//glRotatef(35.0, 0.0, 1.0, 0.0);
	//glTranslatef(2.0, 0.0, 0.0);
	//glRotatef(35.0, 0.0, 0.0, 1.0);
	//glRotatef(35.0, 1.0, 0.0, 0.0);
	//glTranslatef(0.0, 0.0, 2.0);

	int width; int height;
	glfwGetFramebufferSize(window, &width, &height);
	glm::mat4 projection = glm::perspective(45.0f, (float)width / (float)height, 0.1f, 400.0f);
	//gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, -1.0, 0.0);
	glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 0), glm::vec3(0, 0, 5), glm::vec3(0, -1, 0));
	glm::mat4 camera(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2),
			0.0, 0.0, 0.0, 1.0);
	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 modelView =  camera * view * model;
	//glm::mat4 modelView = view * model;
	cout << "Look at: " << endl;
	cout << projection[0][0] << ", " << projection[0][1] << ", " << projection[0][2] << ", " << projection[0][3] << endl;
	cout << projection[1][0] << ", " << projection[1][1] << ", " << projection[1][2] << ", " << projection[1][3] << endl;
	cout << projection[2][0] << ", " << projection[2][1] << ", " << projection[2][2] << ", " << projection[2][3] << endl;
	cout << projection[3][0] << ", " << projection[3][1] << ", " << projection[3][2] << ", " << projection[3][3] << endl;
	//
	//glTranslatef(t.at<double>(0), t.at<double>(1), t.at<double>(2));
	//glMultMatrixf(cameraPose);
	//gluLookAt(-3.0, 3.0, 4.0, 0.0, -3.0, 5.0, 0.0, -1.0, 0.0);

	GLint modelViewId = glGetUniformLocation(programID, "model_view");
	GLint projectionId = glGetUniformLocation(programID, "projection");
	glUniformMatrix4fv(modelViewId, 1, false, &modelView[0][0]);
	glUniformMatrix4fv(projectionId, 1, false, &projection[0][0]);

	glPointSize(3.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < tracker->roomPointCloud->size(); i+=1) {
		glColor3ub((*tracker->roomPointCloud)[i].r, (*tracker->roomPointCloud)[i].g, (*tracker->roomPointCloud)[i].b);
		glVertex3d((*tracker->roomPointCloud)[i].x, (*tracker->roomPointCloud)[i].y, (*tracker->roomPointCloud)[i].z);
	}

	/*glPointSize(5.0);
	for (int i =0; i < 200; i++) {
		glColor3ub(255.0, 0.0, 255.0);
		glVertex3d(1.0, 1.0, (float)i);
	}*/

	glEnd();

	glFlush();

	glfwSwapBuffers(window);
	//glfwWaitEvents();
	glfwPollEvents();
	glPopMatrix();
	//glPopAttrib();
}


Viewer::Viewer(Tracker &t) {
	programID = 0;
	tracker = &t;
	frame = 0;

	if (!glfwInit()) {
		cout << "Error loading glfw" << endl;
		exit(EXIT_FAILURE);
	}

	window = glfwCreateWindow(640, 480, "User video", NULL, NULL);
	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, windowResized);
	int width; int height;
	glfwGetFramebufferSize(window, &width, &height);
	windowResized(window, width, height);
	glfwSetErrorCallback(error_callback);
	programID = LoadShaders("default.vert", "phong.frag");
	glEnable(GL_DEPTH_TEST);

	if (DEBUG) {
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
		//viewer->addArrow(p, pcl::PointXYZ(pointInFrontOfCamera.x, pointInFrontOfCamera.y, pointInFrontOfCamera.z), 255, 0, 255, false, "head_direction");
		viewer->addSphere(pcl::PointXYZ(pointInFrontOfCamera.x, pointInFrontOfCamera.y, pointInFrontOfCamera.z), 0.05, 255, 0, 255, "direction_point");

		cvtColor(tracker->roomBwImage, roomImage, CV_GRAY2BGR);
		waitKey(1);
	} else {
		cameraPose[0] = 1.0; cameraPose[1] = 0.0; cameraPose[2] = 0.0; cameraPose[3] = 2.0;
		cameraPose[4] = 0.0; cameraPose[5] = 1.0; cameraPose[6] = 0.0; cameraPose[7] = 0.0;
		cameraPose[8] = 0.0; cameraPose[9] = 0.0; cameraPose[10] = 1.0; cameraPose[11] = 0.0;
		cameraPose[12] = 0.0; cameraPose[13] = 0.0; cameraPose[14] = 0.0; cameraPose[15] = 1.0;
	}
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
	Mat center = -R.t() * t;

	if (DEBUG) {
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

		Mat pointInFrontOfCamera = (Mat_<double>(3,1) << 0, 0, 0.4);
		Mat output = R.t() * pointInFrontOfCamera;
		output = output + center;
		cout << "New point in front of camera: " << output << endl;
		viewer->removeShape("head_direction");
		//viewer->addArrow(pcl::PointXYZ(output.at<double>(0), output.at<double>(1), output.at<double>(2)), p, 255, 0, 0, false, "direction");
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

		//viewer->loadCameraParameters("camera_params");
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
		//imwrite("matches.jpg", imageMatches);
		waitKey(1);
	} else {

		/*cameraPose[0] = (float) R.at<double>(0, 0); cameraPose[1] = (float) R.at<double>(0, 1); cameraPose[2] = (float) R.at<double>(0, 2); cameraPose[3] = (float) t.at<double>(0);
		cameraPose[4] = (float) R.at<double>(1, 0); cameraPose[5] = (float) R.at<double>(1, 1); cameraPose[6] = (float) R.at<double>(1, 2); cameraPose[7] = (float) t.at<double>(1);
		cameraPose[8] = (float) R.at<double>(2, 0); cameraPose[9] = (float) R.at<double>(2, 1); cameraPose[10] = (float) R.at<double>(2, 2); cameraPose[11] = (float) t.at<double>(2);
		cameraPose[12] = 0.0; cameraPose[13] = 0.0; cameraPose[14] = 0.0; cameraPose[15] = 1.0;*/
		cout << "Image size: " << tracker->roomBwImage.size() << endl;
		imshow("Device", tracker->deviceBwImage);
		imshow("External", tracker->roomBwImage);
		if (!tracker->roomDepth.empty()) {
			makeViewableDepthImage(tracker->roomDepth, viewableDepth);
			imshow("Depth", viewableDepth);
		}
		draw(R, t);
	}


	frame++;
}







