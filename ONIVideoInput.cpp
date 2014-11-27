/*
 * ONIVideoInput.cpp
 *
 *  Created on: Nov 19, 2014
 *      Author: john
 */
#include "ONIVideoInput.h"

using namespace std;
using namespace cv;

#define THROW_IF_FAILED(retVal) {if (retVal != XN_STATUS_OK) {cout << xnGetStatusString(retVal); throw xnGetStatusString(retVal);}}

void XN_CALLBACK_TYPE newUserCallback(xn::UserGenerator& /*generator*/, XnUserID nId, void* void_video) {
	cout << "New user" << endl;
	static_cast<ONIVideoInput *>(void_video)->newUser(nId);
}

void XN_CALLBACK_TYPE lostUserCallback(xn::UserGenerator& /*generator*/, XnUserID nId, void* void_video) {
	cout << "Lost user" << endl;
	(static_cast<ONIVideoInput *>(void_video))->lostUser(nId);
}

void XN_CALLBACK_TYPE poseDetectedCallback(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* void_video) {
	cout << "Pose detected" << endl;
	(static_cast<ONIVideoInput *>(void_video))->poseDetected(strPose, nId);
}

void XN_CALLBACK_TYPE calibrationStartCallback(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* void_video) {
	cout << "Calibration start" << endl;
	(static_cast<ONIVideoInput *>(void_video))->calibrationStart(nId);
}

void XN_CALLBACK_TYPE calibrationCompleteCallback(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* void_video) {
	cout << "Calibration complete" << endl;
	(static_cast<ONIVideoInput *>(void_video))->calibrationComplete(nId, eStatus);
}


// Callback: New user was detected
void ONIVideoInput::newUser(XnUserID nId) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d New User %d\n", epochTime, nId);
	// New user found
	if (needPose) {
		userGen.GetPoseDetectionCap().StartPoseDetection(strPose, nId);
	} else {
		userGen.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE ONIVideoInput::lostUser(XnUserID nId) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Lost user %d\n", epochTime, nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE ONIVideoInput::poseDetected(const XnChar* strPose, XnUserID nId) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
	userGen.GetPoseDetectionCap().StopPoseDetection(nId);
	userGen.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE ONIVideoInput::calibrationStart(XnUserID nId) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Calibration started for user %d\n", epochTime, nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE ONIVideoInput::calibrationComplete(XnUserID nId, XnCalibrationStatus eStatus) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK) {
		// Calibration succeeded
		printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
		userGen.GetSkeletonCap().StartTracking(nId);
	} else {
		// Calibration failed
		printf("%d Calibration failed for user %d\n", epochTime, nId);
		if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT) {
			printf("Manual abort occured, stop attempting to calibrate!");
			return;
		} if (needPose) {
			userGen.GetPoseDetectionCap().StartPoseDetection(strPose, nId);
		} else {
			userGen.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

ONIVideoInput::ONIVideoInput(std::string filename, int startFrame) {
	supportsUserTracking = true;
	needPose = false;
	strcpy(strPose, "");
	THROW_IF_FAILED(context.Init());

	THROW_IF_FAILED(context.OpenFileRecording(filename.c_str(), player));
	THROW_IF_FAILED(player.SetRepeat(false));

	THROW_IF_FAILED(imageGen.Create(context));
	XnPixelFormat pixelFormat = imageGen.GetPixelFormat();
	if (pixelFormat != XN_PIXEL_FORMAT_RGB24) {
		THROW_IF_FAILED(imageGen.SetPixelFormat(XN_PIXEL_FORMAT_RGB24));
	}
	xn::ImageMetaData xImageMap2;
	imageGen.GetMetaData(xImageMap2);
	frame_height = xImageMap2.YRes();
	frame_width = xImageMap2.XRes();

	depthGen.Create(context);
	XnUInt32 nframes;
	player.GetNumFrames(depthGen.GetName(), nframes);
	totalFrameCount = nframes;

	xn::NodeInfo nin = depthGen.GetInfo();
	if ((XnNodeInfo*)nin == 0)
		throw "Could not read DepthGenerator info. Probably, the input file is corrupted";

	XnProductionNodeDescription description = nin.GetDescription();
	XnFieldOfView fov;
	depthGen.GetFieldOfView(fov);
	cout << "Field of view: " << fov.fHFOV << ", "<< fov.fVFOV << endl;

	XnStatus status = context.FindExistingNode(XN_NODE_TYPE_USER, userGen);
	if (status != XN_STATUS_OK) {
		THROW_IF_FAILED(userGen.Create(context));
	}
	if (!userGen.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		supportsUserTracking = false;
		printf("Supplied user generator doesn't support skeleton\n");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	THROW_IF_FAILED(userGen.RegisterUserCallbacks(newUserCallback, lostUserCallback, (void*)this, hUserCallbacks));
	THROW_IF_FAILED(userGen.GetSkeletonCap().RegisterToCalibrationStart(calibrationStartCallback, (void*)this, hCalibrationStart));
	THROW_IF_FAILED(userGen.GetSkeletonCap().RegisterToCalibrationComplete(calibrationCompleteCallback, (void*)this, hCalibrationComplete));

	if (userGen.GetSkeletonCap().NeedPoseForCalibration()) {
		needPose = true;
		if (!userGen.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			printf("Pose required, but not supported\n");
			supportsUserTracking = false;
		}
		THROW_IF_FAILED(userGen.GetPoseDetectionCap().RegisterToPoseDetected(poseDetectedCallback, (void*)this, hPoseDetected));
		userGen.GetSkeletonCap().GetCalibrationPose(strPose);

		//THROW_IF_FAILED(userGen.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress));
	}

	userGen.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	//THROW_IF_FAILED(userGen.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress));

	THROW_IF_FAILED(context.StartGeneratingAll());

	currentFrameCount = 0;

	firstImageFrame = Mat();
	firstDepthFrame = Mat();
	getNextImageFrame(firstImageFrame);
	getNextDepthFrame(firstDepthFrame);
	Mat test = Mat();
	Point3f testLocation;
	for (int i = 0; i < startFrame; i++) {
		getNextImageFrame(test);
		getNextDepthFrame(test);
		getNextUserHeadLocation(testLocation);
	}
}

ONIVideoInput::~ONIVideoInput() {
	context.StopGeneratingAll();
	context.Release();
}

void ONIVideoInput::getFirstImageFrame(Mat &output) {
	firstImageFrame.copyTo(output);
}

void ONIVideoInput::getFirstDepthFrame(Mat &output) {
	firstDepthFrame.copyTo(output);
}

bool ONIVideoInput::getNextImageFrame(Mat &output) {
	if (currentFrameCount >= totalFrameCount) {
		return false;
	}
	THROW_IF_FAILED(imageGen.WaitAndUpdateData());
	xn::ImageMetaData xImageMap;
	imageGen.GetMetaData(xImageMap);
	XnRGB24Pixel* imgData = const_cast<XnRGB24Pixel*>(xImageMap.RGB24Data());
	cv::Mat image(frame_height, frame_width, CV_8UC3, reinterpret_cast<void*>(imgData));
	cv::cvtColor(image, image, CV_BGR2RGB); // opencv image format is BGR

	if (!image.empty()) {
		image.copyTo(output);
		currentFrameCount++;
		return true;
	}
	return false;
}

bool ONIVideoInput::getNextDepthFrame(Mat &output) {
	THROW_IF_FAILED(depthGen.WaitAndUpdateData());
	xn::DepthMetaData xDepthMap;
	depthGen.GetMetaData(xDepthMap);
	XnDepthPixel* depthData = const_cast<XnDepthPixel*>(xDepthMap.Data());
	cv::Mat depth(frame_height, frame_width, CV_16U, reinterpret_cast<void*>(depthData));
	if (!depth.empty()) {
		depth.copyTo(output);
		return true;
	}
	return false;
}

bool ONIVideoInput::getNextUserHeadLocation(cv::Point3f &headLocation) {

	// TODO: make sure the coordinantates are correct
	THROW_IF_FAILED(userGen.WaitAndUpdateData());
	XnUserID users[15];
	XnUInt16 numUsers = 15;
	// updates numUsers
	userGen.GetUsers(users, numUsers);
	cout << "Number of users: " << numUsers << endl;
	if (numUsers > 0) {
		cout << "Checking capability" << endl;
		if (userGen.GetSkeletonCap().IsTracking(users[0]) && userGen.GetSkeletonCap().IsJointActive(XN_SKEL_HEAD)) {
			XnSkeletonJointPosition joint;
			userGen.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_HEAD, joint);
			cout << "Checking confidence" << endl;
			if (joint.fConfidence >= 0.5) {
				headLocation.x = joint.position.X * 0.001f;
				headLocation.y = joint.position.Y * 0.001f;
				headLocation.z = joint.position.Z * 0.001f;
				cout << "Head location: " << headLocation << endl;
				return true;
			}
		}
	}
	return false;
}

int ONIVideoInput::getCurrentFrameCount() {
	return currentFrameCount;
}

Point2f ONIVideoInput::getFocalLength() const{
	// TODO: finish this
	return Point2f(0, 0);
}

Point2f ONIVideoInput::getCenterOfProjection() const{
	// TODO
	return Point2f(0, 0);
}



