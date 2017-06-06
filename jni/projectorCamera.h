/********************************************************************
Projection time:	2016/11/03   10:55
File Name: 	projectorCamera.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       projectorCamera system processing file
*********************************************************************/


#ifndef PROJECTOR_CAMERA
#define PROJECTOR_CAMERA

#include "Htime.h"
#include <Math.h>
#include <android/log.h>
#include "sendEvent.h"
#include <numeric>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include "role.h"
#include "inTheAirGesture.h"
#define PCL_NO_PRECOMPILE


#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//LOG DEFINITIONS
#define LOG_TAG_FLOW "ProjectorCamera/ProcessingFlow"
#define LOGF(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_FLOW, __VA_ARGS__))

#define LOG_TAG "ProjectorCamera/DebugInforcv::Mation"
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))

#define LOG_TAG_CALB "ProjectorCamera/CalibrationInforcv::Mation"
#define LOGC(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_CALB, __VA_ARGS__))

#define LOG_TAG_WARN "ProjectorCamera/WarningInforcv::Mation"
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_WARN, __VA_ARGS__))

#define SCREEN_WIDTH_MM  744
#define SCREEN_HEIGHT_MM 434
#define CAMERA_HEIGHT_MM 800
#define SCREEN_WIDTH_PX  1280
#define SCREEN_HEIGHT_PX 720
#define SALVER_WIDTH_MM 190
#define DEPTH_SURFACE 650


struct StereoProjection
{
	vector<cv::Point2f> cameraVertex;
	vector<cv::Point2f> projectorVertex;
	vector<cv::Point3f> proVertex3D;
	vector<float>   vertexDepth;
	vector<TouchHand> curtHands;
	vector<TouchHand> lastHands;
	cv::Point2f center;
	int lastId,first,last_bias;
	bool valid,boatDemo;
	StereoProjection() :valid(false),lastId(0),first(0){};
};

struct ObjectInfo{
    cv::Mat object;
    cv::Rect rec;
    float maxdepth,cArea;
    vector<float> angles;
    vector<cv::Point> contour;
};

class ProjectorCamera
{
public:
	ProjectorCamera();
	~ProjectorCamera();

	void init();
	void calibration();
	void calibrationFixed();
	void processing(cv::Mat& colorSrc, cv::Mat& depthSrc);
	bool isHistContinue(cv::Mat&);
	void getBgDepth();
	void getDynamicBgDepth();
	void getFgDepth();
	void getContour();
	void findFinger();
	void findOnDeskObject();
	void findInAirObject();
	void findInAirGesture();
	void refineRect(cv::Rect& rec, cv::Mat image);
	void refineFingerPosition(int);
	void transAxisCameraToPro();
	void fillDepthImageHole(cv::Mat &);
	void convertFloatToChar(cv::Mat& src, cv::Mat& dst);
	void fingerTipDepthDistribution(cv::Point2f&,float&,float&);
	int  intersectionPointsWithBoard(ObjectInfo&);
	cv::Rect palmRectangle(ObjectInfo&);
	void palmInfor(ObjectInfo&, TouchHand&);
	float contourAverageDepth(vector<cv::Point>&);
	float caclAngle(TouchHand&,int);
	float caclAngleForInAirGesture(cv::Point& left, cv::Point& center, cv::Point& right);
	string  intTostring(int);
    cv::Point2f fingerDirection(cv::Point2f, cv::Point2f, cv::Point2f);
	void calibDepToPro(vector<cv::Point2f>&, vector<float>&, vector<cv::Point3f>&);
	void calibDepToPro(vector<cv::Point2f>&, vector<float>&, vector<cv::Point2f>&);
	cv::Point2f calibDepToPro(cv::Point2f p_cam, float depth);
	void calibCamToPro(vector<cv::Point2f>&, vector<float>&, vector<cv::Point3f>&);
	void calibCamToPro(vector<cv::Point2f>&, vector<float>&, vector<cv::Point2f>&);
	cv::Point2f calibCamToPro(cv::Point2f p_cam, float depth);
	cv::Point2f homogCamToPro(cv::Point2f);	
	void refineVerticals(vector<cv::Point3f>&);
	void clockwiseContour(vector<cv::Point3f>&);
	void clockwiseContour(vector<cv::Point2f>&);
	void clockwiseContour(vector<cv::Point>&);

	//zhangbo
    bool isHandGraspOrOpen(double& handRatio, int& approxCurveSize, int& hullsize, int& littleangle, int& distance_x,int& distance_y, cv::Point& centerpoint);
    int handWaveState(vector<cv::Point>& pointsInLastFrames);
	cv::Point CalculateSumofPoints(cv::Point pointA, cv::Point pointB);
    void findConvexityDefects(vector<cv::Point>& contour, vector<int>& hull, vector<ConvexityDefect>& convexDefects);
    void inAirGestureManage(bool& openstate, int& handwave);


	//images 
	cv::Mat colorImg, depthImg, irImage, depthImg_old, binaryCopy;
	cv::Mat averaImg, foreground, foreground_store, foreground_copy, averaIR;		
	vector<cv::Mat> bgdepths;
	cv::Mat hist, homo;

	//cv::Rectangle
	cv::Rect screenRoi,filterRoi;

	//parameters
	bool calibrated, initBg, setViewDepth;
	size_t  frameId, initFrames;	

	float nearSurFaceThresh,nearCameraThresh;
	double maxdepth;
	int histMinPixs;

	//calibration
	int gridstart_x, gridstart_y,gridwidth;
	int offset_x,offset_y,offset_init_x,offset_init_y;

	//android state 
	int fd, appState;

	//stereo projection result
	StereoProjection stereoProjectDesk,stereoProjectHover;	

	//objects in foreground image
	vector<ObjectInfo> objects;

    //procamera calibration
    string cablibParaFilePath;
    cv::Mat colToProR, colToProT;
    cv::Mat depToColR, depToColT;
    cv::Mat camera_KK, camera_dis;
    cv::Mat project_KK, project_dis;
    cv::Mat depth_KK, depth_dis; 

    //homography
    cv::Mat depToColHomo;
    cv::Mat colToProHomo;
    cv::Mat depToProHomo;

    //state model role
    Role fingerTouchRole;

    //zhangbo
    //in air gesture variables
	int ncount_totalimg, ncount_grasp;
	int ncount_totalimg_open, ncount_open;
	vector<cv::Point>pointsInLastFrames;
	//InAirGestureInfo inairgesture;
	vector<bool> handgraspstates_temp;//hand grasp
    vector<int> handwavestates;//hand wave
    bool graspstate_temp;

	int lastframe;
	int lastframe_grasp;
	int lastframe_wave;
	
    bool m_graspstate;
    bool m_openstate;
    int m_wavestate;

	//output info-->for android
	bool graspstate_finnal;
	bool openstate_finnal;
	int wavestate_finnal;
	
	//dynamic background updating 
	vector<bool> ifBackground;
    int pixelInd;
    int fgNum, bgNum;
    vector<bool> isFingers;

     //for test
     vector<TouchPoint> touchPointsAll;
     int countid;
};
#endif