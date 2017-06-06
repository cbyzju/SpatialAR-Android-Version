/********************************************************************
Projection time:	2016/11/03   10:55
File Name: 	projectorCamera.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       projectorCamera system processing file
*********************************************************************/


#include "projectorCamera.h"
/*!
@function              sort points
@abstract              point comparison by location
@discussion
@param
@result
*/
bool cmpPoint(cv::Point2f& pt1, cv::Point2f& pt2)
{
	//high priority sort y
	if (abs(pt1.y - pt2.y) > 10)
	{
		return pt1.y <= pt2.y;
	}

	//default priority sort x
	return pt1.x <= pt2.x;
}

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
ProjectorCamera::ProjectorCamera()
{	
	LOGF("Processing flow : %s", "ProjectorCamera constructor start");
	calibrated = false;
	initBg     = false;
	setViewDepth = true;
	initFrames = 50;
	frameId    = 0;
	nearSurFaceThresh = 4;
	nearCameraThresh  = 1000;
	histMinPixs       = 15;

	//calibration
	gridstart_x = 25;
	gridstart_y = 47;
	gridwidth   = 50;
	offset_x    = 0;
	offset_y    = 0;

	//rectangle
	screenRoi   = cv::Rect(100, 100, 500, 300);
	filterRoi   = cv::Rect(48, 35, 40, 30);

	//zhangbo
	ncount_totalimg = 0;
    ncount_grasp = 0;

    ncount_totalimg_open = 0;
    ncount_open = 0;

    m_graspstate = false;
    m_openstate = false;
    m_wavestate = 0;

    graspstate_finnal = GRASPFAITTURE;
    openstate_finnal = OPENFAITURE;
    wavestate_finnal = NOWAVE;

	init();
	//for test
	remove("frame_test.txt");
    countid = 0;
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
ProjectorCamera::~ProjectorCamera()
{
    LOGF("Processing flow : %s", "ProjectorCamera destructor start");
}

/*!
@function
@abstract              initialization calibration parameters
@discussion
@param
@result
*/
void ProjectorCamera::init()
{
	//calibration result data	
	LOGD("try to open procamera_calib_paras.xml, %s", cablibParaFilePath.c_str());

	cv::FileStorage fsin;
	fsin.open("/sdcard/procamera_calib_paras.xml", cv::FileStorage::READ);
	if (!fsin.isOpened())
	{
		LOGD("failed to open file cascade_zfz.xml");
		return;
	}
	else
	{
		fsin["colToProR"] >> colToProR;
		fsin["colToProT"] >> colToProT;
		fsin["depToColR"] >> depToColR;
		fsin["depToColT"] >> depToColT;
		fsin["camera_KK"]  >> camera_KK;
		fsin["camera_dis"] >> camera_dis;
		fsin["project_KK"]  >> project_KK;
		fsin["project_dis"] >> project_dis;
		fsin["depth_KK"]  >> depth_KK;
		fsin["depth_dis"] >> depth_dis;
        fsin["depToColHomo"] >>depToColHomo;
        fsin["colToProHomo"] >>colToProHomo;
        fsin["depToProHomo"] >>depToProHomo;
        fsin["screenRoi"] >> screenRoi;
		LOGD("success to open file procamera_calib_paras.xml");
	}
	fsin.release();
}

/*!
@function              judge object depth distribution is continue or not
@abstract              just for finger touch area
@discussion
@param
@result
*/
bool ProjectorCamera::isHistContinue(cv::Mat& object)
{
	//LOGF("Processing flow : %s", "isHistContinue start");

	//init flag parameters
	bool iscontinue(false);
	size_t front(0), behind(20);

	//hist parameters setting
	int channel[1] = { 0 };
	const int histSize[1] = { 4 };
	float hranges[2] = { 5, 13 };
	const float* ranges[1] = { hranges };
	calcHist(&object, 1, channel, cv::Mat(), hist, 1, histSize, ranges);

	//checkout hist is continue or not
	for (int row = 1; row < hist.rows; row++)
	{
		if (hist.at<float>(row, 0) >= 5)
		{
			front = row;
		}
		else
		{
			break;
		}
	}

	return front >= hist.rows - 1;
}

/*!
@function              int->string
@abstract
@discussion
@param
@result
*/
string ProjectorCamera::intTostring(int num)
{
	stringstream ss;
	ss << num;
	string out;
	ss >> out;

	return out;
}

/*!
@function              using fixed calibration parameters
@abstract
@discussion
@param
@result
*/
void ProjectorCamera::calibrationFixed()
{
    LOGF("Processing flow : %s", "calibrationFixed start");

	//image setting
	homo = cv::Mat::eye(3, 3, CV_64F);
	homo = (cv::Mat_<double>(3, 3) <<
			2.5797362305397424e+000, -3.7274467612214901e-001,
			4.2250897290272846e+001, 5.4430909225976175e-003,
			2.5050769943386775e+000, -4.7408720715676232e+001,
			2.5509261309249496e-005, -4.8136826315280307e-004, 1.0);

	calibrated = true;
	fd = EVENT::opendev();

	LOGC("calibration stage : %s", "success");
}

/*!
@function              calibrate the homography between projector and camera
@abstract              homography between to plane is more accurate
@discussion
@param
@result
*/
void ProjectorCamera::calibration()
{
    LOGF("Processing flow : %s", "calibration start");
	//check input image
	if (colorImg.empty())
	{
		LOGW("Processing warn : %s", "calibration empty colorImg");
		return;
	}

    //image setting
    colorImg = colorImg(screenRoi);
	cv::Mat src  = colorImg,src_gray;
	cv::cvtColor(src, src_gray, 6);

	//Corner finding parameters
	double qualityLevel = 0.01;
	double minDistance = 14;
	int    blockSize = 3;
	double k = 0.04;
	bool useHarrisDetector = false;
	vector<cv::Point2f> corners;

	//Apply corner detection :Determines strong corners on an image.
	cv::goodFeaturesToTrack(src_gray,
		corners,
		400,
		qualityLevel,
		minDistance,
		cv::Mat(),
		blockSize,
		useHarrisDetector,
		k);

	if(corners.size()<100) return;

	//Draw corners detected
	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].x < filterRoi.x - corners[i].y / 35 ||
		    corners[i].y < filterRoi.y ||
            corners[i].x > src.cols - filterRoi.width + corners[i].y  / 35||
            corners[i].y > src.rows - filterRoi.height)
        {
			corners.erase(corners.begin() + i);
			i--;
			continue;
		}
		cv::circle(src, corners[i], 1, cv::Scalar(0, 0, 255));
	}//End all points

	//save corner points image
    imwrite("/sdcard/calibration.jpg", src);

	//sort corner points
	sort(corners.begin(), corners.end(), cmpPoint);

	//calculate the screen corresponding points
	vector<cv::Point2f> vscreenPoints;
	vscreenPoints.push_back(cv::Point2f(gridstart_x, gridstart_y));
	cv::Point2f lastRowStartPoint(corners[0]);
    for (int i = 1; i < corners.size(); i++)
    {
    	cv::Point2f last = vscreenPoints[i - 1];
    	if (corners[i].y - corners[i-1].y > 10)
    	{
    		int scalar = 0;
    		if (corners[i].x - lastRowStartPoint.x < -10) continue;
    		else if (corners[i].x - lastRowStartPoint.x < 10)
    		{
    			scalar = 0;
    			lastRowStartPoint.x = (lastRowStartPoint.x + corners[i].x) / 2;
    		}
    		else if (corners[i].x - lastRowStartPoint.x < 30)
    		{
    			scalar = 1;
    		}
    		else return;
    		last.x = gridstart_x + (gridwidth * scalar);
    		last.y += gridwidth;
    		vscreenPoints.push_back(last);
    	}
    	else
    	{
    		int scalar = 1;
    		if (corners[i].x - corners[i - 1].x > 30) scalar = 2;
    		if (corners[i].x - corners[i - 1].x > 50) scalar = 3;
    		last.x += gridwidth * scalar;
    		vscreenPoints.push_back(last);
    	}
    }

	//calculate homography
	if(corners.size()>100)
	{
	     homo = cv::findHomography(corners, vscreenPoints, cv::LMEDS);//LMEDS RANSAC
	     LOGC("calibration stage homography1: %f", homo.at<double>(0,0));
	     LOGC("calibration stage homography2: %f", homo.at<double>(0,1));
	     LOGC("calibration stage homography3: %f", homo.at<double>(0,2));
	     LOGC("calibration stage homography1: %f", homo.at<double>(1,0));
         LOGC("calibration stage homography2: %f", homo.at<double>(1,1));
         LOGC("calibration stage homography3: %f", homo.at<double>(1,2));
         LOGC("calibration stage homography1: %f", homo.at<double>(2,0));
         LOGC("calibration stage homography2: %f", homo.at<double>(2,1));
         LOGC("calibration stage homography3: %f", homo.at<double>(2,2));
	}
	else
	{
	     return;
	}

	//test homegraphy
	vector<cv::Point2f> testH(corners.size());
	cv::perspectiveTransform(corners, testH, homo);
	float maxsub(0);
	for (int i = 0; i < corners.size(); i++)
	{
		float sub = max(abs(vscreenPoints[i].x - testH[i].x),
		                abs(vscreenPoints[i].y - testH[i].y));
		if(maxsub < sub) maxsub = sub;
		if(maxsub > 6)
		{
			LOGC("calibration stage : %s", " failed");
			LOGC("calibration stage max distance: %f", maxsub);
			return;
		}
	}

	calibrated = true;
	fd = EVENT::opendev();

	LOGC("calibration stage : %s", "success");
    LOGC("calibration stage max distance: %f", maxsub);
}

/*!
@function              get background depth
@abstract              need to select valib depth value
@discussion
@param
@result
*/
void ProjectorCamera::getBgDepth()
{
	//average image
	if (frameId <= initFrames)
	{
		//accumulate depth image
		bgdepths.push_back(depthImg);
	}
	else if (frameId == initFrames + 1)
	{
	    averaImg = cv::Mat(depthImg.rows, depthImg.cols,CV_32F);
        //every pixel
        for (int row = 0; row < depthImg.rows; row++)
		{
			for (int col = 0; col < depthImg.cols; col++)
			{
			    //get valid depth data
				int  validnum = 0;
				float sum = 0;
				for (auto bg : bgdepths)
				{
					float depth = bg.at<float>(row, col);
					if (depth > 600)
					{
						validnum++;
						sum += depth;
					}
				}

                //get average depth
				if (validnum > 0)
				{
					averaImg.at<float>(row, col) = sum / validnum;
				}
				else
				{
					averaImg.at<float>(row, col) = 0;
				}
			}//end col
		}//end row

		//averaImg /= initFrames;
		initBg = true;
		nearCameraThresh = averaImg.at<float>(averaImg.rows / 2,averaImg.cols /2) - 250;

		LOGF("Processing flow : %s", "getBgDepth success");
	}
}

/*!
@function              get foreground object
@abstract              morphology operation is time consuming
@discussion
@param
@result
*/
void ProjectorCamera::getFgDepth()
{
    //LOGF("Processing flow : %s", "getFgDepth start");

	//get absolute depth image
	foreground = abs(depthImg - averaImg);

	threshold(foreground, foreground, nearSurFaceThresh, 255, CV_THRESH_TOZERO);

    //cvtColor(foreground,foreground_copy,8);
    //imwrite("/sdcard/foreground_copy.jpg",foreground_copy);
	//LOGD("nativeStart caught : %s", "getFgDepth success");
}

/*!
@function              main processing flow
@abstract
@discussion
@param
@result
*/
void ProjectorCamera::processing(cv::Mat& iRSrc, cv::Mat& depthSrc)
{
    //LOGF("Processing flow : %s", "processing start");
	CVTIME processingTotalTime,initTime;

	//check input image
	if (depthSrc.empty() || iRSrc.empty())
	{
		LOGW("Processing warn : %s", "empty depthImg");
		return;
	}

	if (depthSrc.rows != 480 || depthSrc.cols != 640 ||
	    iRSrc.rows != 480 || iRSrc.cols != 640)
    {
    	LOGW("Processing warn : %s", "error size depthImg");
    	return;
    }

    //depth image preprocessing
	frameId++;
	depthImg = depthSrc(screenRoi);
	irImage  = iRSrc(screenRoi);
	colorImg = irImage;
	depthImg.convertTo(depthImg, CV_32F);

	//1.get static background depth
	/*if (!initBg)
	{
	    getBgDepth();
	    return;
	}*/
    //LOGD("nativeStart caught getBgDepthTime: %f", getBg.getClock());
	//2.get dynamic background depth
	getDynamicBgDepth();
	if (frameId <= initFrames)
    {
    	LOGD("Initialize the Background!,frameID = %d",frameId);
        return;
    }
    //LOGD("nativeStart caught getDynamicBgDepthTime: %f", getBg.getClock());

	//get foreground depth
	CVTIME getFgDepthTime;
	getFgDepth();
	//LOGD("nativeStart caught getFgDepthTime: %f", getFgDepthTime.getClock());

    //get foreground
    CVTIME getContourTime;
    getContour();
    //LOGD("nativeStart caught getContourTime: %f", getContourTime.getClock());

    if(appState != 3)
    {
        //fingerTouchRole.appState = appState;
	    CVTIME findFingerTime;
	    findFinger();
	    //LOGD("nativeStart caught findFingerTime: %f", findFingerTime.getClock());
	}

    //LOGD("appState: %d", appState);
    if(appState == 1)
    {
        CVTIME findOnDeskObjectTime;
	    findOnDeskObject();
	    //findInAirObject();
	    LOGD("nativeStart caught cake findOnDeskObjectTime: %f", findOnDeskObjectTime.getClock());
	}
	else if(appState == 2)
	{
	    CVTIME findOnDeskObjectTime;
	    findOnDeskObject();
	    findInAirObject();
	    LOGD("nativeStart caught water findOnDeskObjectTime: %f", findOnDeskObjectTime.getClock());
	}
	else if(appState == 3)
	{
        CVTIME findInAirGestureTime;
        findInAirGesture();
        LOGD("nativeStart caught findInAirGestureTime: %f", findInAirGestureTime.getClock());
	}

    depthImg.release();
    depthSrc.release();
	//LOGF("Processing flow processingTotalTime: %f", processingTotalTime.getClock());
}

/*!vtouchHand_last
@function              find contour information in foreground image
@abstract			   using Object class keep contour information
@discussion
@param
@result
*/
void ProjectorCamera::getContour()
{
    //detect contours in foreground image
	cv::Mat fgImage,binaryIm;
	foreground.copyTo(fgImage);
	convertFloatToChar(fgImage,binaryIm);

	std::vector< std::vector<cv::Point> > contours;
	cv::findContours(binaryIm, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	binaryCopy = binaryIm;

    objects.clear();
    for (int i = 0; i < contours.size(); i++)
    {
        //get contour area
		vector<cv::Point> contour = contours[i];
		cv::Mat contourMat = cv::Mat(contour);
		double cArea = contourArea(contourMat);

        //min object area
        if (cArea < 300) continue;

        //fill hole
		cv::Rect rec = cv::boundingRect(contours[i]);
		cv::Mat object = foreground(rec);

        //exclude static object
		double mindepth;
		minMaxLoc(object, &mindepth, &maxdepth);

        ObjectInfo objectInfo;
        objectInfo.object   = object;
        objectInfo.maxdepth = maxdepth;
        objectInfo.contour  = contour;
        objectInfo.cArea    = cArea;
        objectInfo.rec	    = rec;

        objects.push_back(objectInfo);
    }
}

/*!
@function              calculate object intersection point with border
@abstract
@discussion
@param
@result
*/
int  ProjectorCamera::intersectionPointsWithBoard(ObjectInfo& object)
{
	//get DP points of the object
	TouchHand temHand;
	cv::approxPolyDP(cv::Mat(object.contour), temHand.approxCurve, 3, true);

	//palm intersection points with border
	int crossNum(0);
	for (int ind = 0; ind < temHand.approxCurve.size(); ind++)
	{
		cv::Point temPt = temHand.approxCurve[ind];
		if (temPt.x < 10 || depthImg.cols - temPt.x < 10 ||
			temPt.y < 10 || depthImg.rows - temPt.y < 10)
		{
			crossNum++;
		}
	}

	//LOGD("findFinger frameId%d crossNum%d",frameId,crossNum);
	return crossNum;
}

/*!
@function              calculate palm rectangle
@abstract
@discussion
@param
@result
*/
cv::Rect ProjectorCamera::palmRectangle(ObjectInfo& object)
{
    //LOGF("Processing flow : %s", "palmRectangle start");

	//paras
	cv::Rect boundingBox = object.rec;
	cv::Rect palmRec;
	cv::Size bound(depthImg.size());
	int thresh = 65;

	//left
	if (boundingBox.x == 1)
	{
		palmRec = cv::Rect(std::max(0, boundingBox.width - thresh),
			boundingBox.y, std::min(boundingBox.width, thresh), boundingBox.height);
	}
	//top
	if (boundingBox.y == 1)
	{
		palmRec = cv::Rect(boundingBox.x, std::max(0, boundingBox.height - thresh),
			boundingBox.width, std::min(boundingBox.height, thresh));
	}
	//right
	if (boundingBox.x + boundingBox.width == (bound.width - 1))
	{
		palmRec = cv::Rect(boundingBox.x, boundingBox.y,
			std::min(boundingBox.width, thresh), boundingBox.height);
	}
	//bottom
	if (boundingBox.y + boundingBox.height == (bound.height - 1))
	{
		palmRec = cv::Rect(boundingBox.x, boundingBox.y, boundingBox.width,
			std::min(boundingBox.height, thresh));
	}

	return palmRec;
}

/*!
@function              get plam center and radius
@abstract
@discussion
@param
@result
*/
void ProjectorCamera::palmInfor(ObjectInfo& object, TouchHand& curtHand)
{
    //LOGF("Processing flow : %s", "palmInfor start");

	//distanceTransformation
	cv::Mat disTransResult;
	cv::distanceTransform(binaryCopy(object.rec), disTransResult, CV_DIST_L2, 3);

	//palm center and radius
	cv::Rect palmRec = palmRectangle(object);
	double palmMinVal(-1), palmMaxVal(-1);
	cv::Point  palmMinLoc(-1, -1), palmMaxLoc(-1, -1);

	//adjust rectangle start point to object
	palmRec.x -= object.rec.x;
	palmRec.y -= object.rec.y;
	refineRect(palmRec, disTransResult);
	minMaxLoc(disTransResult(palmRec), &palmMinVal, &palmMaxVal, &palmMinLoc, &palmMaxLoc);

	//adjust rectangle start point to whole image
	palmMaxLoc.x += object.rec.x;
	palmMaxLoc.y += object.rec.y;

    //deal board situation
    float dis = depthImg.rows - palmMaxLoc.y;
    if(dis < 10) palmMaxVal = 20 ;

	//return information
	curtHand.palmCenter = palmMaxLoc;
	curtHand.palmRadius = palmMaxVal;
	curtHand.palmDepth  = foreground.at<float>(curtHand.palmCenter);
	curtHand.frameId    = frameId;
}

/*!
@function              fingertip depth distribution
@abstract
@discussion
@param
@result
*/
void ProjectorCamera::fingerTipDepthDistribution(cv::Point2f& fingerTip, float& percent, float& bfdis)
{
    //LOGF("Processing flow : %s %f %f", "fingerTipDepthDistribution start",fingerTip.x,fingerTip.y);

	//paras
	float foresum(0), backsum(0);
	int   backnum(0), tipnum(0),forenum(0);

	int srcPoint_x = fingerTip.x;
	int srcPoint_y = fingerTip.y;
	//calculate distribution
	for (int row = srcPoint_y - 5; row < srcPoint_y + 5; row++)
	{
		float * depthPt = (float*)depthImg.row(row).data;
		float * foregPt = (float*)foreground.row(row).data;
		float * averaPt = (float*)averaImg.row(row).data;
		for (int col = srcPoint_x- 5; col < srcPoint_x + 5; col++)
		{

			if (*(depthPt + col) < 0.1 || (*(averaPt + col) < *(depthPt + col)))
			{
				fingerTip += cv::Point2f(col, row);
				tipnum++;
				continue;
			}

			if (*(foregPt + col) > 0)
			{
				fingerTip += cv::Point2f(col, row);
				foresum += *(depthPt + col);
				forenum++;
				tipnum++;
			}
			else
			{
				backsum += *(depthPt + col);
				backnum++;
			}
		}
	}

	if (forenum > 0)
	{
		bfdis = -(foresum / forenum - backsum / backnum);
		fingerTip.x /= tipnum;
		fingerTip.y /= tipnum;
	}

    if(forenum + backnum > 0)
	    percent = 100 * forenum / (forenum + backnum);
	else
	    percent = 0;
	percent = forenum;
	//LOGF("Processing flow : %s", "fingerTipDepthDistribution end");
}

/*!
@function              detection finger from foreground image
@abstract			   also including predition
@discussion
@param
@result
*/
void ProjectorCamera::findFinger()
{
	//LOGF("Processing flow : %s", "findFinger start");

	//clear last touch information
	vector<TouchHand> vtouchHand_last;
	if (fingerTouchRole.histTouchHands.size() > 0)
	    vtouchHand_last = fingerTouchRole.histTouchHands[fingerTouchRole.histTouchHands.size() - 1];
	fingerTouchRole.curtTouchHands.clear();
	fingerTouchRole.curtFrameId = frameId;
	fingerTouchRole.linuxFileId = fd;
	fingerTouchRole.existPalm = false;
	stereoProjectHover.lastHands = stereoProjectHover.curtHands;
	stereoProjectHover.curtHands.clear();

	for (int ind = 0; ind < objects.size(); ind++)
	{
		//object intersection points with border
		int crossNum = intersectionPointsWithBoard(objects[ind]);
		if (crossNum < 2 || crossNum>6) continue;

		//approxPolyDP current object coutour
		TouchHand curtHand;
		cv::approxPolyDP(cv::Mat(objects[ind].contour), curtHand.approxCurve, 10, true);
		if (curtHand.approxCurve.size() < 3) continue;

		//get convex hull
		vector<int> hull;
		convexHull(cv::Mat(curtHand.approxCurve), hull, false, false);

		//get palm information
		palmInfor(objects[ind], curtHand);
        stereoProjectHover.curtHands.push_back(curtHand);

		//is palm real condition
		LOGD("nativeStart caught in findFinger touch palmRadius%f,palmDepth%f, x%d, y%d",
		curtHand.palmRadius,curtHand.palmDepth,curtHand.palmCenter.x,curtHand.palmCenter.y);
		if (curtHand.palmRadius < 20 || curtHand.palmRadius > 40 ||
		    curtHand.palmDepth  < 40 || curtHand.palmDepth > 180)
		{
			continue;
		}

		//give zhangbo for background refresh
		fingerTouchRole.existPalm = true;

		// assemble point set of convex hull
		bool existFinger(false);

		for (int pointInd = 0; pointInd < hull.size(); pointInd++)
		{
			//index is convex point
			int index = hull[pointInd];
			int Point_x = curtHand.approxCurve[index].x;
			int Point_y = curtHand.approxCurve[index].y;

			//boarder condition muster bigger than 5
			int skipWidth = 6;
			if (Point_x + skipWidth >= foreground.cols || Point_x - skipWidth <= 0 ||
				Point_y + skipWidth >= foreground.rows || Point_y - skipWidth <= 0)
				continue;

            //finger to palm condition
            float fingerRatio = norm(curtHand.approxCurve[index] - curtHand.palmCenter)/curtHand.palmRadius;
            LOGD("nativeStart caught in findFinger touch palm ratio: %d %f", frameId,fingerRatio);
            //border situation
            float minRatio(1.4),maxRatio(3.5);
            if(curtHand.palmCenter.y > depthImg.rows - 10) maxRatio = 4.2;
            if(fingerRatio < minRatio || fingerRatio > maxRatio)
                continue;

			//angle condition
			float angle = caclAngle(curtHand, index);
			LOGD("nativeStart caught in findFinger angle: %f", angle);
			if (angle > 70) continue;

			//finger roi
			int radius = 12;
			cv::Rect figerRec = cv::Rect(Point_x - radius, Point_y - radius, 2 * radius, 2 * radius);
			refineRect(figerRec, foreground);
			cv::Mat fingerRoi = foreground(figerRec);

            //string name = "/sdcard/bbb_" + intTostring(frameId) + ".png";
            //imwrite(name,depthImg - 700);

			//finger roi condition 1
			double mindepth, maxdepth(0);
			minMaxLoc(fingerRoi, &mindepth, &maxdepth);
			//if (maxdepth > 50 && maxdepth < 750) continue;
			LOGD("nativeStart caught in findFinger maxdepth: %f", maxdepth);

			//finger roi depth distribution
			cv::Point2f fingerTip = curtHand.approxCurve[index];
			float forenum(0),bfdis(1000);
			fingerTipDepthDistribution(fingerTip, forenum, bfdis);

			//finger roi depth distribution condition
			LOGD("nativeStart caught in findFinger bfdis: %f %f", bfdis, forenum);
			if (bfdis < 100 && fingerRatio > 2.3)  existFinger = true;
			if (bfdis > 6 || forenum < 17) continue;
			if (!isHistContinue(fingerRoi) && (curtHand.palmCenter.y < depthImg.rows - 15)) continue;

			//save touch points information
			TouchPoint touchPoint;
			touchPoint.tipPosition = curtHand.approxCurve[index];//fingerTip;//curtHand.approxCurve[index];
			touchPoint.tipDepth = averaImg.at<float>(touchPoint.tipPosition.y, touchPoint.tipPosition.x);
			touchPoint.bottomPosition = curtHand.palmCenter;
			touchPoint.bottomDepth = depthImg.at<float>(curtHand.palmCenter);
			touchPoint.getAngle();
			touchPoint.getOrien();
			touchPoint.palmToTip = touchPoint.tipPosition - cv::Point2f(curtHand.palmCenter.x, curtHand.palmCenter.y);
			touchPoint.frameId = frameId;
			touchPoint.isReal = true;
			//touchPoint.tipInPro = calibDepToPro(touchPoint.tipPosition, touchPoint.tipDepth);
			touchPoint.tipInPro = homogCamToPro(touchPoint.tipPosition);

                        //for test
                        LOGD("touchPointsAll.size() = %d",touchPointsAll.size());
                        if(touchPointsAll.size()==0)
                            touchPointsAll.push_back(touchPoint);
                        else if(touchPoint.frameId - touchPointsAll[touchPointsAll.size()-1].frameId <150 ){
                            touchPointsAll.push_back(touchPoint);
                        }
                        else{
                        if(touchPointsAll.size()>1){
                             int totalsize =  touchPointsAll[touchPointsAll.size()-1].frameId-  touchPointsAll[0].frameId+1;
                             int cursize  =  touchPointsAll.size();
                             int lostframes = totalsize- cursize;
                             float lostframesratio = lostframes*1.0/totalsize;
                             float speed =  (sqrt((touchPointsAll[touchPointsAll.size()-1].tipPosition.x-touchPointsAll[0].tipPosition.x)*(touchPointsAll[touchPointsAll.size()-1].tipPosition.x-touchPointsAll[0].tipPosition.x)+(touchPointsAll[touchPointsAll.size()-1].tipPosition.y-touchPointsAll[0].tipPosition.y)*(touchPointsAll[touchPointsAll.size()-1].tipPosition.y-touchPointsAll[0].tipPosition.y)))*1.0/(totalsize*1.0/30);
                             speed = speed/16.0; //cm/s.
                             LOGD("countid = %d, lostframesratio = %f,speed = %f,totalsize = %d,cursize = %d, startPoint = (%f,%f),endPoint = (%f,%f)",countid, lostframesratio,speed,totalsize,cursize,touchPointsAll[0].tipPosition.x,touchPointsAll[0].tipPosition.y,touchPointsAll[touchPointsAll.size()-1].tipPosition.x,touchPointsAll[touchPointsAll.size()-1].tipPosition.y);

                             countid++;
                             ofstream outfile;
                             outfile.open("/sdcard/frame_test.txt", ios::app);
                             if(!outfile)
                                 LOGD("no this file %d", frameId);
                              else
                             {
                                 //outfile<<"end id: "<<touchPointsAll[touchPointsAll.size()-1].frameId<<",start id: "<<touchPointsAll[0].frameId<<",totalsize: "<< totalsize<<",cursize: "<<cursize<<",lostframes: "<<lostframes<<",lostratio: "<<",lostframesratio"<<lostframesratio<<endl;
                                 //outfile<<countid<<" "<<touchPointsAll[0].frameId<<" "<<touchPointsAll[touchPointsAll.size()-1].frameId<<"  "<< totalsize<<" "<<cursize<<" "<<lostframes<<" "<<lostframesratio<<" "<<speed<<" ";
                                 outfile<<countid<<" "<< touchPointsAll[0].frameId<<" "<<touchPointsAll[touchPointsAll.size()-1].frameId<<" "<<totalsize<<" "<<cursize<<" ";
                                 //LOGD("ID = %f, total frame = %d, valid frame = %d", countid, totalsize, cursize);
                                 for(int kk=0; kk<touchPointsAll.size(); ++kk)
                                 {
                                      outfile<< touchPointsAll[kk].frameId<<" "<<touchPointsAll[kk].tipPosition.x<<" "<<touchPointsAll[kk].tipPosition.y<<" ";
                                      //float p_speed = (sqrt((touchPointsAll[kk+1].tipPosition.x-touchPointsAll[kk].tipPosition.x)*(touchPointsAll[kk+1].tipPosition.x-touchPointsAll[kk].tipPosition.x)+(touchPointsAll[kk+1].tipPosition.y-touchPointsAll[kk].tipPosition.y)*(touchPointsAll[kk+1].tipPosition.y-touchPointsAll[kk].tipPosition.y)))*1.0/((touchPointsAll[kk+1].frameId-touchPointsAll[kk].frameId)*1.0/30);
                                     // p_speed = p_speed/16.0; //cm/s
                                     // outfile<<p_speed<<" ";
                                 }
                                 outfile<<"\r\n";
                                 //outfile<<endl;
                             }
                              outfile.close();
                        }
                            touchPointsAll.clear();
                            touchPointsAll.push_back(touchPoint);
                        }

            			LOGD("nativeStart caught in findFinger detect touchPoint: %d %f %f %f %f %f %f/n", frameId,
            				touchPoint.tipPosition.x, touchPoint.tipPosition.y,
            				touchPoint.tipInPro.x, touchPoint.tipInPro.y, angle,fingerRatio);

            			curtHand.touchPoints.push_back(touchPoint);
            		}

            		if (curtHand.touchPoints.size()>0)
            		{
            		    LOGD("nativeStart caught in findFinger %d",curtHand.touchPoints.size());
            			fingerTouchRole.curtTouchHands.push_back(curtHand);
            		}

                    continue;//关闭补点操作。
            		int nearestHistHandInd = -1;
            		for (int handInd = 0; handInd < vtouchHand_last.size(); handInd++)
            		{
            			float palmDis = norm(vtouchHand_last[handInd].palmCenter - curtHand.palmCenter);

            			LOGD("nativeStart caught in findFinger insert touchPoint: %d firstId%d palmDis%f lastid%d oldDepth%f curDepth%f histSize%d curtSize%d",
            				frameId, fingerTouchRole.firstTouchId, palmDis, vtouchHand_last[handInd].frameId,
            				vtouchHand_last[handInd].palmDepth, foreground.at<float>(curtHand.palmCenter),
            				vtouchHand_last[handInd].touchPoints.size(), curtHand.touchPoints.size());

            			//static hand
                        if (existFinger) continue;

            			if (palmDis < 5 ||
            			    palmDis > 45 + 15* (frameId - vtouchHand_last[handInd].frameId) || //palm center is near
            				frameId - vtouchHand_last[handInd].frameId >= 3 ||                        //continue in 3 frames
            				curtHand.touchPoints.size() >= vtouchHand_last[handInd].touchPoints.size() || //exist history hand,who's touch points bigger than current hand
            				foreground.at<float>(curtHand.palmCenter) - vtouchHand_last[handInd].palmDepth > 15 || //hand palm depth not shape change
            				curtHand.palmCenter.y > depthImg.rows - 10)                              //near border not add
            			{
            				continue;
            			}
            			else
            			{
            				nearestHistHandInd = handInd;
            				break;
            			}
            		}

            		if (nearestHistHandInd < 0) continue;

            		TouchHand& corpHand = vtouchHand_last[nearestHistHandInd];
            		TouchHand  instHand;
            		for (int pointInd = 0; pointInd < corpHand.touchPoints.size(); pointInd++)
            		{
            			TouchPoint touchPoint = corpHand.touchPoints[pointInd];
            			touchPoint.tipPosition = touchPoint.palmToTip + cv::Point2f(curtHand.palmCenter.x, curtHand.palmCenter.y);
            			if(touchPoint.tipPosition.y < 0 || touchPoint.tipPosition.y > depthImg.rows - 1 ||
            			   touchPoint.tipPosition.x < 0 || touchPoint.tipPosition.x > depthImg.cols - 1)
            			   continue;
            			touchPoint.tipDepth = averaImg.at<float>(touchPoint.tipPosition.y, touchPoint.tipPosition.x);
            			touchPoint.bottomPosition = curtHand.palmCenter;
            			touchPoint.bottomDepth = depthImg.at<float>(curtHand.palmCenter);
            			touchPoint.getAngle();
            			touchPoint.getOrien();
            			touchPoint.palmToTip = touchPoint.tipPosition - cv::Point2f(curtHand.palmCenter.x, curtHand.palmCenter.y);
            			touchPoint.frameId = frameId;
            			touchPoint.isReal = false;
            			//touchPoint.tipInPro = calibDepToPro(touchPoint.tipPosition, touchPoint.tipDepth);
            			touchPoint.tipInPro = homogCamToPro(touchPoint.tipPosition);
            			instHand.touchPoints.push_back(touchPoint);

            			LOGD("nativeStart caught in findFinger insert touchPoint: %d %d %d %f %f/n", frameId,
            				touchPoint.tipPosition.x, touchPoint.tipPosition.y,
            				touchPoint.tipInPro.x, touchPoint.tipInPro.y);

            		}

                    //success insert touch points
                    if(instHand.touchPoints.size()>0)
                    {
            		    instHand.palmCenter = curtHand.palmCenter;
            		    instHand.palmDepth = corpHand.palmDepth;
            		    instHand.frameId = frameId;
            		    fingerTouchRole.curtTouchHands.push_back(instHand);
            		}

            	} // contour conditional

            	//transform position from camera to projector
            	//transAxisCameraToPro();
            	return;
            }

/*!
@function              detection projection above desk
@abstract			   in the air point depth is not stable
@discussion
@param
@result
*/
void ProjectorCamera::findInAirObject()
{
    //no boat on the desk
    if(stereoProjectDesk.valid == false) return;

    for(int ind=0;ind<stereoProjectHover.curtHands.size();ind++)
    {
        int scalar = 5;
        TouchHand& hand = stereoProjectHover.curtHands[ind];

        if(hand.palmRadius < 20 || hand.palmRadius > 100 ||
           hand.palmDepth > 480 || hand.palmDepth  < 50 ||
           hand.palmCenter.x < scalar || hand.palmCenter.x > depthImg.cols - scalar ||
           hand.palmCenter.y < scalar || hand.palmCenter.y > depthImg.rows - scalar)
        {
            stereoProjectHover.curtHands.erase(stereoProjectHover.curtHands.begin() + ind);
            ind--;
            continue;
        }
    }

    //only deal one hand
    if (stereoProjectHover.curtHands.size() > 1)
    {
        if(stereoProjectHover.lastHands.size() == 0)
        {
            stereoProjectHover.curtHands.erase(stereoProjectHover.curtHands.begin()+1,stereoProjectHover.curtHands.end());
        }
        else
        {
            //find nearest hand
            float minDis(1000000);
            int index(0);
            LOGD("stereoProjectHover curtHands size %d last %d",stereoProjectHover.curtHands.size(),stereoProjectHover.lastHands.size());
            for(int ind=0;ind<stereoProjectHover.curtHands.size();ind++)
            {
                float dis = norm(stereoProjectHover.lastHands[0].palmCenter - stereoProjectHover.curtHands[ind].palmCenter);
                if(dis<minDis)
                {
                    index  = ind;
                    minDis = dis;
                }
                LOGD("stereoProjectHover dis %f",dis);
            }
            vector<TouchHand> nearHands;
            nearHands.push_back(stereoProjectHover.curtHands[index]);
            stereoProjectHover.curtHands = nearHands;
        }
    }

    if(stereoProjectHover.curtHands.size() == 1)
    {
        //keep point stable
        if(stereoProjectHover.lastHands.size() == 1 &&
           norm(stereoProjectHover.lastHands[0].palmCenter - stereoProjectHover.curtHands[0].palmCenter) < 5)
        {
            stereoProjectHover.proVertex3D.erase(stereoProjectHover.proVertex3D.begin()+1,stereoProjectHover.proVertex3D.end());
        }
        else
        {
            stereoProjectHover.cameraVertex.clear();
            stereoProjectHover.vertexDepth.clear();
		    stereoProjectHover.cameraVertex.push_back(cv::Point2f(stereoProjectHover.curtHands[0].palmCenter.x,
		    stereoProjectHover.curtHands[0].palmCenter.y));
		    stereoProjectHover.vertexDepth.push_back(depthImg.at<float>(stereoProjectHover.curtHands[0].palmCenter));

            stereoProjectHover.proVertex3D.clear();
            calibDepToPro(stereoProjectHover.cameraVertex,stereoProjectHover.vertexDepth,stereoProjectHover.proVertex3D);
        }

    	struct PointCmp
    	{
    		PointCmp(cv::Point3f ct) : center(ct){};
    		cv::Point3f center;
    		bool operator () (cv::Point3f& a, cv::Point3f& b)
    		{
    			return (norm(a - center)<norm(b - center));
    		}
    	};

    	vector<cv::Point3f> deskPts(stereoProjectDesk.proVertex3D);
    	sort(deskPts.begin(),deskPts.end(),PointCmp(stereoProjectHover.proVertex3D[0]));

        stereoProjectHover.proVertex3D.push_back(stereoProjectHover.proVertex3D[0]);
        stereoProjectHover.proVertex3D.push_back(deskPts[0]);
        stereoProjectHover.proVertex3D.push_back(deskPts[1]);
        stereoProjectHover.valid = true;
    }
    return;
}

/*!
@function              detection projection on the desk
@abstract			   there is two condition
@discussion
@param
@result
*/
void ProjectorCamera::findOnDeskObject()
{
	float plate_max = 15000;  //constraint the plate area, upper bound for plate
	float plate_min = 10000;  //lower bound for plate
	float first_diff = 15;    //the diff tolerance between vpoints and icp
	float second_diff = 6;    //the diff tolerance between current frame and last frame
	//LOGF("Processing flow : %s", "findOnDeskObject start");

	for (int ind = 0; ind < objects.size(); ind++)
	{
	    //LOGD("Processing flow findOnDeskObject: object %d, area %f\n", ind, objects[ind].cArea);
		if (objects[ind].cArea<plate_min) continue;

		TouchHand temHand;
		cv::Mat contourMat = cv::Mat(objects[ind].contour);
		cv::approxPolyDP(contourMat, temHand.approxCurve, 15, true);
		bool boatDemo = false;

		cv::Point2f ppt[4];
		vector<cv::Point2f> vpoints(ppt, ppt + 4);
		cv::Scalar center;
		cv::Point centerPoint;
		//std::cout << "temHand.approxCurve.size(): "<<temHand.approxCurve.size() << endl;
#pragma region read camera parameters and store them by Eigen, it is convinient for calculation
		double cx = depth_KK.at<double>(0, 2);
		double cy = depth_KK.at<double>(1, 2);
		double fx = depth_KK.at<double>(0, 0);
		double fy = depth_KK.at<double>(1, 1);
		Eigen::Matrix<double, 3, 3> DepToColR;
		Eigen::Matrix<double, 3, 1> DepToColT;
		Eigen::Matrix<double, 3, 3> ColToProR;
		Eigen::Matrix<double, 3, 1> ColToProT;

		DepToColR << depToColR.at<double>(0, 0), depToColR.at<double>(0, 1), depToColR.at<double>(0, 2),
			         depToColR.at<double>(1, 0), depToColR.at<double>(1, 1), depToColR.at<double>(1, 2),
			         depToColR.at<double>(2, 0), depToColR.at<double>(2, 1), depToColR.at<double>(2, 2);
		DepToColT << depToColT.at<double>(0, 0), depToColT.at<double>(1, 0), depToColT.at<double>(2, 0);
		ColToProR << colToProR.at<double>(0, 0), colToProR.at<double>(0, 1), colToProR.at<double>(0, 2),
			         colToProR.at<double>(1, 0), colToProR.at<double>(1, 1), colToProR.at<double>(1, 2),
			         colToProR.at<double>(2, 0), colToProR.at<double>(2, 1), colToProR.at<double>(2, 2);
		ColToProT << colToProT.at<double>(0, 0), colToProT.at<double>(1, 0), colToProT.at<double>(2, 0);
#pragma endregion
#pragma region hand in the view, beside or on the plate
		if (temHand.approxCurve.size() != 4)                                                   //process the scenario that there are other objects in the scene other than plate.
		{
			if (stereoProjectDesk.lastId == 0) continue;                                      //do not project when we place the plate into the scene for the first time.

			first_diff = 20;                                                                   //the diff tolerance between vpoints and icp
			second_diff = 8;                                                                   //the diff between c
			cv::Mat plate, plateIm;
			cv::threshold(foreground, plate, 36, 255, CV_THRESH_TOZERO_INV);                        //segment hand from the foreground
			convertFloatToChar(plate, plateIm);                                                //used for findContours
			vector< vector<cv::Point> > contours;
			cv::findContours(plateIm, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);      //store all corner points in the contours.

			vector<cv::Point> corner;
			vector<cv::Point2f> tmp_vpoints;
			vector<cv::Point2f> refined;                                                     //useful points
			vector<cv::Point> rejected;                                                      //useless points
			vector<float> v_depth;                                                          //store the depth value of points.
			vector<cv::Point3f> vp_pro;                                                     //the 3D coordinate
			for (size_t index = 0; index < contours.size(); ++index)
			{
				double area = contourArea(contours[index]);                                //the area of the contours
				if (area < 5000)                                                           //don't care noise from small object
					continue;
				tmp_vpoints.clear();
				refined.clear();
				rejected.clear();
				approxPolyDP(contours[index], corner, 15, true);                          //用多边形去拟合轮廓，找到角点

#pragma region refine the corner points
				refined.push_back(corner[0]);
				for (size_t ind = 1; ind < corner.size(); ++ind)                          //refine the corner points
				{
					if (ind == corner.size() - 1)
					{
						if (norm(corner[ind] - corner[ind - 1])>90 && norm(corner[ind] - corner[ind - 1])<130 || norm(corner[ind] - corner[0])>90 && norm(corner[ind] - corner[0])<130)
							refined.push_back(corner[ind]);
						else
							rejected.push_back(corner[ind]);
					}
					else
					{
						if (norm(corner[ind] - corner[ind - 1])>90 && norm(corner[ind] - corner[ind - 1])<130 || norm(corner[ind] - corner[ind + 1])>90 && norm(corner[ind] - corner[ind + 1])<130)
							refined.push_back(corner[ind]);
						else
							rejected.push_back(corner[ind]);
					}
				}
				if (refined.size() < 3)
					continue;

				if (refined.size() == 3)
				{
					first_diff = 20;    //the diff tolerance between vpoints and icp
					second_diff = 15;
					clockwiseContour(refined);
					double val_1 = abs((refined[0].x - refined[1].x)*(refined[0].x - refined[2].x) + (refined[0].y - refined[1].y)*(refined[0].y - refined[2].y));
					double val_2 = abs((refined[1].x - refined[0].x)*(refined[1].x - refined[2].x) + (refined[1].y - refined[0].y)*(refined[1].y - refined[2].y));
					double val_3 = abs((refined[2].x - refined[1].x)*(refined[2].x - refined[0].x) + (refined[2].y - refined[1].y)*(refined[2].y - refined[0].y));
					if (val_1<val_2 && val_1<val_3)
						refined.push_back(refined[1] + refined[2] - refined[0]);
					if (val_2<val_1 && val_2<val_3)
						refined.push_back(refined[0] + refined[2] - refined[1]);
					if (val_3<val_1 && val_3<val_2)
						refined.push_back(refined[0] + refined[1] - refined[2]);
				}
#pragma endregion
				cv::RotatedRect rorec = minAreaRect(refined);                      //寻找包含所有输入2D点的最小旋转矩形
				rorec.points(ppt);
				for (int ind = 0; ind < 4; ++ind)
					tmp_vpoints.push_back(ppt[ind]);
				clockwiseContour(tmp_vpoints);
				bool outofBoundary = false;  //check if the points are out of boundary or not.
				vector<float> d_val;
				vector<cv::Point3f> tmp_3D;
				for (int ind = 0; ind < tmp_vpoints.size(); ++ind)
				{
					if (tmp_vpoints[ind].x<1 || tmp_vpoints[ind].x>screenRoi.width - 1 || tmp_vpoints[ind].y<1 || tmp_vpoints[ind].y>screenRoi.height - 1)
					{
						outofBoundary = true;
						break;
					}
					d_val.push_back(averaImg.at<float>(tmp_vpoints[ind].y, tmp_vpoints[ind].x));
				}
				if (outofBoundary)
					continue;
				calibDepToPro(tmp_vpoints, d_val, tmp_3D);
				bool isNotPlate = false;  //to check this contour is a plate or not.
				if (abs(norm(tmp_3D[0] - tmp_3D[1])
					- norm(tmp_3D[0] - tmp_3D[3])) > 30 ||
					abs(norm(tmp_3D[2] - tmp_3D[1])
					- norm(tmp_3D[2] - tmp_3D[3])) > 30)
					isNotPlate = true;

				bool notmoved = false;
				if (isNotPlate)
				{
					vector<cv::Point3f> lastVertex3D = stereoProjectDesk.proVertex3D;
					Eigen::Matrix<double, 3, 4> pre_vpoints;
					pre_vpoints << lastVertex3D[0].x, lastVertex3D[1].x, lastVertex3D[2].x, lastVertex3D[3].x,
						           lastVertex3D[0].y, lastVertex3D[1].y, lastVertex3D[2].y, lastVertex3D[3].y,
						           lastVertex3D[0].z, lastVertex3D[1].z, lastVertex3D[2].z, lastVertex3D[3].z;

					for (size_t ii = 0; ii < 4; ++ii)
					{
						pre_vpoints(0, ii) = -pre_vpoints(0, ii);
						pre_vpoints.col(ii) = ColToProR.inverse()*(pre_vpoints.col(ii) - ColToProT);
						pre_vpoints.col(ii) = DepToColR.inverse()*(pre_vpoints.col(ii) - DepToColT);
						pre_vpoints(0, ii) = pre_vpoints(0, ii)*fx / pre_vpoints(2, ii) + cx - screenRoi.x;
						pre_vpoints(1, ii) = pre_vpoints(1, ii)*fy / pre_vpoints(2, ii) + cy - screenRoi.y;  //匹配完的点，转到图像坐标系下
					}
					for (int inx = 0; inx < tmp_vpoints.size(); ++inx)
					{
						for (int iny = 0; iny < 4; ++iny)
						{
							cv::Point2f pre(pre_vpoints(0, iny), pre_vpoints(1, iny));
							double distance = norm(tmp_vpoints[inx] - pre);
							if (distance < 16)
							{
								notmoved = true;
								break;
							}
						}
						if (notmoved)
						{
							//cout << "I am in notPlate-3" << endl;
							vpoints[0] = cv::Point2f(pre_vpoints(0, 0), pre_vpoints(1, 0));
							vpoints[1] = cv::Point2f(pre_vpoints(0, 1), pre_vpoints(1, 1));
							vpoints[2] = cv::Point2f(pre_vpoints(0, 2), pre_vpoints(1, 2));
							vpoints[3] = cv::Point2f(pre_vpoints(0, 3), pre_vpoints(1, 3));
							center = mean(vpoints);
							centerPoint = cv::Point(center.val[0], center.val[1]);
							vpoints.push_back(centerPoint);
							break;
						}
					}
				}
				if (notmoved)
					break;
				if (isNotPlate && !notmoved)
					continue;
				vpoints[0] = ppt[0];
				vpoints[1] = ppt[1];
				vpoints[2] = ppt[2];
				vpoints[3] = ppt[3];
				clockwiseContour(vpoints);
				center = mean(vpoints);
				centerPoint = cv::Point(center.val[0], center.val[1]);
				float centerDepth = foreground.at<float>(centerPoint.y, centerPoint.x);
				if (centerDepth < 15) boatDemo = true;
				vpoints.push_back(centerPoint);
			}
		}
#pragma endregion
#pragma region just plate
		else
		{
			if (objects[ind].cArea<plate_min || objects[ind].cArea>plate_max) continue; //to remove hand，which also has four verticals.
			center = mean(cv::Mat(temHand.approxCurve));
			centerPoint = cv::Point(center.val[0], center.val[1]);
			float centerDepth = foreground.at<float>(centerPoint.y, centerPoint.x);
			if (centerDepth < 15) boatDemo = true;
			if (centerDepth > 100) continue;
			//is a rectangle, to remove hand，which also has four verticals, and its area is between the boundary.
			if (abs(norm(temHand.approxCurve[0] - temHand.approxCurve[1])
				- norm(temHand.approxCurve[2] - temHand.approxCurve[3])) > 40 ||
				abs(norm(temHand.approxCurve[2] - temHand.approxCurve[1])
				- norm(temHand.approxCurve[0] - temHand.approxCurve[3])) > 40)
				continue;

			first_diff = 15;    //the diff tolerance between vpoints and icp
			second_diff = 6;

			//get four verticals
			cv::RotatedRect rorec = minAreaRect(objects[ind].contour);
			rorec.points(ppt);
			vpoints[0] = temHand.approxCurve[0];
			vpoints[1] = temHand.approxCurve[1];
			vpoints[2] = temHand.approxCurve[2];
			vpoints[3] = temHand.approxCurve[3];
			for (size_t index = 0; index < 4; ++index)
			{
				for (size_t ind = 0; ind < 4; ++ind)
				{
					if (norm(vpoints[index] - ppt[ind]) < 20)
						vpoints[index] = cv::Point2f((vpoints[index].x + ppt[ind].x) / 2, (vpoints[index].y + ppt[ind].y) / 2);
				}
			}
			clockwiseContour(vpoints);
			center = mean(vpoints);
			centerPoint = cv::Point(center.val[0], center.val[1]);
			vpoints.push_back(centerPoint);
		}
#pragma endregion

#pragma region check the points out of boundary or not
		bool outofBoundary = false;
		for (size_t k = 0; k < vpoints.size(); ++k) //图像像素坐标系下的点在相机坐标系下的物理坐标
		{
			if (vpoints[k].x<1 || vpoints[k].x>screenRoi.width - 1 || vpoints[k].y<1 || vpoints[k].y>screenRoi.height - 1)
			{
				outofBoundary = true;
				break;
			}
		}
		if (outofBoundary)
			continue;
#pragma endregion

#pragma region read the template and construct target point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>("/sdcard/five_n.pcd", *target) == -1)// open template point cloud, five_f
		{
			//PCL_ERROR("Couldn't read file target.pcd\n");
			return;
		}
#pragma endregion
#pragma region construct source point cloud, select four corner points and the center
		source->width = target->points.size();
		source->height = 1;
		source->is_dense = false;
		source->points.resize(source->width * source->height);
		Eigen::Matrix<double, 3, 1> Tmp;
		Eigen::Matrix<double, 3, 1> PointInCol;
		for (size_t k = 0; k < vpoints.size(); ++k) //图像像素坐标系下的点在相机坐标系下的物理坐标
		{
			double depthVal = averaImg.at<float>(vpoints[k].y, vpoints[k].x);
			Tmp(0, 0) = depthVal * (vpoints[k].x + screenRoi.x - cx)*1.0 / fx;
			Tmp(1, 0) = depthVal * (vpoints[k].y + screenRoi.y - cy)*1.0 / fy;
			Tmp(2, 0) = depthVal;
			PointInCol = DepToColR*Tmp + DepToColT;
			source->points[k].x = PointInCol(0, 0);
			source->points[k].y = PointInCol(1, 0);
			source->points[k].z = PointInCol(2, 0);
		}
#pragma endregion
		/*
#pragma region remove lens distortion
		//去除镜头的径向畸变
		for (size_t index = 0; index < vpoints.size(); ++index)
		{
			vpoints[index].x += screenRoi.x;
			vpoints[index].y += screenRoi.y;
		}
		//vector<cv::Point2f> point_in_camera_undised;
		cv::undistortPoints(vpoints, vpoints, depth_KK, depth_dis, cv::Mat::eye(3, 3, CV_64F), depth_KK);
		for (size_t mk = 0; mk < vpoints.size(); ++mk)
		{
			vpoints[mk].x -= screenRoi.x;
			vpoints[mk].y -= screenRoi.y;
		}
#pragma endregion
		*/
#pragma region icp registration
		size_t iterations = 200;
		//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setEuclideanFitnessEpsilon(1e-8);
		//.setTransformationEpsilon(1e-8);
		icp.setMaximumIterations(iterations);
		icp.setInputSource(source);
		icp.setInputTarget(target);
		icp.align(*result);
		Eigen::Matrix4f transformation = icp.getFinalTransformation(); // Obtain the transformation that aligned source to target

		Eigen::Matrix<double, 3, 3> RR;
		Eigen::Matrix<double, 3, 3> R_inv;
		Eigen::Matrix<double, 3, 1> TT;
		RR << transformation(0, 0), transformation(0, 1), transformation(0, 2),
			  transformation(1, 0), transformation(1, 1), transformation(1, 2),
			  transformation(2, 0), transformation(2, 1), transformation(2, 2);
		R_inv = RR.inverse();
		TT << transformation(0, 3), transformation(1, 3), transformation(2, 3);
#pragma endregion
#pragma region move the key points in template to the right place by RT matrix achieved by icp algorithm
		Eigen::Matrix<double, 3, 4> Corner;
		Eigen::Matrix<double, 3, 1> Point;
		for (size_t ii = 0; ii < 4; ++ii)
		{
			Point << target->points[ii].x, target->points[ii].y, target->points[ii].z;
			Corner.col(ii) = R_inv * (Point - TT);  //模板点在相机坐标系下的坐标
		}

		Eigen::Matrix<double, 3, 4> CornerInImage;
		for (size_t ii = 0; ii < 4; ++ii)
		{
			CornerInImage.col(ii) = DepToColR.inverse()*(Corner.col(ii) - DepToColT);
			CornerInImage(0, ii) = CornerInImage(0, ii)*fx / CornerInImage(2, ii) + cx- screenRoi.x;
			CornerInImage(1, ii) = CornerInImage(1, ii)*fy / CornerInImage(2, ii) + cy - screenRoi.y;  //模板点在图像坐标系下坐标
		}

		vector<cv::Point3f> reg(4);
		Eigen::Matrix<double, 3, 4> CornerInPro;
		for (size_t index = 0; index < 4; ++index) //模板点在投影仪坐标系下的坐标
		{
			CornerInPro.col(index) = ColToProR*Corner.col(index) + ColToProT;
			CornerInPro(0, index) = -CornerInPro(0, index);
		}
		reg[0] = cv::Point3f(CornerInPro(0, 0), CornerInPro(1, 0), CornerInPro(2, 0));
		reg[1] = cv::Point3f(CornerInPro(0, 3), CornerInPro(1, 3), CornerInPro(2, 3));
		reg[2] = cv::Point3f(CornerInPro(0, 1), CornerInPro(1, 1), CornerInPro(2, 1));
		reg[3] = cv::Point3f(CornerInPro(0, 2), CornerInPro(1, 2), CornerInPro(2, 2));

#pragma endregion
#pragma region keep stable
		//（1）匹配失败处理：如果匹配完的点与vpoints距离不大，就用匹配完的点（更精确）；否则使用vpoints（匹配失败）。
		float max_dis_1 = 0;
		for (size_t mm = 0; mm < 4; ++mm)
		{
			float min_dis = 10000;
			for (size_t nn = 0; nn < 4; ++nn)
			{
				float dis = sqrt((CornerInImage(0, mm) - vpoints[nn].x)*(CornerInImage(0, mm) - vpoints[nn].x) + (CornerInImage(1, mm) - vpoints[nn].y)*(CornerInImage(1, mm) - vpoints[nn].y));
				if (dis < min_dis)
					 min_dis = dis;
			}
			if (min_dis > max_dis_1)
				max_dis_1 = min_dis; //匹配出的4个点和检测出的4个点的最大距离
		}
		vector<cv::Point3f> output_1;
		vector<cv::Point2f> vpoints_four;
		vector<float>  vertexDepth;
		if (max_dis_1 > first_diff)
		{
			for (size_t index = 0; index < 4; ++index)
			{
				vpoints_four.push_back(vpoints[index]);
				vertexDepth.push_back(averaImg.at<float>(vpoints[index].y, vpoints[index].x)); //at有点问题
			}

			calibDepToPro(vpoints_four, vertexDepth, output_1);  //如果距离过大，使用vpoints在投影仪坐标系下的坐标
			refineVerticals(output_1);
			second_diff = 8; //vpoints的波动比较大
		}
		else
		{
			output_1 = reg;  //否则使用icp匹配出的点坐标
		}

		//（2）防抖处理：如果（1）输出的点与上一次的点距离不大，就用上一次的点（防抖动）；否则使用本次的点（物体被移动）。
		float bias = 0;    //y方向的系统偏差
		float dep_bias = 0; //深度方向的系统偏差
		vector<cv::Point3f> output_2(4);
		if (stereoProjectDesk.lastId == 0)
		{
			output_2 = output_1;
			stereoProjectDesk.proVertex3D = output_2;
			for (size_t index = 0; index <stereoProjectDesk.proVertex3D.size(); ++index)
			{
				stereoProjectDesk.proVertex3D[index].y -= bias;
				stereoProjectDesk.proVertex3D[index].z -= dep_bias;
			}
			stereoProjectDesk.lastId++;
			stereoProjectDesk.valid = true;
			continue;
		}

		stereoProjectDesk.lastId++;

		for (size_t index = 0; index <stereoProjectDesk.proVertex3D.size(); ++index)
		{
			stereoProjectDesk.proVertex3D[index].y += bias;
			stereoProjectDesk.proVertex3D[index].z += dep_bias;
		}
		vector<cv::Point3f> lastVertex3D = stereoProjectDesk.proVertex3D;
		stereoProjectDesk.proVertex3D.clear();

		float max_dis_2 = 0;
		for (size_t mm = 0; mm < output_1.size(); ++mm)
		{
			float min_dis = 10000;
			for (size_t nn = 0; nn < lastVertex3D.size(); ++nn)
			{
				float dis = sqrt((output_1[mm].x - lastVertex3D[nn].x)*(output_1[mm].x - lastVertex3D[nn].x) + (output_1[mm].y - lastVertex3D[nn].y)*(output_1[mm].y - lastVertex3D[nn].y));
				if (dis < min_dis)
					min_dis = dis;
			}
			if (min_dis > max_dis_2)
				max_dis_2 = min_dis;
		}

		if (max_dis_2 <= second_diff) // 0~6
		{
			output_2 = lastVertex3D;
		}
		else if (max_dis_2 > second_diff  && max_dis_2 < second_diff * 2) //6~10
		{
			for (size_t mm = 0; mm < 4; ++mm)
			{
				float min_dis = 1000;
				for (size_t nn = 0; nn < 4; ++nn)
				{
					float dis = sqrt((lastVertex3D[mm].x - output_1[nn].x)*(lastVertex3D[mm].x - output_1[nn].x) + (lastVertex3D[mm].y - output_1[nn].y)*(lastVertex3D[mm].y - output_1[nn].y));
					cout << dis << endl;
					if (dis < min_dis)
					{
						output_2[mm] = cv::Point3f((lastVertex3D[mm].x + output_1[nn].x) / 2, (lastVertex3D[mm].y + output_1[nn].y) / 2, (lastVertex3D[mm].z + output_1[nn].z) / 2);
						min_dis = dis;
					}

				}
			}
		}
		else // >10
		{
			output_2 = output_1;
		}

		clockwiseContour(output_2);  //counter clockwise the points
		for (size_t index = 0; index <output_2.size(); ++index)
		{
			output_2[index].y -= bias;
			output_2[index].z -= dep_bias;
		}
		stereoProjectDesk.proVertex3D = output_2;
		stereoProjectDesk.valid = true;
#pragma endregion
	}
}


/*!
@function              make sure contour points is colokwise
@abstract			   using multiplication cross
@discussion
@param
@result
*/
void ProjectorCamera::clockwiseContour(vector<cv::Point2f>& verticals)
{
    if(verticals.size()<3) return;

    cv::Mat allPoints(verticals);
    cv::Scalar center = mean(verticals);
    cv::Point2f O(center.val[0], center.val[1]);
    cv::Point2f a = verticals[0] - O;//o->a
    cv::Point2f b = verticals[1] - O;//o->a

    float hint = a.x*b.y - b.x*a.y;

    if(hint < 0) reverse(verticals.begin(),verticals.end());
}

/*!
@function              make sure contour points is counter clockwise
@abstract			   using multiplication cross
@discussion
@param
@result
*/
void ProjectorCamera::clockwiseContour(vector<cv::Point3f>& verticals)
{
	if (verticals.size()<3) return;

	cv::Scalar center = cv::mean(verticals);
	cv::Point2f O(center.val[0], center.val[1]);
	cv::Point2f first(verticals[0].x, verticals[0].y);
	cv::Point2f second(verticals[1].x, verticals[1].y);
	cv::Point2f a = first - O;//o->a
	cv::Point2f b = second - O;//o->b

	//use vector cross product to judge counter clockwise orientation
	float hint = a.x*b.y - b.x*a.y;
	if (hint < 0) reverse(verticals.begin(), verticals.end());
}


/*!
@function              make sure contour points is colokwise
@abstract			   using multiplication cross
@discussion
@param
@result
*/
void ProjectorCamera::clockwiseContour(vector<cv::Point>& verticals)
{
    if(verticals.size()<3) return;

    cv::Mat allPoints(verticals);
    cv::Scalar center = mean(verticals);
    cv::Point2f O(center.val[0], center.val[1]);
    cv::Point2f x1(verticals[0].x, verticals[0].y);
    cv::Point2f x2(verticals[1].x, verticals[1].y);
    cv::Point2f a = x1 - O;//o->a
    cv::Point2f b = x2 - O;//o->a

    float hint = a.x*b.y - b.x*a.y;

    if(hint < 0) reverse(verticals.begin(),verticals.end());
}

/*!
@function              refine detection verticals according to real model
@abstract			   make diagonal line is same
@discussion
@param
@result
*/
void ProjectorCamera::refineVerticals(vector<cv::Point3f>& verticals)
{
    if(verticals.size() != 4) return;

    double realDiagonal = SALVER_WIDTH_MM * sqrt(2.0);

    for(int ind=0;ind<2;ind++)
    {
        double detectDiagonal = norm(verticals[ind] - verticals[ind+2]);
        double bias = (detectDiagonal - realDiagonal) / 2;

        cv::Point3f unitVector = verticals[ind+2] - verticals[ind];
        double L = sqrt(unitVector.x * unitVector.x + unitVector.y * unitVector.y + unitVector.z * unitVector.z);
        unitVector.x = unitVector.x / L;
        unitVector.y = unitVector.y / L;
        unitVector.z = unitVector.z / L;

        verticals[ind] = verticals[ind] + bias * unitVector;
        verticals[ind+2] = verticals[ind+2] - bias * unitVector;
        LOGD("refineVerticals be%f af%f",detectDiagonal,norm(verticals[ind] - verticals[ind+2]));
    }
}

/*!
@function              2d points in depth image to 3d points in projector axis
@abstract			   using depth image information
@discussion
@param
@result
*/
void ProjectorCamera::calibDepToPro(vector<cv::Point2f>& vp_cam, vector<float>& v_depth,vector<cv::Point3f>& vp_pro)
{
    //add rectangle bias, get point in src depth image
    vector<cv::Point2f>  projection = vp_cam;
    for(int ind=0;ind<vp_cam.size();ind++)
    {
        projection[ind] += cv::Point2f(screenRoi.x,screenRoi.y);
    }

	//undistored points in image
    vector<cv::Point2f> point_in_camera_undised;
	cv::undistortPoints(projection, point_in_camera_undised, depth_KK, depth_dis, cv::Mat::eye(3, 3, CV_64F), depth_KK);

	//image plane-> camera axis->projector axis
    vp_pro.clear();
    for(int ind=0;ind<point_in_camera_undised.size();ind++)
    {
        //depth image plane->depth camera axis
        cv::Point2f temPoint = point_in_camera_undised[ind];
        double cx = depth_KK.at<double>(0,2);
        double cy = depth_KK.at<double>(1,2);
        double fx = depth_KK.at<double>(0,0);
        double fy = depth_KK.at<double>(1,1);
        double Zc = v_depth[ind];
        double Xc = Zc * (temPoint.x - cx) / fx;
		double Yc = Zc * (temPoint.y - cy) / fy;
        //LOGD("stereo projection in the air proAxis,Xc%f Yc%f Zc%f", Xc,Yc,Zc);

        //depth axis->color camera axis
        cv::Point3d depAxis(Xc,Yc,Zc);
        cv::Mat camAxis = depToColR * cv::Mat(depAxis) + depToColT;

        //camera axis->projector axis
        cv::Mat out = colToProR * camAxis + colToProT;

		//add distortion
		//vector<cv::Point2f> v_imagePoints2d;
		//vector<cv::Point3f> v_worldPoints3d;
		//v_worldPoints3d.push_back(Point3f(camAxis));
		//projectPoints(Mat(v_worldPoints3d), colToProR, colToProT, project_KK, project_dis, v_imagePoints2d);

        cv::Point3f proAxis;
        double pfx = project_KK.at<double>(0,0);
        double pfy = project_KK.at<double>(1,1);
        double pcx = project_KK.at<double>(0,2);
        double pcy = project_KK.at<double>(1,2);

        //get 3d point in projector axis
        proAxis.x = -out.at<double>(0,0);
        proAxis.y = out.at<double>(1,0);
        proAxis.z = out.at<double>(2,0);
        vp_pro.push_back(proAxis);
        //LOGD("stereo projection in the air proAxis,fx%f fy%f cx%f cy%f bias%f",pfx,pfy,pcx,pcy,(pcx) * out.at<double>(2,0) / pfy);
    }
}

/*!
@function              get depth image point's corresponding point in projector
@abstract			   using depth image information
@discussion
@param
@result
*/
cv::Point2f ProjectorCamera::calibDepToPro(cv::Point2f p_cam, float depth)
{
	//add rectangle bias, get point in src depth image
    vector<cv::Point2f> vp_cam,point_in_camera_undised;
    p_cam = p_cam + cv::Point2f(screenRoi.x,screenRoi.y);
    vp_cam.push_back(p_cam);

	//undistored points in image
	cv::undistortPoints(vp_cam, point_in_camera_undised, depth_KK, depth_dis, cv::Mat::eye(3, 3, CV_64F), depth_KK);

    for(int ind=0;ind<point_in_camera_undised.size();ind++)
    {
        //depth image plane-> depth camera axis
        cv::Point2f temPoint = point_in_camera_undised[ind];
        double cx = depth_KK.at<double>(0,2);
        double cy = depth_KK.at<double>(1,2);
        double fx = depth_KK.at<double>(0,0);
        double fy = depth_KK.at<double>(1,1);
        double Zc = depth;
        double Xc = Zc * (temPoint.x  - cx) / fx;
		double Yc = Zc * (temPoint.y  - cy) / fy;

        //depth axis->color camera axis
        cv::Point3d depAxis(Xc,Yc,Zc);
        cv::Mat camAxis = depToColR *cv::Mat(depAxis) + depToColT;

        //camera axis->projector axis
        cv::Point3f camAxisPoint;
        camAxisPoint.x = camAxis.at<double>(0,0);
        camAxisPoint.y = camAxis.at<double>(1,0);
        camAxisPoint.z = camAxis.at<double>(2,0);

		//projector axis to projector plane
        vector<cv::Point2f> imagePoints2;
        vector<cv::Point3f> v_Rchess_in_world;
        v_Rchess_in_world.push_back(camAxisPoint);
        projectPoints(cv::Mat(v_Rchess_in_world), colToProR, colToProT, project_KK,  // project
		project_dis, imagePoints2);

		//flip
		imagePoints2[0].x = 1280 - imagePoints2[0].x;
		imagePoints2[0].y = 720  - imagePoints2[0].y;
        return imagePoints2[0];
    }
}

/*!
@function              get depth image point's corresponding point in projector
@abstract			   using depth image information
@discussion
@param
@result
*/
void ProjectorCamera::calibDepToPro(vector<cv::Point2f>& vp_cam, vector<float>& v_depth,vector<cv::Point2f>& vp_pro)
{
	//add rectangle bias, get point in src depth image
    for(int ind=0;ind<vp_cam.size();ind++)
    {
        vp_cam[ind] += cv::Point2f(screenRoi.x,screenRoi.y);
    }

	//undistored points in image
    vector<cv::Point2f> point_in_camera_undised;
	cv::undistortPoints(vp_cam, point_in_camera_undised, depth_KK, depth_dis, cv::Mat::eye(3, 3, CV_64F), depth_KK);

	//image plane-> camera axis->projector axis
    vp_pro.clear();
    for(int ind=0;ind<point_in_camera_undised.size();ind++)
    {
        //image plane-> camera axis
        cv::Point2f temPoint = point_in_camera_undised[ind];
        double cx = depth_KK.at<double>(0,2);
        double cy = depth_KK.at<double>(1,2);
        double fx = depth_KK.at<double>(0,0);
        double fy = depth_KK.at<double>(1,1);
        double Zc = v_depth[ind];
        double Xc = Zc * (temPoint.x  - cx) / fx;
		double Yc = Zc * (temPoint.y  - cy) / fy;

        //depth axis->color camera axis
        cv::Point3d depAxis(Xc,Yc,Zc);
        cv::Mat camAxis = depToColR * cv::Mat(depAxis) + depToColT;

        //camera axis->projector axis
        cv::Point3f camAxisPoint;
        camAxisPoint.x = camAxis.at<double>(0,0);
        camAxisPoint.y = camAxis.at<double>(1,0);
        camAxisPoint.z = camAxis.at<double>(2,0);

		//projector axis to projector plane
        vector<cv::Point2f> imagePoints2;
        vector<cv::Point3f> v_Rchess_in_world;
        v_Rchess_in_world.push_back(camAxisPoint);
        projectPoints(cv::Mat(v_Rchess_in_world), colToProR, colToProT, project_KK,
		project_dis, imagePoints2);

		//flip
		imagePoints2[0].x = 1280 - imagePoints2[0].x;
		imagePoints2[0].y = 720 - imagePoints2[0].y;
        vp_pro.push_back(imagePoints2[0]);
    }
}

/*!
@function              2d points in color image to 3d points in projector axis
@abstract			   using depth image information
@discussion
@param
@result
*/
void ProjectorCamera::calibCamToPro(vector<cv::Point2f>& vp_cam, vector<float>& v_depth, vector<cv::Point3f>& vp_pro)
{
	//add rectangle bias, get point in src depth image
	vector<cv::Point2f>  projection = vp_cam;
	for (int ind = 0; ind<vp_cam.size(); ind++)
	{
		projection[ind] += cv::Point2f(screenRoi.x, screenRoi.y);
	}

	//undistored points in image
	vector<cv::Point2f> point_in_camera_undised;
	cv::undistortPoints(projection, point_in_camera_undised, camera_KK, camera_dis, cv::Mat::eye(3, 3, CV_64F), camera_KK);

	//image plane-> camera axis->projector axis
	vp_pro.clear();
	for (int ind = 0; ind<point_in_camera_undised.size(); ind++)
	{
		//depth image plane->depth camera axis
		cv::Point2f temPoint = point_in_camera_undised[ind];
		double cx = camera_KK.at<double>(0, 2);
		double cy = camera_KK.at<double>(1, 2);
		double fx = camera_KK.at<double>(0, 0);
		double fy = camera_KK.at<double>(1, 1);
		double Zc = v_depth[ind];
		double Xc = Zc * (temPoint.x - cx) / fx;
		double Yc = Zc * (temPoint.y - cy) / fy;
		//LOGD("stereo projection in the air proAxis,Xc%f Yc%f Zc%f", Xc,Yc,Zc);

		//depth axis->color camera axis
		cv::Point3d camAxis(Xc, Yc, Zc);

		//camera axis->projector axis
		cv::Mat out = colToProR * cv::Mat(camAxis) + colToProT;

		//add distortion
		//vector<cv::Point2f> v_imagePoints2d;
		//vector<cv::Point3f> v_worldPoints3d;
		//v_worldPoints3d.push_back(Point3f(camAxis));
		//projectPoints(Mat(v_worldPoints3d), colToProR, colToProT, project_KK, project_dis, v_imagePoints2d);

		cv::Point3f proAxis;
		double pfx = project_KK.at<double>(0, 0);
		double pfy = project_KK.at<double>(1, 1);
		double pcx = project_KK.at<double>(0, 2);
		double pcy = project_KK.at<double>(1, 2);

		//get 3d point in projector axis
		proAxis.x = -out.at<double>(0, 0);
		proAxis.y = out.at<double>(1, 0);
		proAxis.z = out.at<double>(2, 0);
		vp_pro.push_back(proAxis);
		//LOGD("stereo projection in the air proAxis,fx%f fy%f cx%f cy%f bias%f",pfx,pfy,pcx,pcy,(pcx) * out.at<double>(2,0) / pfy);
	}
}

/*!
@function              get color image point's corresponding point in projector
@abstract			   using depth image information
@discussion
@param
@result
*/
cv::Point2f ProjectorCamera::calibCamToPro(cv::Point2f p_cam, float depth)
{
	//add rectangle bias, get point in src depth image
	vector<cv::Point2f> vp_cam, point_in_camera_undised;
	p_cam = p_cam + cv::Point2f(screenRoi.x, screenRoi.y);
	vp_cam.push_back(p_cam);

	//undistored points in image
	cv::undistortPoints(vp_cam, point_in_camera_undised, camera_KK, camera_dis, cv::Mat::eye(3, 3, CV_64F), camera_KK);

	for (int ind = 0; ind<point_in_camera_undised.size(); ind++)
	{
		//depth image plane-> depth camera axis
		cv::Point2f temPoint = point_in_camera_undised[ind];
		double cx = camera_KK.at<double>(0, 2);
		double cy = camera_KK.at<double>(1, 2);
		double fx = camera_KK.at<double>(0, 0);
		double fy = camera_KK.at<double>(1, 1);
		double Zc = depth;
		double Xc = Zc * (temPoint.x - cx) / fx;
		double Yc = Zc * (temPoint.y - cy) / fy;

		//camera axis->projector axis
		cv::Point3f camAxisPoint;
		camAxisPoint.x = Xc;
		camAxisPoint.y = Yc;
		camAxisPoint.z = Zc;

		//projector axis to projector plane
		vector<cv::Point2f> imagePoints2;
		vector<cv::Point3f> v_Rchess_in_world;
		v_Rchess_in_world.push_back(camAxisPoint);
		projectPoints(cv::Mat(v_Rchess_in_world), colToProR, colToProT, project_KK,  // project
			project_dis, imagePoints2);

		//flip
		imagePoints2[0].x = 1280 - imagePoints2[0].x;
		imagePoints2[0].y = 720 - imagePoints2[0].y;
		return imagePoints2[0];
	}
}

/*!
@function              get color image point's corresponding point in projector
@abstract			   using depth image information
@discussion
@param
@result
*/
void ProjectorCamera::calibCamToPro(vector<cv::Point2f>& vp_cam, vector<float>& v_depth, vector<cv::Point2f>& vp_pro)
{
	//add rectangle bias, get point in src depth image
	for (int ind = 0; ind<vp_cam.size(); ind++)
	{
		vp_cam[ind] += cv::Point2f(screenRoi.x, screenRoi.y);
	}

	//undistored points in image
	vector<cv::Point2f> point_in_camera_undised;
	cv::undistortPoints(vp_cam, point_in_camera_undised, camera_KK, camera_dis, cv::Mat::eye(3, 3, CV_64F), camera_KK);

	//image plane-> camera axis->projector axis
	vp_pro.clear();
	for (int ind = 0; ind<point_in_camera_undised.size(); ind++)
	{
		//image plane-> camera axis
		cv::Point2f temPoint = point_in_camera_undised[ind];
		double cx = camera_KK.at<double>(0, 2);
		double cy = camera_KK.at<double>(1, 2);
		double fx = camera_KK.at<double>(0, 0);
		double fy = camera_KK.at<double>(1, 1);
		double Zc = v_depth[ind];
		double Xc = Zc * (temPoint.x - cx) / fx;
		double Yc = Zc * (temPoint.y - cy) / fy;

		//camera axis->projector axis
		cv::Point3f camAxisPoint;
		camAxisPoint.x = Xc;
		camAxisPoint.y = Yc;
		camAxisPoint.z = Zc;

		//projector axis to projector plane
		vector<cv::Point2f> imagePoints2;
		vector<cv::Point3f> v_Rchess_in_world;
		v_Rchess_in_world.push_back(camAxisPoint);
		projectPoints(cv::Mat(v_Rchess_in_world), colToProR, colToProT, project_KK,
			project_dis, imagePoints2);

		//flip
		imagePoints2[0].x = 1280 - imagePoints2[0].x;
		imagePoints2[0].y = 720 - imagePoints2[0].y;
		vp_pro.push_back(imagePoints2[0]);
	}
}

/*!
@function              color point  / depth point to projector point
@abstract			   using homography
@discussion
@param
@result
*/

cv::Point2f ProjectorCamera::homogCamToPro(cv::Point2f camPoint)
{
	cv::Point2f proPoint;
	vector<cv::Point2f> v_camPoints, v_proPoints;
	v_camPoints.push_back(camPoint + cv::Point2f(screenRoi.x, screenRoi.y));

	//different start point
	if (!setViewDepth)
		perspectiveTransform(v_camPoints, v_proPoints, homo);
	else
		perspectiveTransform(v_camPoints, v_proPoints, depToProHomo);

	//flip
	proPoint = cv::Point2f(1280, 720) - v_proPoints[0];

	//offset
	proPoint = proPoint - cv::Point2f(offset_x,offset_y) - cv::Point2f(offset_init_x,offset_init_y);
	return proPoint;
}

/*!
@function              get depth image point's corresponding point in projector
@abstract			   using depth image information
@discussion
@param
@result
*/
void ProjectorCamera::transAxisCameraToPro()
{
}

/*!
@function              calculate angle constructed by three points
@abstract			   there points are near by in a contour
@discussion
@param
@result
*/
float  ProjectorCamera::caclAngle(TouchHand& curtHand, int ind)
{
	//get index
	int index = ind, leftind(0), rightind(0);
	if (index == 0)
	{
		leftind = curtHand.approxCurve.size() - 1;
		rightind = index + 1;
	}
	else if (index == curtHand.approxCurve.size() - 1)
	{
		leftind = index - 1;
		rightind = 0;
	}
	else
	{
		leftind = index - 1;
		rightind = index + 1;
	}

	//get point
	cv::Point left(curtHand.approxCurve[leftind]);
	cv::Point center(curtHand.approxCurve[index]);
	cv::Point right(curtHand.approxCurve[rightind]);

	//get orientation
	float angle(0);
	cv::Point lc = left  - center;
	cv::Point rc = right - center;

	//get angle
	float cos_angle = (lc.x * rc.x + lc.y*rc.y) / (norm(lc) * norm(rc));
	angle = acos(cos_angle);
	angle *= 180 / M_PI;

	return angle;
}

/*
*function: calculate angle constructed by three points
*note    :
*/
float  ProjectorCamera::caclAngleForInAirGesture(cv::Point& left, cv::Point& center, cv::Point& right)
{
	float angle(0);
	cv::Point lc = left - center;
	cv::Point rc = right - center;

	float cos_angle = (lc.x * rc.x + lc.y*rc.y) / (sqrtf(lc.x*lc.x + lc.y*lc.y) * sqrtf(rc.x*rc.x + rc.y *rc.y));

	angle = acos(cos_angle);

	angle *= 180 / M_PI;
	return angle;
}

/*!
@function              calculate the direction of finger tip
@abstract			   there points are near by in a contour
@discussion
@param
@result
*/
cv::Point2f ProjectorCamera::fingerDirection(cv::Point2f L, cv::Point2f M, cv::Point2f R)
{
	//make sure L nearer than R
	if (norm(L - M) > norm(R - M))
	{
		cv::Point2f tem = L;
		L = R;
		R = tem;
	}

	//calculate the L pojection point in MR
	float dis = norm(L - M);
	cv::Point2f Fdirection = R - M;
	float normtem = norm(Fdirection);
	Fdirection.x /= normtem;
	Fdirection.y /= normtem;
	cv::Point2f O = M + dis * Fdirection;
	O = (O + L)*0.5;

	return O;
}

/*!
@function              make sure rectangle do not beyond corresponding image
@abstract
@discussion
@param
@result
*/
void ProjectorCamera::refineRect(cv::Rect& rec, cv::Mat image)
{
	//min value
	if (rec.x < 0) rec.x = 0;
	if (rec.y < 0) rec.y = 0;

	//max value
	if (rec.x + rec.width  > image.cols) rec.width  = image.cols - rec.x;
	if (rec.y + rec.height > image.rows) rec.height = image.rows - rec.y;
}

/*!
@function              fill the holes in depth image
@abstract              using near by point depth
@discussion
@param
@result
*/
void ProjectorCamera::fillDepthImageHole(cv::Mat& object)
{
	//every row
	for (int row = 0; row < object.rows; row++)
	{
		float * pt = (float *)object.row(row).data;
		//every col
		for (int col = 0; col < object.cols; col++)
		{
			//point has valid depth value
			if (*(pt + col) > nearCameraThresh)
			{
				//get average depth in nearby domain
				vector<float> valid; float val(0);
				for (int srow = max(0, row - 1); srow < min(object.rows, row + 2); srow++)
				{
					float* spt = (float *)object.row(srow).data;
					for (int scol = max(0, col - 1); scol < min(object.cols, col + 2); scol++)
					{
						if (*(spt + scol) < nearCameraThresh) valid.push_back(*(spt + scol));
					}
				}

				//set default value
				if (valid.empty()) *(pt + col) = 44;
				else
				{
					val = std::accumulate(valid.begin(), valid.end(), 0);
					*(pt + col) = val / valid.size();
				}
			}//end thresh condition
		}//end cols
	}//end rows
}

/*!
@function              calculate object average depth
@abstract              is time consuming
@discussion
@param
@result
*/
float ProjectorCamera::contourAverageDepth(vector<cv::Point>& contour)
{
	//paras
	cv::Rect contRect = cv::boundingRect(contour);
	int pointCount(0);
	float depthSum(0);

	//check each valid point
	for (int row = contRect.y; row < contRect.y + contRect.height; row++)
	{
		for (int col = contRect.x; col < contRect.x + contRect.width; col++)
		{
			//test if point is valid
			if (pointPolygonTest(contour, cv::Point(col, row), 1) >= 0 &&
				foreground.at<float>(row, col) > 0)
			{
				pointCount++;
				depthSum += foreground.at<float>(row, col);
			}

		}
	}

	return depthSum / pointCount;
}

/*!
@function              change mat data format from float to uchar
@abstract              is fast than opencv cvtColor function
@discussion
@param
@result
*/
void ProjectorCamera::convertFloatToChar(cv::Mat& src, cv::Mat& dst)
{
	//make sure src not empty
	if (src.empty()) return;

	dst = cv::Mat(src.rows, src.cols, CV_8UC1, cv::Scalar(0));

	//if src mat data is continue is memory
	if (src.isContinuous() && dst.isContinuous())
	{
		float * srcPt = (float *)src.data;
		uchar * dstPt = dst.data;
		for (int ind = 0; ind<src.rows * src.cols; ind++)
		{
			if ((*srcPt) > 0) *dstPt = 1;
			srcPt++;
			dstPt++;
		}
	}
	else
	{
		for (int row = 0; row<src.rows; row++)
		{
			float * srcPt = (float *)src.row(row).data;
			uchar * dstPt = dst.row(row).data;
			for (int col = 0; col<src.cols; col++)
			{
				if ((*srcPt) > 0) *dstPt = 1;
				srcPt++;
				dstPt++;
			}
		}
	}
}

/*
*function: detection in the ari gesture from foreground image
*note    : by zhang bo
*/
void ProjectorCamera::findInAirGesture(){
	int objectssize = objects.size();
	 LOGD("frameId = %d, objectssize = %d",frameId,objectssize);
	if (!objectssize)
		return;

	//将前景转化为3通道,方便显示
	vector<cv::Mat> foregrounds;
	for (int i = 0; i < 3; i++)
		foregrounds.push_back(foreground);
	cv::merge(foregrounds, foreground_copy);
    //LOGD("foreground_copy.cols = %d,foreground_copy.rows = %d",foreground_copy.cols,foreground_copy.rows);
	for (int i = 0; i < objects.size(); i++){
		//min object area
		int cArea = objects[i].cArea;
		if (cArea < 3000 || cArea > 40000) continue;
		cv::Mat contourMat = cv::Mat(objects[i].contour);

		vector<cv::Point> approxCurve;
		//get DP points of the object
		cv::approxPolyDP(contourMat, approxCurve, 10, true);

		vector< vector<cv::Point> > debugContourV;
		debugContourV.push_back(approxCurve);

		//get convex hull
		vector<int> hull;
		cv::convexHull(cv::Mat(approxCurve), hull, false, false);
		//lixing's code--2-3ms的时间。
		cv::Mat disResult;
		cv::distanceTransform(binaryCopy, disResult, CV_DIST_L2, 3);
		vector<int> new_counter_ind;
		vector<cv::Point> new_conter_pt(approxCurve);
		double maxVal = 0.0;double minVal = 0.0;
		cv::Point maxLoc = cv::Point(-1, -1);
		cv::Point minLoc = cv::Point(-1, -1);
		if (new_conter_pt.size() > 1)
		{
			cv::Rect contRect = boundingRect(new_conter_pt);
			int bound_height = 80;
			cv::Rect boundRect = cv::Rect(contRect.x, contRect.y, contRect.width,( contRect.y + bound_height) > screenRoi.height ? screenRoi.height - contRect.y : bound_height);
			//rectangle(foreground_copy, boundRect,COLOR_RED, 2);
			for (int i = 0; i < new_conter_pt.size(); i++)
				cv::minMaxLoc(disResult(boundRect), &minVal, &maxVal, &minLoc, &maxLoc);
			maxLoc.x += boundRect.x;
			maxLoc.y += boundRect.y;
			//LOGD("frameId = %d,maxLoc.x = %d,maxLoc.y = %d,new_conter_pt.size() = %d,maxVal = %f",frameId,maxLoc.x,maxLoc.y,new_conter_pt.size(),maxVal);
		}
        //LOGD("frameId = %d, maxLoc.x = %d,maxLoc.y = %d, maxVal = %d",frameId,maxLoc.x,maxLoc.y,maxVal);
		// find convexity defects
		vector<ConvexityDefect> convexDefects;
		findConvexityDefects(approxCurve, hull, convexDefects);
		// assemble point set of convex hull
		vector<cv::Point> hullPoints;
		for (int k = 0; k < hull.size(); k++){
			int curveIndex = hull[k];
			cv::Point p = approxCurve[curveIndex];
			hullPoints.push_back(p);
		}
		// area of hull and curve
		double hullArea = contourArea(cv::Mat(hullPoints));//凸包点
		double curveArea = contourArea(cv::Mat(approxCurve));//凹包点+凸包点
		double handRatio = curveArea / hullArea;

		//calculate the centers of the likely hand
		cv::Point center_point = cv::Size(0, 0);
		center_point.x = maxLoc.x; center_point.y = maxLoc.y;

		bool isInvalidPoint = center_point.x < 20 || center_point.y < 10 || center_point.x > foreground_copy.cols - 20 || center_point.y > foreground_copy.rows -  10||maxVal<=10||approxCurve.size()<4;
		bool isBindPoint = (center_point.y>(4*foreground_copy.rows)/5)&&(center_point.y<foreground_copy.rows-10)&&(center_point.x>foreground_copy.cols/2-20)&&(center_point.x<foreground_copy.cols/2+20);
		//LOGD("(3*foreground_copy.rows)/4 = %d,foreground_copy.rows-10 = %d,foreground_copy.cols/2-70 = %d,foreground_copy.cols/2+70 = %d",(3*foreground_copy.rows)/4,foreground_copy.rows-10,foreground_copy.cols/2-70,foreground_copy.cols/2+70);
        LOGD("Before:frameId = %d,center_point.x = %d,center_point.y = %d,maxVal = %f, isInvalidPoint = %d,isBindPoint = %d",frameId,center_point.x,center_point.y,maxVal,isInvalidPoint,isBindPoint);
		if (isInvalidPoint) {//invalid center point.
		    LOGD("the point is invalid!!");
		    continue;
		}
		else if(isBindPoint){//bind center point.
		    LOGD("the point is in the blind area!!!");
            pointsInLastFrames.clear();
            continue;
		}
		LOGD("the point is valid!!!");
		//push hand centers into vector
		//if (center_point.x || center_point.y){
			pointsInLastFrames.push_back(center_point);//推入新的帧
			if (pointsInLastFrames.size() > trajFrameNum)
				pointsInLastFrames.erase(pointsInLastFrames.begin());//清除第一帧
		//}
		//计算y方向的最小值
		if(pointsInLastFrames.size() ==trajFrameNum){
            int miny = 0;
             for (int nsorty = 0; nsorty < trajFrameNum - 1; nsorty++){
                miny = pointsInLastFrames[nsorty].y<pointsInLastFrames[nsorty+1].y ? pointsInLastFrames[nsorty].y :pointsInLastFrames[nsorty+1].y;
            }
            LOGD("frameId = %d,miny = %d",frameId,miny);
            if(miny>4*foreground_copy.rows/5){
                pointsInLastFrames.clear();
                continue;
            }
		}

		LOGD("After:frameId = %d,center_point.x = %d,center_point.y = %d, maxVal = %f,cArea = %d, pointsInLastFrames.size() = %d",frameId,center_point.x,center_point.y,maxVal,cArea,pointsInLastFrames.size());
		//rectified by zb：将凹凸包点同时判定，同时修改angle的约束
		int nlittleangle = 0;
		for (int j = 0; j < approxCurve.size(); j++)
		{
			//所有凹凸点的index
			int index = j, leftind(0), rightind(0);

			//所有凹凸点构成的角度
			float angle(0);
			if (index == 0)
			{
				leftind = approxCurve.size() - 1;
				rightind = index + 1;
			}
			else if (index == approxCurve.size() - 1)
			{
				leftind = index - 1;
				rightind = 0;
			}
			else
			{
				leftind = index - 1;
				rightind = index + 1;
			}
			//将靠近边缘的角度判定剔除
			if (approxCurve[index].x < 10 || approxCurve[index].y < 10 || approxCurve[index].x>foreground_copy.cols - 10 || approxCurve[index].y>foreground_copy.rows - 10)
				continue;
			angle = caclAngleForInAirGesture(approxCurve[leftind], approxCurve[index], approxCurve[rightind]);

			if (angle < 75)//
				nlittleangle++;
		}
		//1.hand grasp state
		int approxCurvesize = approxCurve.size(); int hullPointssize = hullPoints.size();
		int maxdistance_x = abs(pointsInLastFrames[0].x - pointsInLastFrames[pointsInLastFrames.size() - 1].x);
		int maxdistance_y = abs(pointsInLastFrames[0].y - pointsInLastFrames[pointsInLastFrames.size() - 1].y);
		bool openstate = isHandGraspOrOpen(handRatio, approxCurvesize, hullPointssize, nlittleangle, maxdistance_x,maxdistance_y, center_point);

		//2.hand wave state
		int wavestate = handWaveState(pointsInLastFrames);

		//in air gesture management
		inAirGestureManage(openstate, wavestate);
		LOGD("frameId = %d, Sucessfully end the loop!!!",frameId);
	}
}

bool ProjectorCamera::isHandGraspOrOpen(double& handRatio, int& approxCurveSize, int& hullsize, int& littleangle, int& distance_x,int& distance_y,cv::Point& centerpoint){
	ncount_totalimg++;//hand grasp state
	ncount_totalimg_open++;//hand open state
	bool grapestate = GRASPFAITTURE;
	bool openstate = OPENFAITURE;

	//单帧判定的结果
	bool hullstate = ((approxCurveSize - hullsize) >= 1 && (approxCurveSize - hullsize) <= 4) && (hullsize >= 5 && hullsize <= 7) && (approxCurveSize <= 10 && approxCurveSize >= 6);
	bool pointstate = (centerpoint.x > 120 && centerpoint.x < depthImg.cols - 120 && centerpoint.y >20 && centerpoint.y < depthImg.rows-depthImg.rows/4);
	//hand grasp state
	if (/*handRatio > 0.7&&*/ /*handRatio<0.95&&*/ !littleangle && hullstate && pointstate){
		ncount_grasp++;
	}
	//hand open state------------------new
	else if (littleangle >= 2 && approxCurveSize >= 10 && approxCurveSize - hullsize >= 3 && pointstate){
		ncount_open++;
	}

	//多帧给出最后结果，hand grasp state output
	if (ncount_totalimg == trajFrameNum){//make a decision every trajFrameNum
		if (ncount_grasp >= trajFrameNum && distance_x < 30 && distance_y < 30){//first frame is no grasping,last frame is grasping.
			grapestate = GRASPSUCCESS;
		}
		ncount_grasp = 0;
		ncount_totalimg = 0;
	}
	handgraspstates_temp.push_back(grapestate);

	//hand open state output----------new
	if (grapestate == GRASPSUCCESS){
		ncount_open = 0;
		ncount_totalimg_open = 0;
	}
	else if (grapestate == GRASPFAITTURE){
		if (ncount_totalimg_open == trajFrameNum*ntimes){
			if (ncount_open >= trajFrameNum*(ntimes -1)&& distance_x < 30 && distance_y < 30){
				openstate = OPENSUCCESS;
			}
		ncount_open = 0;
		ncount_totalimg_open = 0;
		}
	}
	return openstate;
}

int ProjectorCamera::handWaveState(vector<cv::Point>& pointsInLastFrames){

	int frameNum = pointsInLastFrames.size();
	if (frameNum != trajFrameNum)
		return NOWAVE;
	//检测xy方向为升序或者降序（升为1，降为-1）
	int isSortAscending_x = pointsInLastFrames[0].x <= pointsInLastFrames[1].x ? 1 : -1;//x:1-->right,-1-->left.
	int isSortAscending_y = pointsInLastFrames[0].y <= pointsInLastFrames[1].y ? 1 : -1;//y:1-->dowm,-1-->up.
	bool issorted_x = true, issorted_y = true;//排序标志位
	//检查x方向有序性
	for (int nsortx = 0; nsortx < trajFrameNum - 1; nsortx++){
		if (isSortAscending_x*(pointsInLastFrames[nsortx].x - pointsInLastFrames[nsortx + 1].x) >= 0){
			issorted_x = false;
			break;
		}
	}
	//检查y方向有序性
	for (int nsorty = 0; nsorty < trajFrameNum - 1; nsorty++){
		if (isSortAscending_y*(pointsInLastFrames[nsorty].y - pointsInLastFrames[nsorty + 1].y) >= 0){
			issorted_y = false;
			break;
		}
	}
	float maxdistace_x = abs(pointsInLastFrames[0].x - pointsInLastFrames[frameNum - 1].x);
	float maxdistace_y = abs(pointsInLastFrames[0].y - pointsInLastFrames[frameNum - 1].y);
	//no hand wave
	if ((!issorted_y) && (!issorted_x)){
		return NOWAVE;
	}
	//hand wave in x derection
	//else if ((!issorted_y) && issorted_x && maxdistace_x > 120 && maxdistace_y < 60){
	else if (issorted_x && maxdistace_x > 120 && maxdistace_y < 100 && pointsInLastFrames[0].y < 5*depthImg.rows/6){
		//TODO: hand wave detection in x direction
		if (isSortAscending_x == 1 && pointsInLastFrames[0].x <= depthImg.cols / 2-5 && pointsInLastFrames[trajFrameNum-1].x >= depthImg.cols / 2+5){//right
			return WAVERIGHT;
		}
		else if (isSortAscending_x == -1 && pointsInLastFrames[trajFrameNum - 1].x <= depthImg.cols / 2-5 && pointsInLastFrames[0].x >= depthImg.cols / 2+5){//left
			return WAVELEFT;
		}
	}
	return NOWAVE;
}

void ProjectorCamera::findConvexityDefects(vector<cv::Point>& contour, vector<int>& hull, vector<ConvexityDefect>& convexDefects)
{
	if (hull.size() > 0 && contour.size() > 0)
	{
		CvSeq* contourPoints;
		CvSeq* defects;
		CvMemStorage* storage;
		CvMemStorage* strDefects;
		CvMemStorage* contourStr;
		CvConvexityDefect *defectArray = 0;

		strDefects = cvCreateMemStorage();
		defects = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), strDefects);

		//We transform our vector<Point> into a CvSeq* object of CvPoint.
		contourStr = cvCreateMemStorage();
		contourPoints = cvCreateSeq(CV_SEQ_KIND_GENERIC | CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), contourStr);
		for (int i = 0; i < (int)contour.size(); i++) {
			CvPoint cp = { contour[i].x, contour[i].y };
			cvSeqPush(contourPoints, &cp);
		}

		//Now, we do the same thing with the hull index
		int count = (int)hull.size();
		//int hullK[count];
		int* hullK = (int*)malloc(count*sizeof(int));
		for (int i = 0; i < count; i++) { hullK[i] = hull.at(i); }
		CvMat hullMat = cvMat(1, count, CV_32SC1, hullK);

		// calculate convexity defects
		storage = cvCreateMemStorage(0);
		defects = cvConvexityDefects(contourPoints, &hullMat, storage);
		defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*defects->total);
		cvCvtSeqToArray(defects, defectArray, CV_WHOLE_SEQ);
		//printf("DefectArray %i %i\n",defectArray->end->x, defectArray->end->y);

		//We store defects points in the convexDefects parameter.
		for (int i = 0; i<defects->total; i++){
			ConvexityDefect def;
			def.start = cv::Point(defectArray[i].start->x, defectArray[i].start->y);
			def.end = cv::Point(defectArray[i].end->x, defectArray[i].end->y);
			def.depth_point = cv::Point(defectArray[i].depth_point->x, defectArray[i].depth_point->y);
			def.depth = defectArray[i].depth;
			convexDefects.push_back(def);
		}

		// release memory
		cvReleaseMemStorage(&contourStr);
		cvReleaseMemStorage(&strDefects);
		cvReleaseMemStorage(&storage);
	}
}

cv::Point ProjectorCamera::CalculateSumofPoints(cv::Point pointA, cv::Point pointB){
	return cv::Size(pointA.x + pointB.x, pointA.y + pointB.y);
}

void ProjectorCamera::inAirGestureManage(bool& openstate, int& handwave){

	//1.hand grasp management
	if (handgraspstates_temp.size() == trajFrameNum){
		graspstate_temp = handgraspstates_temp.back();
		handgraspstates_temp.clear();
	}

	//if (graspstate_temp == GRASPSUCCESS&& !m_graspstate){
    if (graspstate_temp == GRASPSUCCESS&& !m_graspstate&&!m_openstate){

		lastframe_grasp = frameId;

		m_graspstate = true;
		graspstate_finnal = GRASPSUCCESS;
		LOGD("hand grasp detected!!! frameId = %d",frameId);
		return;
	}

	else if (frameId - lastframe_grasp > trajFrameNum*(ntimes*2)){

		m_graspstate = false;
		graspstate_finnal = GRASPFAITTURE;
	}
	else{
		graspstate_finnal = GRASPFAITTURE;
	}

	//2.hand open management
	//if (openstate == OPENSUCCESS&&!m_openstate){
	if (openstate == OPENSUCCESS&&!m_openstate&&!m_graspstate){


	    lastframe = frameId;

		m_openstate = true;
		openstate_finnal = OPENSUCCESS;
		LOGD("hand open detected!!! frameId = %d",frameId);
		return;
	}
	//else if (openstate == OPENSUCCESS && frameId - lastframe>=trajFrameNum*(ntimes+1)){
	else if (frameId - lastframe>=trajFrameNum*(ntimes*2)){
		m_openstate = false;
		openstate_finnal = OPENFAITURE;
	}
	else{
		openstate_finnal = OPENFAITURE;
	}

	//3.hand wave management
	if (1 == handwave&&m_wavestate == 0){
		lastframe_wave = frameId;
		wavestate_finnal = WAVERIGHT;
		m_wavestate = 1;
		pointsInLastFrames.clear();
		LOGD("hand right wave detected!!! frameId = %d",frameId);
		return;
	}
	else if(2 == handwave&&m_wavestate == 0){
		lastframe_wave = frameId;
		wavestate_finnal = WAVELEFT;
		m_wavestate = 2;
		pointsInLastFrames.clear();
		LOGD("hand left wave detected!!! frameId = %d",frameId);
		return;
	}
	else if (frameId - lastframe_wave > trajFrameNum * ntimes){
		m_wavestate = 0;
		wavestate_finnal = NOWAVE;

		if(handwave)
		    pointsInLastFrames.clear();
	}
	else{
		wavestate_finnal = NOWAVE;
	}

}

/*!
@function              get dynamic background depth
@abstract              compared to the history information
@discussion
@param
@result
*/
void ProjectorCamera::getDynamicBgDepth()
{
	try{
		/* 1.update the depthframe window each frame */
		if (frameId <= initFrames) // initFrames = 50
		{
			//accumulate depth image
			bgdepths.push_back(depthImg);
			isFingers.push_back(false);
			LOGD("bgdepths.push_back frameId%d ", frameId);

		}
		else
		{
			bgdepths.push_back(depthImg);
			bgdepths.erase(bgdepths.begin()); // delete the first depth frame from the window
            if(fingerTouchRole.existPalm)
                isFingers.push_back(true);
            else
                isFingers.push_back(false);
            isFingers.erase(isFingers.begin());
			//LOGD("frameId = %d,fingerTouchRole.curtTouchHands.size() = %d,ncount = %d",frameId,fingerTouchRole.curtTouchHands.size(),ncount);
		}
		//averaImg = depthImg.clone();
		/* 2.initialize averaImg */

		if (frameId == initFrames + 1)
		{
			LOGD("Enter the if !!,frameId = %d", frameId);
			for (int index = 0; index < depthImg.rows*depthImg.cols; index++)
			{
				ifBackground.push_back(true);
			}
			//LOGD("ifBackground is OK,frameId = %d", frameId);
			averaImg = cv::Mat(depthImg.rows, depthImg.cols, CV_32F);
			LOGD("Background initialization completed!,frameId = %d", frameId);
			//averaImg = depthImg.clone();
			assert(depthImg.rows == averaImg.rows && depthImg.cols == averaImg.cols);
			cv::Mat sum = bgdepths[0].clone();
			for (int i = 1; i < initFrames; i++)
				sum = sum + bgdepths[i];
			averaImg = sum / initFrames;
			LOGD("Background initialization completed!,frameId = %d", frameId);
		}

		/* 3.calculate background every windowStep frames */
		if ((frameId % windowStep) == 0 && frameId > initFrames)
			// after pushing all initial frames into window
		{
			CVTIME getBgDepthTime;
			LOGD("enter the if loop,frameId = %d", frameId);
			int obsPixel = 0;
			cv::Mat sum = bgdepths[initFrames - calLength].clone();int fingerFrameNum = 0;
			for (int i = initFrames - calLength + 1; i < initFrames; i++){
			    sum = sum + bgdepths[i];
			}
			//TODO: if hand finger down the surface.
			for(int j = 0;j<isFingers.size();j++){
			    if(isFingers[j])
            	    fingerFrameNum++;
			}
			LOGD("frameId = %d,fingerFrameNum = %d",frameId,fingerFrameNum);
			if(fingerFrameNum >= 1)
			    return;
			LOGD("bgdepths adding successfully,frameId = %d", frameId);
			cv::Mat avimg = sum / calLength;
			for (int row = 15; row < avimg.rows-25; row++)
				for (int col = 10; col < avimg.cols-10; col++)
				{
				pixelInd = row * depthImg.cols + col;
				if (ifBackground[pixelInd] == true && abs(avimg.at<float>(row, col) - averaImg.at<float>(row, col)) > AvgThreshold && frameId > initFrames + 2)
				{
					ifBackground[pixelInd] = false; // mean depth has obvious difference, change the status
				}
				else if (ifBackground[pixelInd] == false && abs(avimg.at<float>(row, col) - averaImg.at<float>(row, col)) < AvgThreshold)
					ifBackground[pixelInd] = true; // former obstruction has been removed, change the status
				}
			LOGD("ifBackground caculation successfully,frameId = %d", frameId);
			for (int index = 0; index < depthImg.rows*depthImg.cols; index++)
			{
				if (ifBackground[index] == false)
				{
					obsPixel++;
				}
			}
			//LOGD("obsPixel caculation successfully,frameId = %d", frameId);
			LOGD("Obstruction Area: = %d", obsPixel);
			if (obsPixel >(obstrArea))
				LOGD("frameId = %d :No background updating!", frameId);
			else
			{
				LOGD("frameId = %d :Update the background", frameId);
				averaImg = avimg;
			}
			LOGD("nativeStart caught getBgDepthTime: %f", getBgDepthTime.getClock());
		}
	}
	catch (cv::Exception& e){
		LOGF("Bgmodel caught cv::Exception: %s", e.what());
	}
}