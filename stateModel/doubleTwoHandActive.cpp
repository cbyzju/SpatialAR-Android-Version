/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	doubleActive.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       doubleActive processing file
*********************************************************************/


#include "idle.h"
#include "singleActive.h"
#include "doubleOneHandActive.h"
#include "doubleTwoHandActive.h"
#include "sendEvent.h"
#include <android/log.h>

#define LOG_TAG "ProjectorCamera/DebugInformation"
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
DoubleTwoHandActive::DoubleTwoHandActive()
{
	setDescription("DoubleTwoHandActive");
	minMoveDis = 5;
	maxMoveDis = minMoveDis * 10;
	minPinchDis = 12;
	mul2IdleInterval = 4;
	mul2SigInterval  = 4;
	mul2MulInterval  = 4;
	insertNum = 5;


}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
DoubleTwoHandActive::~DoubleTwoHandActive()
{

}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
void DoubleTwoHandActive::Excute(Role* role)
{
	switch (role->getCurtTouchState())
	{
	case CURT_IDLE_TOUCH:
	{
		dealIdelTouch(role);
		break;
	}
	case CURT_SINGLE_TOUCH:
	{
		dealSingleTouch(role);
		break;
	}
	case CURT_1HAND2_TOUCH:
	{
		dealOneHandDoubleTouch(role);
		break;
	}
	case CURT_2HAND2_TOUCH:
	{
		dealTwoHandDoubleTouch(role);
		break;
	}
	default:
		break;
	}
}

/*!
@function
@abstract              deal current idle touch state
@discussion
@param
@result
*/
void DoubleTwoHandActive::dealIdelTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > mul2IdleInterval)
	{
		//end single touch event
		EVENT::up(role->linuxFileId);

		//change state
		role->currentState = Idle::getInstance();

		//clear histroy data
		role->histTouchHands.clear();
	}
	//state between double active to idle
	else
	{

	}

	return;
}

/*!
@function
@abstract              deal current single touch state
@discussion
@param
@result
*/
void DoubleTwoHandActive::dealSingleTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > mul2SigInterval)
	{
		//end single touch event
		EVENT::up(role->linuxFileId);

		//change state
		role->currentState = SingleActive::getInstance();

		//clear histroy data
		role->histTouchHands.clear();

		//set paras
		role->firstTouchId = role->curtTouchHands[0].frameId;
		role->lastTouchId = role->curtTouchHands[0].frameId;
		role->histTouchHands.push_back(role->curtTouchHands);
	}
	//state between double active to single active
	else
	{
	}

	return;
}

/*!
@function
@abstract              deal current one hand double touch state
@discussion
@param
@result
*/
void DoubleTwoHandActive::dealOneHandDoubleTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > mul2MulInterval)
	{
		//end single touch event
		EVENT::up(role->linuxFileId);

		//change state
		role->currentState = DoubleOneHandActive::getInstance();

		//clear histroy data
		role->histTouchHands.clear();

		//set paras
		role->firstTouchId = role->curtTouchHands[0].frameId;
		role->lastTouchId = role->curtTouchHands[0].frameId;
		role->histTouchHands.push_back(role->curtTouchHands);
	}
	//state between single active to double active
	else
	{
	}

	return;
}

/*!
@function
@abstract              deal current two hand double touch state
@discussion
@param
@result
*/
void DoubleTwoHandActive::dealTwoHandDoubleTouch(Role* role)
{
    LOGD("DoubleTwoHandActive dealTwoHandDoubleTouch");
   	//touch information
	TouchPoint& curtTouchPoint1 = (role->curtTouchHands[0].touchPoints[0]);
	TouchPoint& curtTouchPoint2 = (role->curtTouchHands[1].touchPoints[0]);
	TouchPoint& lastTouchPoint1 = (role->histTouchHands[role->histTouchHands.size() - 1][0].touchPoints[0]);
	TouchPoint& lastTouchPoint2 = (role->histTouchHands[role->histTouchHands.size() - 1][1].touchPoints[0]);

	//keep order according to palm center position
	if (norm(role->curtTouchHands[0].palmCenter - role->histTouchHands[role->histTouchHands.size() - 1][0].palmCenter) >
		norm(role->curtTouchHands[0].palmCenter - role->histTouchHands[role->histTouchHands.size() - 1][1].palmCenter))
	{
		TouchHand temHand = role->curtTouchHands[0];
		role->curtTouchHands[0] = role->curtTouchHands[1];
		role->curtTouchHands[1] = temHand;
	}

	//point change state
	bool point1Move(true);
	bool point2Move(true);
	bool refreshPoints(true);

	//every point keep stable
	if (norm(curtTouchPoint1.tipInPro - lastTouchPoint1.tipInPro) < minMoveDis)
	{
		curtTouchPoint1.tipInPro = lastTouchPoint1.tipInPro;
		point1Move = false;
	}
	if (norm(curtTouchPoint2.tipInPro - lastTouchPoint2.tipInPro) < minMoveDis)
	{
		curtTouchPoint2.tipInPro = lastTouchPoint2.tipInPro;
		point2Move = false;
	}

	//calculate point move distance
	float lastPointDis = norm(curtTouchPoint1.tipInPro - curtTouchPoint2.tipInPro);

	float curtPointDis = norm(lastTouchPoint1.tipInPro - lastTouchPoint2.tipInPro);

	//distance between current state and last state
	float pinchDis = abs(curtPointDis - lastPointDis);
	
	//pinch move
	if (pinchDis > minPinchDis || point1Move || point2Move)
	{
		cv::Point2f d1 = curtTouchPoint1.tipInPro - lastTouchPoint1.tipInPro;
		cv::Point2f d2 = curtTouchPoint2.tipInPro - lastTouchPoint2.tipInPro;

		for (int ind = insertNum - 1; ind >= 0; ind--)
		{
			EVENT::multitouch(role->linuxFileId, 
				curtTouchPoint1.tipInPro.x - (float)ind * d1.x / insertNum, 
				curtTouchPoint1.tipInPro.y - (float)ind * d1.y / insertNum,
				curtTouchPoint2.tipInPro.x - (float)ind * d2.x / insertNum, 
				curtTouchPoint2.tipInPro.y - (float)ind * d2.y / insertNum,
				curtTouchPoint1.orien, curtTouchPoint1.angle, 
				curtTouchPoint2.orien, curtTouchPoint2.angle);
		}		
	}
	//pinch static
	else
	{
		EVENT::multitouch(role->linuxFileId,
			lastTouchPoint1.tipInPro.x,
			lastTouchPoint1.tipInPro.y,
			lastTouchPoint2.tipInPro.x,
			lastTouchPoint2.tipInPro.y,
			lastTouchPoint1.orien, lastTouchPoint1.angle,
			lastTouchPoint2.orien, lastTouchPoint2.angle);
	}

	role->histTouchHands.push_back(role->curtTouchHands);
	role->lastTouchId = role->curtFrameId;

	return;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
void DoubleTwoHandActive::movePrediction(Role* role)
{
	return;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
void DoubleTwoHandActive::pinchFilter(Role* role)
{
	return;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
void DoubleTwoHandActive::rotationFilter(Role* role)
{
	return;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
State* DoubleTwoHandActive::getInstance()
{
	static DoubleTwoHandActive instance;
	return &instance;
}