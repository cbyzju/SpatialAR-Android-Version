/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	singleActive.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       singleActive processing file
*********************************************************************/


#include "idle.h"
#include "singleActive.h"
#include "doubleOneHandActive.h"
#include "doubleTwoHandActive.h"
#include "sendEvent.h"
#include "projectorCamera.h"
/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
SingleActive::SingleActive()
{
	setDescription("singleActive");
	sig2IdleInterval = 3;
	sig2MulInterval  = 6;
	minMoveDis = 20;
	maxMoveDis = minMoveDis * 10;
	insertNum  = 5;
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
SingleActive::~SingleActive()
{

}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
void SingleActive::Excute(Role* role)
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
void SingleActive::dealIdelTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > sig2IdleInterval)
	{
		//end single touch event
		EVENT::up(role->linuxFileId);

		//change state
		role->currentState = Idle::getInstance();

		//clear histroy data
		role->histTouchHands.clear();
	}
	//state between two hand double active to idle
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
void SingleActive::dealSingleTouch(Role* role)
{
    LOGD("SingelActive dealSingleTouch");
	//touch information
	TouchPoint curtTouchPoint(role->curtTouchHands[0].touchPoints[0]);
	TouchPoint lastTouchPoint(role->histTouchHands[role->histTouchHands.size() - 1][0].touchPoints[0]);

	//distance between current point and last point
	float dis = norm(curtTouchPoint.tipInPro - lastTouchPoint.tipInPro);
    LOGD("single touch dis %f", dis);

    int appOffset = 0;
    if(role->appState == 4) appOffset = 15;
	//(SPEED MOVE)
	if (dis > maxMoveDis)
	{

	}
	//(MOVE) 
	else if (dis > minMoveDis + appOffset)
	{
		//move prediction
		movePrediction(role);

		cv::Point2f last2cur = curtTouchPoint.tipInPro - lastTouchPoint.tipInPro;
		for (int ind = insertNum - 1; ind >= 0; ind--)
		{
			//if not shake using current data
			EVENT::singletouch(role->linuxFileId, curtTouchPoint.tipInPro.x - (float)ind * last2cur.x / insertNum,
				curtTouchPoint.tipInPro.y - (float)ind * last2cur.y / insertNum,
				curtTouchPoint.orien, curtTouchPoint.angle);
		}
	}
	//(STATIC)
	else
	{
		//if shake using histroy point and current angle ande orientation
		EVENT::singletouch(role->linuxFileId, lastTouchPoint.tipInPro.x,
			lastTouchPoint.tipInPro.y, curtTouchPoint.orien, curtTouchPoint.angle);
		role->curtTouchHands[0].touchPoints[0].tipInPro.x = lastTouchPoint.tipInPro.x;
		role->curtTouchHands[0].touchPoints[0].tipInPro.y = lastTouchPoint.tipInPro.y;
	}

	role->histTouchHands.push_back(role->curtTouchHands);
	role->lastTouchId = role->curtFrameId;

	return;
}

/*!
@function
@abstract              deal current one hand double touch state
@discussion
@param
@result
*/
void SingleActive::dealOneHandDoubleTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > sig2MulInterval)
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
void SingleActive::dealTwoHandDoubleTouch(Role* role)
{
	if ((role->curtFrameId - role->lastTouchId) > sig2MulInterval)
	{
		//end single touch event
		EVENT::up(role->linuxFileId);

		//change state
		role->currentState = DoubleTwoHandActive::getInstance();

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
@abstract              get an current state
@discussion
@param
@result
*/
void SingleActive::movePrediction(Role* role)
{
	return ;
}

/*!
@function
@abstract              get an current state
@discussion
@param
@result
*/
State* SingleActive::getInstance()
{
	static SingleActive instance;
	return &instance;
}