/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	doubleActive.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       doubleActive processing file
*********************************************************************/

#ifndef _DOUBLE_TWO_HAND_ACTIVE_
#define _DOUBLE_TWO_HAND_ACTIVE_

#include "state.h"
#include "touchInfo.h"

class Role;
class State;

class DoubleTwoHandActive :public State
{
private:
	DoubleTwoHandActive();
	~DoubleTwoHandActive();

public:
	void Excute(Role*);

	static State* getInstance();

	void dealIdelTouch(Role*);

	void dealSingleTouch(Role*);

	void dealOneHandDoubleTouch(Role*);

	void dealTwoHandDoubleTouch(Role*);

	void movePrediction(Role *);

	void pinchFilter(Role *);

	void rotationFilter(Role *);

public:

	//double active to idle interval
	int mul2IdleInterval;

	//double active to singel active interval
	int mul2SigInterval;

	int mul2MulInterval;

	//double active min move distance
	float minMoveDis;

	//double active max move distance
	float maxMoveDis;

	//double active min pinch distance
	float minPinchDis;

	//insert nums between two continue points
	int insertNum;
};
#endif