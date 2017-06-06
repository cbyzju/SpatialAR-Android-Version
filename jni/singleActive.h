/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	singleActive.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       singleActive processing file
*********************************************************************/


#ifndef _SINGLE_ACTIVE_
#define _SINGLE_ACTIVE_

#include "state.h"
#include "touchInfo.h"


class SingleActive :public State
{
private:
	SingleActive();
	~SingleActive();
	
public:
	void Excute(Role*);

	static State* getInstance();

	void movePrediction(Role *);

	void dealIdelTouch(Role*);

	void dealSingleTouch(Role*);

	void dealOneHandDoubleTouch(Role*);

	void dealTwoHandDoubleTouch(Role*);

public:
	
	//single active to idle interval
	int sig2IdleInterval;

	//single active to double active interval
	int sig2MulInterval;

	//single active min move distance
	float minMoveDis;

	//singel active max move distance
	float maxMoveDis;

	//insert nums between two continue points
	int insertNum;
};
#endif