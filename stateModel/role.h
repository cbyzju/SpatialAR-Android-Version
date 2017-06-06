/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	role.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       procamera interactive base state file
*********************************************************************/

#ifndef _ROLE_
#define _ROLE_

#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "touchInfo.h"
//#include "sendEvent.h"
#include "state.h"

//class State;

#define CURT_IDLE_TOUCH 0
#define CURT_SINGLE_TOUCH 1
#define CURT_1HAND2_TOUCH 2
#define CURT_2HAND2_TOUCH 4
#define CURT_OTHER_TOUCH  8

using namespace std;
//using namespace cv;

class Role
{
public:
	Role();
	~Role();

	//���õ�ǰ״̬  
	void setState(State*);

	//��ȡ��ǰ״̬  
	State* getState() const;

	//refresh state
	void update();

	//check current touch state
	int getCurtTouchState();

	bool isCurtIdleTouch();

	bool isCurtSingleTouch();

	bool isCurtOneHandDoubleTouch();

	bool isCurtTwoHandDoubleTouch();

public:
	//for state use
	State*   currentState;
	int firstTouchId;
	int lastTouchId;
	int appState;
	vector<vector<TouchHand> > histTouchHands;

	//for every updata
	int curtFrameId;
	int linuxFileId;
	int touchState;
	bool existPalm;
	vector<TouchHand> curtTouchHands;
};
#endif