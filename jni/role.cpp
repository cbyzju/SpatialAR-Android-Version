/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	role.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       procamera interactive base state file
*********************************************************************/


#include "role.h"
#include "idle.h"
/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
Role::Role()
{
	//initialize with idle state
	appState = 0;
	currentState = Idle::getInstance();
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
Role::~Role()
{

}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
void Role::update()
{
	currentState->Excute(this);
}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
State* Role::getState() const
{
	return currentState;
}

/*!
@function
@abstract              refresh current state
@discussion
@param
@result
*/
void Role::setState(State* newState)
{
	currentState = newState;
}

/*!
@function
@abstract              check current is idle touch state
@discussion
@param
@result
*/
bool Role::isCurtIdleTouch()
{
	return(this->curtTouchHands.size() == 0);
}

/*!
@function
@abstract              check current is single touch state
@discussion
@param
@result
*/
bool Role::isCurtSingleTouch()
{
	return(this->curtTouchHands.size() == 1 &&
		this->curtTouchHands[0].touchPoints.size() == 1);
}

/*!
@function
@abstract              check current is one hand double touch state
@discussion
@param
@result
*/
bool Role::isCurtOneHandDoubleTouch()
{
	return(this->curtTouchHands.size() == 1 &&
		this->curtTouchHands[0].touchPoints.size() == 2);
}

/*!
@function
@abstract              check current is two hand double touch state
@discussion
@param
@result
*/
bool Role::isCurtTwoHandDoubleTouch()
{
	return(this->curtTouchHands.size() == 2 &&
		this->curtTouchHands[0].touchPoints.size() == 1 &&
		this->curtTouchHands[1].touchPoints.size() == 1);
}

/*!
@function
@abstract              check current is two hand double touch state
@discussion
@param
@result
*/
int Role::getCurtTouchState()
{
	if (isCurtIdleTouch() )   return CURT_IDLE_TOUCH;
	if (isCurtSingleTouch() ) return CURT_SINGLE_TOUCH;
	if (isCurtOneHandDoubleTouch() ) return CURT_1HAND2_TOUCH;
	if (isCurtTwoHandDoubleTouch() ) return CURT_2HAND2_TOUCH;

	return CURT_OTHER_TOUCH;
}