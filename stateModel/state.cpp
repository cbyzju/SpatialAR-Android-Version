/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	state.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       procamera interactive base state file
*********************************************************************/


#include "state.h"

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
State::State()
{
	setDescription("");
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
State::~State()
{

}

/*!
@function
@abstract              set state description
@discussion
@param       string    input description
@result
*/
void State::setDescription(string des)
{
	stateDescription = des;
}

/*!
@function
@abstract              set state description
@discussion
@param      
@result		 string    output description
*/
string State::getDescription() const
{
	return stateDescription;
}