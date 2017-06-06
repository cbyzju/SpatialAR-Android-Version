/********************************************************************
Projection time:	2017/03/20   10:55
File Name: 	Htime.cpp
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       time testing file
*********************************************************************/


#include "Htime.h"

/**************************C++ time function***********************/

/*!
@function
@abstract              construction function
@discussion
@param
@result
*/
Htime::Htime(void)
{
	running = false;
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
Htime::~Htime(void)
{
}

/*!
@function
@abstract              start timing
@discussion
@param
@result
*/
void Htime::start()
{
	t1 = clock();
	running = true;
}

/*!
@function
@abstract              reset timing
@discussion
@param
@result
*/
void Htime::reset()
{
	t1=clock();
}

/*!
@function
@abstract              end timing 
@discussion
@param
@result
*/
float Htime::getClock()
{
	return ( clock()-t1 );
}


/**************************opencv time function***********************/

/*!
@function
@abstract              construction function and start timing
@discussion
@param
@result
*/
CVTIME::CVTIME()
{
	clock = cv::getTickCount();
}

/*!
@function
@abstract              destruction function
@discussion
@param
@result
*/
CVTIME::~CVTIME()
{
}

/*!
@function
@abstract              reset timing
@discussion
@param
@result
*/
void CVTIME::reset()
{
	clock = cv::getTickCount();
}

/*!
@function
@abstract              end timing
@discussion
@param
@result
*/
double CVTIME::getClock()
{
	clock = cv::getTickCount() - clock;
	return clock / cv::getTickFrequency() * 1000;
}