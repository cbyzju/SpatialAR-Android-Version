/********************************************************************
Projection time:	2017/03/20   10:55
File Name: 	Htime.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       time testing file
*********************************************************************/


#ifndef HTIME_MYTIME_CLASS
#define HTIME_MYTIME_CLASS

#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>

class Htime
{
public:
	Htime(void);
	~Htime(void);
		
	void reset();
	void start();
	float getClock();

private:
	clock_t t1;
	bool running;
};


class CVTIME
{
public:
	CVTIME();
	~CVTIME();
	
	void reset();
	double getClock();

private:
	int64 clock;
};

#endif