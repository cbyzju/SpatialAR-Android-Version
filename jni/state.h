/********************************************************************
Projection time:	2017/02/21   10:55
File Name: 	state.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       procamera interactive base state file
*********************************************************************/


#ifndef _STATE_  
#define _STATE_  

#include <iostream>
//#include "role.h"
using namespace std;

class Role;
// 定义状态基类
class State
{
public:
	State();
	~State();

public:
	virtual void Excute(Role*) = 0;
	virtual void dealIdelTouch(Role*)   = 0;
	virtual void dealSingleTouch(Role*) = 0;
	virtual void dealOneHandDoubleTouch(Role*) = 0;
	virtual void dealTwoHandDoubleTouch(Role*) = 0;

public:
	string getDescription() const;
	void setDescription(string);
private:
	//描述当前状态的字符串
	string   stateDescription;     
};

#endif