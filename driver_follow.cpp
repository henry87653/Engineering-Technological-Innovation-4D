/***************************************************************************
	Copyright (C) 2019
	All rights reserved
	file : driver_cruise.cpp
	description :test error function
	version: 1.0.6
	Based on Huang's code. Modified by Lu at 19:29 April 2 2019.
modified by Lu at  March/31/2019 14:29
	https://github.com/henry87653/Engineering-Technological-Innovation-4D
 ***************************************************************************/
 /*
	  WARNING !
	  DO NOT MODIFY CODES BELOW!
 */
#ifdef _WIN32
#include <windows.h>
#endif
#include "driver_follow.h"
static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);
// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_follow";	// name of the module (short).
	modInfo[0].desc = "user module for CyberFollower";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}
// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}
/*
	 WARNING!
	 DO NOT MODIFY CODES ABOVE!
*/
/*
	define your variables here.
	following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;
const int topGear = 6;
float distance;
float expectedDistance;
float leaderAcc;
float leaderSpeed;

float lastDistance;
float lastLeaderSpeed;

float S_err = 0;
float S_errSum = 0;
float S_errDiff = 0;
float kp_s;	//kp for speed							     //
float ki_s;	//ki for speed							     //
float kd_s;	//kd for speed							     //

float D_err = 0;
float D_errDiff = 0;
float D_errSum = 0;
float kp_dr;//kp for direction						     //
float ki_dr;//ki for direction					    	 //
float kd_dr;//kd for direction						     //

float Y_err = 0;
float Y_errSum = 0;
float Y_errDiff = 0;
float kp_y;	//kp for y      						     //
float ki_y;	//ki for y      						     //
float kd_y;	//kd for y      						     //
float Tmp;

int total_T = 0;
void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	/* you can modify the print code here to show what you want */
}
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	distance = pow((_Leader_X*_Leader_X + _Leader_Y * _Leader_Y), 0.5);
	leaderSpeed = _speed + (distance - lastDistance) * 180;
	lastDistance = distance;
	leaderAcc = (leaderSpeed - lastLeaderSpeed) / 0.02;
	lastLeaderSpeed = leaderSpeed;
	
	S_err = _speed - leaderSpeed;
	

	//方向pid模型
	kp_dr = 1;
	ki_dr = 0.2;
	kd_dr = 0.5;
	if (total_T < 5000)//at the begining (initial)
	{
		D_err = -atan2(_Leader_X, _Leader_Y);
	}
	else
	{
		D_err = 2 * (_yaw - 3 * atan2(_Leader_X, _Leader_Y));
		D_errSum = 0.2 * D_errSum + D_err;
	}
	*cmdSteer = constrain(-1.0, 1.0, 0.5* (kp_dr * D_err + ki_dr * D_errSum + kd_dr * D_errDiff) + 0.5 * (_yaw - 8 * atan2(_Leader_X, _Leader_Y)));
	
	printf("leader_y %.3f  acc %.3f  brake%.3f \n", _Leader_Y,*cmdAcc,*cmdBrake);


	//y方向模型
	kp_y= 0.06;
	ki_y = 0.1;
	kd_y =0.1;
	Y_err = (_Leader_Y-10);
	Y_errDiff = Y_err - Tmp;
	Y_errSum = 0.2 * Y_errSum + Y_err;
	Tmp = Y_err;

	float value= constrain(0, 1.0, kp_y * Y_err + ki_y * Y_errSum + kd_y * Y_errDiff);

	if (Y_err>4)
	{
		*cmdAcc = value;
		//*cmdBrake= constrain(0, 1.0, *cmdBrake - 0.1);
		*cmdBrake = 0;
	}
	else
	{
		*cmdBrake = 1-value;
		//*cmdAcc= constrain(0, 1.0,*cmdAcc-0.1);
		*cmdAcc = 0;
	}
	updateGear(cmdGear);
	total_T += 1;
	//printf("curError:%.2f\t", curError);
	//printf("totalError:%.2f\t", totalError);
	//printf("_Leader_X:%.2f\n", _Leader_X);
	//printf("_Leader_Y:%.2f\n", _Leader_Y);
	//printf("total_T:%.2f\n", total_T);
}
void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear > 1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear > 2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear > 3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear > 4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear > 5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}