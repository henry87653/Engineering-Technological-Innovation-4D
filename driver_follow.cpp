/***************************************************************************
	Copyright (C) 2019
	All rights reserved

	file : driver_cruise.cpp
	description :test error function
	version: 

	modified by Huang at  April/18/2019 20:30
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
float lastLeaderAcc = 0;
float VALUE = 0;
float D_err = 0;
float d_err = 0;
float S_err = 0;
float D_errSum = 0;
float d_errSum = 0;
float S_errSum = 0;
float D_errDiff = 0;
float d_errDiff = 0;
float S_errDiff = 0;
float LastTimeDerr = 0;
float kp_s;	//kp for speed							     //
float ki_s;	//ki for speed							     //
float kd_s;	//kd for speed							     //
float kp_d;	//kp for direction						     //
float ki_d;	//ki for direction					    	 //
float kd_d;	//kd for direction						     //
float kp_D;	//kp for distance						     //
float ki_D;	//ki for distance					    	 //
float kd_D;	//kd for distance						     //
float kAcc;


double curError;
double totalError = 0;

int total_T = 0;
int startflag = 0;


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
	//printf("speed %.3f Leader XY(%.3f, %.3f)\n", _speed, _Leader_X, _Leader_Y);
}
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	distance = pow((_Leader_X*_Leader_X + _Leader_Y * _Leader_Y), 0.5);
	leaderSpeed = _speed + (distance - lastDistance) * 180;
	lastDistance = distance;
	leaderAcc = (leaderSpeed - lastLeaderSpeed) / 0.02;
	lastLeaderSpeed = leaderSpeed;

	//ExpectedDistance
	//ldx: modify offset

	//the leader-speed control
	double offset = 0;
	if (leaderSpeed < 50)
	{
		offset = 0;
	}
	else if (leaderSpeed< 150)
	{
		offset = 0.015*(leaderSpeed - 50);
	}
	else if (leaderSpeed < 180)
	{
		offset = 0.035*(leaderSpeed - 150) + 0.015*(leaderSpeed - 50);
	}
	else if (leaderSpeed < 230)
	{
		offset = 4;
	}
	else if (leaderSpeed < 250)
	{
		offset = 5;
	}
	else if (leaderSpeed > 200)
	{
		offset = 6;
	}

	//the leader-acc modify
	if(leaderAcc>0)kAcc = 0.04;
	else kAcc = 0.05;

	offset -= leaderAcc * kAcc;

	if (leaderAcc < -20&& leaderSpeed<70)
	{
		offset += 0.3;
	}
	if (leaderAcc < -30)
	{
		offset += 0.5;

		if (leaderSpeed > 140)
		{
			*cmdBrake += 0.2;
			offset += 0.3;
		}
	}
	if (leaderAcc < -50)
	{
		offset += 0.3;
	
		if (leaderSpeed > 120)
		{
			*cmdBrake += 0.3;
			offset += 0.3;
		}
		if (leaderSpeed > 170)
		{
			*cmdBrake=1;
			offset += 0.3;
		}
		printf("!!!");
	}
	offset -= 0.1*(leaderSpeed - _speed);

	expectedDistance = constrain(9.9, 20, 10.3 + offset);


	//brake\acc control

	kp_D = 0.062;
	ki_D = 0.42;
	kd_D = 0.01;

	D_err = distance - expectedDistance;
	D_errDiff = (D_err - LastTimeDerr) / 0.02;
	LastTimeDerr = D_err;
	D_errSum = 0.2 * D_errSum + D_err;

	*cmdAcc = *cmdBrake = 0;
	VALUE = constrain(-1.0, 1.0, kp_D * D_err + ki_D * D_errSum + kd_D * D_errDiff);
	if (VALUE > 0)
	{
		*cmdAcc = VALUE;
		updateGear(cmdGear);
	}

	else
	{
		*cmdBrake = -VALUE;
		updateGear(cmdGear);
	}

	if (_speed - leaderSpeed > 15 && D_err<1)
	{
		*cmdAcc -= 0.2;
		*cmdBrake += 0.2;
		if (leaderSpeed > 150)*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed >25&& D_err<3)
	{
		*cmdAcc -= 0.1;
		*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed > 15 && D_err < 6)
	{
		*cmdAcc -= 0.1;
		*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed>6 && leaderSpeed<30 && total_T>500)
	{
		*cmdAcc =0;
		*cmdBrake =1;
	}
	if (_speed - leaderSpeed > 3 && leaderSpeed < 7 && total_T>500)
	{
		*cmdAcc = 0;
		*cmdBrake = 0.3;
	}
	if (_speed - leaderSpeed < -10 && D_err>1)
	{
		*cmdAcc += 0.1;
		*cmdBrake -= 0.1;
	}
	if (_speed - leaderSpeed < -15 && D_err>3)
	{
		*cmdAcc += 0.1;
		*cmdBrake -= 0.1;
	}
	


	if (distance < 13 && startflag < 20&& total_T<500)
	{
		*cmdBrake = 1;
		*cmdAcc = 0;
		startflag++;
	}

	


	//direction control
	kp_d = 1;
	ki_d = 0;
	kd_d = 0.6;

	if (_speed < 20)//at the begining (initial)
		d_err = -atan2(_Leader_X, _Leader_Y);
	else
		d_err = 2 * (1 * _yaw - 8 * atan2(_Leader_X, _Leader_Y));

/*if (_midline[0][0] < -6.5) {
		d_err =1* (1 * _yaw - 6 * atan2(_midline[2][0] + 6.5, _midline[2][1])+0.1)+ (1 * _yaw - 6 * atan2(_Leader_X, _Leader_Y));
		printf("!!!");
	}
	if (_midline[0][0] > 7) {
		d_err = 1* (1 * _yaw - 6 * atan2(_midline[2][0] - 7, _midline[2][1]+0.1))+(1 * _yaw - 6 * atan2(_Leader_X, _Leader_Y));
		printf("!!!");
	}
*/
	d_errDiff = d_err - d_errSum;
	d_errSum = 0.2 * d_errSum + d_err;
	*cmdSteer = 1 * constrain(-1.0, 1.0, kp_d * d_err + ki_d * d_errSum + kd_d * d_errDiff);
	printf("%d %.3f %.3f", total_T, _midline[0][0], *cmdSteer);


	
	printf("%d", total_T);

	//*cmdSteer = 0.5 * constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff) + 0.5 * (_yaw - 8 * atan2(_Leader_X, _Leader_Y));
	//*cmdSteer = (_yaw - 8 * atan2(_Leader_X, _Leader_Y));

	/* you can modify the print code here to show what you want */
	total_T++;
	printf(" follow %.2f leader%.2f   XY(%.1f, %.1f)  expected%.3f acc%.2f brake%.2f\n", _speed, leaderSpeed, _Leader_X, _Leader_Y, expectedDistance,*cmdAcc,*cmdBrake);
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