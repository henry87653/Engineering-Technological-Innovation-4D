/***************************************************************************
	Copyright (C) 2019
	All rights reserved

	file : driver_follow.cpp
	description :test error function
	version: 1.4.15

	modified by Y at  April/21/2019 15:16
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
float leaderAcc = 0;
float last1leaderAcc = 0;
float last2leaderAcc = 0;
float last3leaderAcc = 0;
float kAcc = 0.5;
float leaderSpeed;
float lastDistance;
float lastLeaderSpeed;
float D_err = 0;//Distance error
float Dr_err = 0;//Direction error
float S_err = 0;
float D_errSum = 0;
float Dr_errSum = 0;
float S_errSum = 0;
float D_errDiff = 0;
float Dr_errDiff = 0;
float S_errDiff = 0;
float LastTimeDerr = 0;
bool SpeedDown = 0;

double offset = 0;
double threshold = 5;

///pid for speed direction distance
float kp_s;	//kp for speed							     //
float ki_s;	//ki for speed							     //
float kd_s;	//kd for speed							     //
float kp_dr;//kp for direction							 //
float ki_dr;//ki for direction							 //
float kd_dr;//kd for direction							 //
float kp_d;	//kp for distance 					     	 //
float ki_d;	//ki for distance 						     //
float kd_d;	//kd for distance 					         //

//float kp_x;	//kp for x      						     //
//float ki_x;	//ki for x      						     //
//float kd_x;	//kd for x      						     //
//float kp_y;	//kp for y      						     //
//float ki_y;	//ki for y      						     //
//float kd_y;	//kd for y      						     //

//-----------------------------------------
float cmdSpeed;

//--------------------------------------------

double curError;
double totalError = 0;

double total_T = 0;
double fullLeaderAcc = 0;
double fullLeaderBrake = 0;
double leadCtrlAcc = 0;		
double leadCtrlBrake = 0;	

void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);
//==========================get paratemeters from computer==========================
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
	distance = pow((_Leader_X*_Leader_X + _Leader_Y * _Leader_Y), 0.5);//
	leaderSpeed = _speed + (distance - lastDistance) * 180;
	lastDistance = distance;
	leaderAcc = (leaderSpeed - lastLeaderSpeed) / 0.02 * cos(-atan2(_Leader_X, _Leader_Y));
	//leaderAcc = (leaderSpeed - lastLeaderSpeed) / 0.02;
	lastLeaderSpeed = leaderSpeed;
	D_err = distance - expectedDistance;
	S_err = _speed - leaderSpeed;
	last3leaderAcc = last2leaderAcc;
	last2leaderAcc = last1leaderAcc;
	last1leaderAcc = leaderAcc;

	if (last3leaderAcc < 0 && last2leaderAcc < 0 && last1leaderAcc < 0) SpeedDown = 1;
	//else if(last3leaderAcc > 0 && last2leaderAcc > 0 && last1leaderAcc > 0) SpeedDown = 0;
	else SpeedDown = 0;

	//ExpectedDistance
	//Liu's expectedDistance function
	/*if (_speed < 50)
	{
		offset = 0;
	}
	else if (_speed < 150)
	{
		offset = 0.3*threshold*(_speed - 50) / 100;
	}
	else if (_speed < 200)
	{
		offset = 0.3 * threshold + 0.7*threshold*(_speed - 150) / 50;
	}
	else if (_speed < 250)
	{
		offset = threshold;
	}
	else if (_speed > 200)
	{
		offset = threshold;
	}

	if (_speed < 80 && leaderAcc>30)//speed up
	{
		printf("\t\t\tACCELERATE!\t\t\t");
		offset = 0.3;
	}

	//if (leaderAcc < -35)//brake
	//{
	//	printf("\t\t\tBRAKE!\t\t\t");
	//	offset = -0.1 * leaderAcc + 1/(_Leader_Y - 9.899);
	//}


	//if (fabs(Dr_err) > 0.35) *cmdBrake = 1;

	//
	expectedDistance = 10.6 + offset;
	*/
	*cmdAcc = *cmdBrake = 0;//7,0.4,5
	kp_d = 7;
	ki_d = 0.4;
	kd_d = 6;
	expectedDistance = 9.9 + 0.5 + offset;
	///------------------------------------------------------------------------------------------------------
	if (leaderSpeed < 15)			fullLeaderAcc = 31.62684 + 0.30843 * leaderSpeed;
	else if (leaderSpeed < 50)		fullLeaderAcc = 45.6885 - 0.03638 * leaderSpeed;
	else if (leaderSpeed < 70)		fullLeaderAcc = 47.82208 - 0.07608 * leaderSpeed;
	else if (leaderSpeed < 90)		fullLeaderAcc = -3.41724 + 0.59887 * leaderSpeed;
	else if (leaderSpeed < 108)		fullLeaderAcc = 89.41145 - 0.44902 * leaderSpeed;
	else if (leaderSpeed < 120)		fullLeaderAcc = 22.35757 + 0.05803 * leaderSpeed;
	else if (leaderSpeed < 147.25)	fullLeaderAcc = 62.23184 - 0.22849 * leaderSpeed;
	else if (leaderSpeed < 160)		fullLeaderAcc = -41.17475 + 0.41192 * leaderSpeed;
	else if (leaderSpeed < 188.5)	fullLeaderAcc = 32.2078 - 0.06309 * leaderSpeed;
	else if (leaderSpeed < 234.77)	fullLeaderAcc = 25.91248 - 0.05626 * leaderSpeed;
	else if (leaderSpeed < 250)		fullLeaderAcc = 29.10202 - 0.08663 * leaderSpeed;
	else if (leaderSpeed < 254)		fullLeaderAcc = 14.09901 - 0.02562 * leaderSpeed;
	else							fullLeaderAcc = 7.64;

	fullLeaderBrake = -0.0008 * leaderSpeed * leaderSpeed + 0.0439 * leaderSpeed - 62.618;

	leadCtrlAcc = leaderAcc / fullLeaderAcc;
	leadCtrlBrake = leaderAcc / fullLeaderBrake;
	///---------------------------------------------------------------------------------------------
	D_err = distance - expectedDistance;
	D_errDiff = (D_err - LastTimeDerr) / 0.02;
	LastTimeDerr = D_err;
	D_errSum = 0.2 * D_errSum + D_err;

	//offset = 0.2;
	if (SpeedDown) {
		if (_speed < 130) {
			if (-40 < leaderAcc) offset = 0;
			else if (-60 < leaderAcc)offset = 3;
			else if (-75 < leaderAcc) offset = 10;
			else offset = 10;
		}
		else if (_speed < 150) {
			if (-5 < leaderAcc) offset = 0.1;
			else if (-60 < leaderAcc) offset = 2;
			else if (-65 < leaderAcc) offset = 2.5;
			else if (-75 < leaderAcc) offset = 10;
			else offset = 10;
		}
		else if (_speed < 180) {
			if (-5 < leaderAcc) offset = 0.5;
			else if (-60 < leaderAcc) offset = 2;
			else if (-70 < leaderAcc) offset = 3;
			else if (-75 < leaderAcc) offset = 10;
			else offset = 10;
		}
		else if (_speed < 200) {
			if (-10 < leaderAcc) offset = 3;
			else if (-70 < leaderAcc) offset = 4;
			else if (-75 < leaderAcc) offset = 4;
		}
		//else offset = 1 - leaderAcc / 50;
		else offset = 5;
	}
	/*else {
		if (_speed < 130) {
			 offset = 0;
		}
		else if (_speed < 150) {
			offset = 0.1;
		}
		else if (_speed < 180) {
			offset = 0.5;
		}
		else if (_speed < 200) {
			offset = 2.4;
		}
		else offset = 3;
	}*/
	else if (!SpeedDown) {
		if (_speed < 130) { offset = 0; }//0;			
		else offset = constrain(0, 4.5, 0.0025 * _speed * _speed - 0.715 * _speed + 50.405);
		//else offset = constrain(0, 5,  0.0414 * _speed - 5.3276);
	}
	if (_speed > 250) offset = 5.5;
	
	cmdSpeed = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
	if (cmdSpeed > 0) { *cmdAcc = cmdSpeed; updateGear(cmdGear); }
	else *cmdBrake = -cmdSpeed;

	//保险
	//if (expectedDistance - _Leader_Y > 1.3 || offset >= 3) { *cmdAcc = 0; *cmdBrake = 1; }
	if (expectedDistance - _Leader_Y > 1.3 ) { *cmdAcc = 0; *cmdBrake = 1; }

	if (-80 > leaderAcc && _speed > 100) { *cmdAcc = 0; *cmdBrake = 1; }		//为了11号专门打的补丁

	if (_Leader_Y < 10) { *cmdAcc /= 4; *cmdBrake *= 4; }
	if (_Leader_Y > 25 && leaderAcc > -100) { *cmdAcc = 1; *cmdBrake = 0; }				//针对被甩开打的新补丁



	//expectedDistance -----> *cmdAcc & *cmdBrake
	//0.9,0,0.6
	kp_dr = 1.1;
	ki_dr = 0.01;
	kd_dr = 0.6;


	Dr_err = 2 * (0.9 * _yaw - 9 * atan2(_Leader_X, _Leader_Y));
	//D_err = 2 * (_yaw - 7 * atan2(_Leader_X, _Leader_Y));

	Dr_errDiff = Dr_err - Dr_errSum;
	Dr_errSum = 0.2 * Dr_errSum + Dr_err;

	*cmdSteer = 1 * constrain(-1.0, 1.0, kp_dr * Dr_err + ki_dr * Dr_errSum + kd_dr * Dr_errDiff);
	//if (*cmdBrake == 1) { *cmdSteer = 0; }
	//if (*cmdAcc > 0.5 || *cmdBrake > 0.5) { *cmdSteer /= 1.2; }
	
	if (fabs(*cmdSteer) > 0.5) {
		if (_speed < 125) { *cmdAcc /= 4; *cmdBrake += 0.01; offset = 0.3; }		//7&24
		else if(leaderAcc > -70) { *cmdAcc /= 2; *cmdBrake /= 3; }
		else { *cmdAcc /= 1; *cmdBrake /= 3; }
	}
	//if (fabs(*cmdSteer) == 1) { *cmdAcc /= 2; *cmdBrake /= 1; }

	///============================ldx: defined error============================
	curError = sqrt(25 * (_Leader_X * _Leader_X) + (_Leader_Y * _Leader_Y));
	totalError += curError;
	total_T += 1;
	///============================ldx: defined error============================

	///============================printf functions to monitor varieslbes============================
	//printf("curError:%.2f\t", curError);
	//printf("totalError:%.2f\t", totalError);
	//printf("_Leader_X:%.2f\n", _Leader_X);
	//printf("threshold%f\t", threshold);
	printf("speed:%.0f  ", _speed);
	printf("Y:%.2f  ", _Leader_Y);
	//printf("total_T:%.2f\n", total_T);
	//printf("Direction_error:%f\t", Dr_err);
	//printf("offset:%f\t\t\t", offset);
	printf("lAcc:%.0f  ", leaderAcc);
	//printf("leaderSpeed:%.0f\t", leaderSpeed);
	printf("cmdAcc:%.1f  ", *cmdAcc);
	printf("brake:%.1f  ",*cmdBrake);
	printf("turn:%.2f\n", *cmdSteer);
	//printf("Dr_err:%f\t\t", Dr_err);
	//printf("yaw:%f\n",_yaw);
	//printf("offset:%.2f\n", offset);
}


///=================helping functions from TA no need to modify============================
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
///=================helping functions from TA no need to modify============================