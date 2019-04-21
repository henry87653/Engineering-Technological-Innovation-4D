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

typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;

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
int curveflag = 0;


void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);

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
	else if (leaderSpeed < 170)
	{
		offset = 0.035*(leaderSpeed - 150) + 0.02*(leaderSpeed - 50);
	}
	else if (leaderSpeed < 200)
	{
		offset = 0.04*(leaderSpeed - 150) + 0.02*(leaderSpeed - 50);
	}
	else if (leaderSpeed < 230)
	{
		offset = 5;
	}
	else if (leaderSpeed < 250)
	{
		offset =6;
	}
	else if (leaderSpeed > 200)
	{
		offset = 6.5;
	}

	//the leader-acc modify
	if(leaderAcc>0)kAcc = 0.08;
	else if(leaderSpeed<150)kAcc = 0.06;
	else if (leaderSpeed <170)kAcc = 0.12;
	else if(leaderSpeed < 200) kAcc = 0.24;
	else kAcc = 0.5;

	offset -= leaderAcc * kAcc;

/*	if (leaderAcc < -20&& leaderSpeed<70)
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
	*/

	if (leaderAcc < -50)
	{
		offset += 0.3;
	
		if (leaderSpeed > 120)
		{
			offset += 0.3;
		}
		if (leaderSpeed > 150)
		{
			offset += 0.5;
		}
		if (leaderSpeed > 170)
		{
			offset += 1;
		}

	}
	offset -= 0.1*(leaderSpeed - _speed);

	expectedDistance = constrain(9.9, 30, 10.3 + offset);


	//brake\acc control

	kp_D = 0.1;
	ki_D = 0.5;
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
	}
	else
	{
		*cmdBrake = -VALUE;
	}

	
	if (_speed - leaderSpeed > 2.5 && leaderSpeed < 7 && total_T>500)
	{
		*cmdAcc = 0;
		*cmdBrake -=0.3;
	}
	

	if (_speed - leaderSpeed < -10 && D_err>1)
	{
		*cmdAcc += 0.05* D_err;
	
	}


	if (_speed - leaderSpeed < -15 && D_err>3)
	{
		*cmdAcc += 0.1;
	}

	if (leaderSpeed < 15 && leaderAcc>30)*cmdAcc = 1;
	else if (leaderSpeed < 50 && leaderAcc>43)*cmdAcc = 1;
	else if (leaderSpeed<90&&leaderAcc>0.5*leaderSpeed-3.4)*cmdAcc = 1;
	else if (leaderSpeed<108 && leaderAcc>-0.45*leaderSpeed + 89.4)*cmdAcc = 1;
	else if (leaderSpeed < 147 && leaderAcc>28)*cmdAcc = 1;
	else if (leaderSpeed < 188 && leaderAcc>29)*cmdAcc = 1;
	else if (leaderSpeed < 235 && leaderAcc>12)*cmdAcc = 1;
	else if (leaderSpeed<260 && leaderAcc>7.6)*cmdAcc = 1;

	if (_speed - leaderSpeed > 15 && D_err < 1)
	{
		*cmdAcc -= 0.2;
		*cmdBrake += 0.2;
		if (leaderSpeed > 150)*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed > 25 && D_err < 3)
	{
		*cmdAcc -= 0.1;
		*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed > 15 && D_err < 6)
	{
		*cmdAcc -= 0.1;
		*cmdBrake += 0.3;
	}
	if (_speed - leaderSpeed > 6 && leaderSpeed < 30 && total_T>500)
	{
		*cmdAcc = 0;
		*cmdBrake -= 0.5;
	}

	if (leaderAcc < -0.0008*leaderSpeed*leaderSpeed + 0.0439*leaderSpeed - 60)
	{
		*cmdBrake = 1;
	}

	if (_Leader_Y < 10.3)    //the final protect
	{
		*cmdAcc = 0;
		*cmdBrake += 0.3;
	}
	if (_Leader_Y < 10.05)    //the final protect
	{
		*cmdAcc = 0;
		*cmdBrake =1;
	}
	


	if (distance < 13 && startflag < 20&& total_T<500)
	{
		*cmdBrake = 1;
		*cmdAcc = 0;
		startflag++;
	}
	*cmdAcc = constrain(0, 1, *cmdAcc);
	*cmdBrake = constrain(0, 1, *cmdBrake);

	


	//direction control
	circle CircleNear, CircleFar, CircleFoot;
	CircleNear = getR(_midline[10][0], _midline[10][1], _midline[20][0], _midline[20][1], _midline[30][0], _midline[30][1]);
	CircleFar = getR(_midline[20][0], _midline[20][1], _midline[40][0], _midline[40][1], _midline[60][0], _midline[60][1]);
	
	//printf("R %.2f", CircleFar.r);

	kp_d = 1;
	ki_d = 0;
	kd_d = 0.6;

	bool isCurve = false;
	

	if (_speed < 20)//at the begining (initial)
		d_err = -atan2(_Leader_X, _Leader_Y);

	else if (CircleNear.r < 200 && CircleFar.r < 120)//at the curve
	{
		isCurve = true;
		curveflag++;
		d_err = 8* _yaw - 20 * atan2(_Leader_X - 0.5, _Leader_Y);
		printf("curve%d ",curveflag);
		
	}

	else
	{
		d_err = 2 * (4 * _yaw - 8 * atan2(_Leader_X, _Leader_Y));
		isCurve = false;
		curveflag = 0;
	}

	updateGear(cmdGear);

	
	//printf("!%.3f ", d_err);
  
	d_errDiff = d_err - d_errSum;
	d_errSum = 0.2 * d_errSum + d_err;
	*cmdSteer = 1 * constrain(-1.0, 1.0, kp_d * d_err + ki_d * d_errSum + kd_d * d_errDiff);

	if (isCurve)*cmdSteer += 0.1;
	/*
	if (isCurve&&curveflag<50)
	{
		*cmdAcc -= 0.2;
		*cmdBrake += 0.2;
	}
	*/
	if (isCurve && d_err > 0.6  && _speed > 150 && _midline[0][0] < -6.2)  //prevent slipping
	{
		//*cmdAcc -= 0.3;
		*cmdSteer += 0.3;
		printf("safety");
	}
	*cmdSteer = constrain(-1, 1, *cmdSteer);
	if ((*cmdSteer == 1 || *cmdSteer == -1)&&isCurve)*cmdAcc = constrain(0, 0.5, *cmdAcc);

	printf("%.3f ", *cmdSteer);
	

	/*
	if (_midline[0][0] < -6 && CircleFar.r<250) {
		*cmdSteer -= 0.1;
		*cmdBrake = 0;
		*cmdAcc = 0.4;
		printf("!!!");
	}
	if (_midline[0][0] < -6.5&& d_err>2 && CircleNear.r < 250) {
		*cmdSteer -= 0.1;
		*cmdBrake = 0.2;
		*cmdAcc = 0.3;
		printf("!!!");
	}
	*/
	/*if (_midline[0][0] > 7 && d_err < -1) {
		d_err = 1 * (1 * _yaw - 6 * atan2(_midline[2][0] - 7, _midline[2][1] + 0.1)) + (1 * _yaw - 6 * atan2(_Leader_X, _Leader_Y));

	}
	*/
	
	printf(" %d", total_T);

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
circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b*f - e * c) / (b*d - e * a);
	y = (d*c - a * f) / (b*d - e * a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}