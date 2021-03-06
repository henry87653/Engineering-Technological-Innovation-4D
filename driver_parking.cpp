﻿/***************************************************************************
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :对学长代码的变量输出、变量解释（在变量定义处）代码段判断test
	 version: 0.0.9
	 modified by Lu at  May/1/2019 0:09
	 https://github.com/henry87653/Engineering-Technological-Innovation-4D
  ***************************************************************************/

  /*
  WARNING !

  DO NOT MODIFY CODES BELOW!
  */

#ifdef _WIN32
#include <windows.h>
#endif
#include <cmath>
#include <math.h>
#include "driver_parking.h"

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;	 // Init function.
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
	printf("OK!\n");
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
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
static float haltX, haltY, midlined, parkdist, distance, angle;
/***********************变量解释*************************
parkdist//parkdist:车辆中心点与车位方向直线的垂直距离
distance//distance车辆中心点与车位中心点的距离
haltX//haltX,haltY应该是对于_lotX,_lotY的修正？我的理解_lotX,_lotY是目标值；haltX,haltY相当于预瞄点（应该是魔改参数最后调出来的）
haltY//如果直接用_lotX,_lotY作为预瞄点可能会有问题
midlined//车辆与预瞄点之间的距离
angle//角度误差；angle = _lotAngle - _caryaw;
***********************变量解释*************************/
float
X1 = 168.36, Y1 = 138.54,
X2 = 149.51, Y2 = 138.57,
X3 = 31.55, Y3 = 183.43,
X4 = 29.49, Y4 = 346.81,
X5 = 44.61, Y5 = 397.07;


static void userDriverGetParam(float lotX, float lotY, float lotAngle,
	bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2],
	float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	float a, b;//???
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

static float parkDist[5], avgPark, parkAngle[5], avgAngle;
/*
parkDist[0]//parkDist[0]到parkDist[4]保存5个parkdist（车辆中心点与车位方向直线的垂直距离）,parkDist[4]为最新
parkAngle[0]//parkAngle[0]到parkAngle[4]保存5个angle（角度误差）,parkAngle[4]为最新
avgPark//parkDist[0]到parkDist[4]的平均值
avgAngle//parkAngle[0]到parkAngle[4]的平均值
*/
static int state = 0;//作用未知
static bool backcar = false;//is back car start? (only change once)
static bool Stop = false, TurnRight = false, FirstStop = false, startLeftShift = false;
/*
Stop//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
TurnRight//入库之前的右转开始
FirstStop//右转完成，开始倒车flag
startLeftShift);//接近车位，开始向左调整flag，留出位置
*/
int ass = 0; //是否训练车位。若车位1到5#，则ass=1；否则ass=0；

///=================helping functions from TA no need to modify============================
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
//push back是把value放到arr[4]中，arr中的其他值顺次前移，arr[0]被舍弃
void push_back(float arr[], float value) {
	for (int i = 0; i < 4; i++)
		arr[i] = arr[i + 1];
	arr[4] = value;
}
//getMean:取得数组arr[]中arr[0]到arr[4]一共5个元素的平均值
float getMean(float arr[]) {
	float sum = 0;
	for (int i = 4; i >= 0; i = i - 1)
		sum = sum + arr[i];
	return sum / 5;
}
///=================helping functions from TA no need to modify============================
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/***********************变量解释*************************
	bFinished//parking is finished?(only change once)
	************************变量解释************************/
	distance = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
	haltX = _lotX + 10.8*cos(_lotAngle + 0.15);
	haltY = _lotY + 10.8*sin(_lotAngle + 0.15);
	midlined = sqrt((_carX - haltX) * (_carX - haltX) + (_carY - haltY) * (_carY - haltY));
	parkdist = (tan(_lotAngle)*(_carX - _lotX) - (_carY - _lotY)) / (sqrt(tan(_lotAngle)*tan(_lotAngle) + 1));
	if (cos(_lotAngle) >= 0) parkdist = -parkdist;
	angle = _lotAngle - _caryaw;
	if (angle > PI) angle -= 2 * PI;
	if (angle < -PI) angle += 2 * PI;
	push_back(parkDist, parkdist);
	push_back(parkAngle, angle);
	avgPark = getMean(parkDist);
	avgAngle = getMean(parkAngle);
	if (distance < sqrt(0.008)) Stop = true;
	if (distance < sqrt(4000.0)) startLeftShift = true;
	if (midlined < 20) TurnRight = true;
	if (midlined < 5)  FirstStop = true;
	if ((fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) ||
		(fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) ||
		(fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) ||
		(fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) ||
		(fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1))ass = 1;
	if (!*bFinished) {
		if (Stop) {
			printf(" *test1* ");
			*cmdSteer = -1;
			*cmdBrake = 1.0;
			*cmdGear = -1;
			*cmdAcc = 0.0;
			if (fabs(_speed) < 0.2)
				*bFinished = true;
		}
		else if (backcar) {
			printf(" *test2* ");
			float k1 = 5.605095541, k2 = 24, k3 = 4;
			if (fabs(parkdist) > 0.5)k3 = 2;
			*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) {//#1
				k3 = 4; k2 = 25;
				*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			}
			if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) {//#2 
				k3 = 4; k2 = 25;
				*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			}

			if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) {//#3
				k3 = 4; k2 = 25;
				*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			}

			if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) {//#4 
				k3 = 4; k2 = 24;
				*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			}

			if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) { //#5
				k3 = 4; k2 = 25;
				*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(parkdist)-1.872*avgPark;
			}


			if (fabs(_speed) > k3 * distance + 5) {
				printf(" *test3* ");
				*cmdBrake = 0.2;
				*cmdGear = -1;
				*cmdAcc = 0;
			}
			else {
				printf(" *test4* ");
				*cmdBrake = 0.0;
				*cmdGear = -1;
				*cmdAcc = 1;
				if (ass == 0 && fabs(parkdist) > 0.01 && fabs(angle) > 0.01)*cmdAcc = 0.6;
			}
		}
		else if (TurnRight) {
			//printf(" *test5* ");
			*cmdGear = 1;
			if (!backcar && !FirstStop) {
				printf(" *test6* ");
				*cmdSteer = -0.5*fabs(atan2(haltX - _carX, haltY - _carY));
				if (_speed < midlined) *cmdAcc = 0.2, *cmdBrake = 0;
				else *cmdAcc = 0, *cmdBrake = 0.2;
			}
			if (!backcar && (FirstStop || fabs(angle) < 0.3)) {
				printf(" *test7* ");
				float k1 = 1.0, k2 = 0.2, k3 = 0.04;
				if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) k2 = 0.1; //#1
				if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) k1 = 1.0201; //#2 
				if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) k2 = 0.8998;  //#3
				if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) k1 = 0.2;  //#4
				if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) k1 = 1.05; //#5
				*cmdSteer = (k1*_caryaw - (_lotAngle + 0.61)) / 3.14;
				*cmdBrake = k2 * _speed + k3 * midlined + 0.2;
				*cmdAcc = 0;

				if (_speed < 1) backcar = true;
			}
		}
		else if (startLeftShift) {
			printf(" *test8* ");
			*cmdAcc = 0.2;
			float k1 = 1, k2 = 0.0, k3 = 4;
			*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 2.5, _midline[20][1])) / 3.14;
			if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) {
				k1 = 1;
				*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 3.5, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) {
				k1 = 1.1, k2 = 0.28;
				*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 1.5, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) {
				k2 = 0.002;
				*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 2.3, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) {
				k1 = 1.1, k2 = 0.28;
				*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 1.779, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) {
				k1 = 0.95, k3 = 4;
				*cmdSteer = (-k3 * atan2(_midline[20][0] - _width * k2 - 2.3, _midline[20][1])) / 3.14;
				*cmdAcc = 0.18;
			}
			*cmdGear = 2;
			*cmdBrake = 0;
		}
		else {
			printf(" *test9* ");
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdSteer = (_yaw - 8 * atan2(_midline[10][0] - 1.5, _midline[10][1])) / 3.14;
			*cmdGear = 1;
		}
	}
	if (*bFinished) {
		printf(" *test10* ");
		float k1 = 1.6;
		if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1)  k1 = 2; //#3
		if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1)  k1 = 2; //#4
		if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1)  k1 = 2; //#5

		*cmdSteer = (distance > 10) ? 0 : (_yaw - 8 * atan2(_midline[30][0] + k1 * _width, _midline[30][1])) / 3.14;
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = 1;

	}
	///=======================================printf functions============================================
	if (TurnRight) {
		printf("== ");
		//printf("bFinished:%d ", *bFinished);//parking is finished?(only change once)
		//printf("backcar:%d ", backcar);//is back car start? (only change once)
		//rintf("Stop:%d ", Stop);//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
		//printf("TurnRight:%d ", TurnRight);//入库之前的右转开始
		//printf("FirstStop:%d ", FirstStop);//右转完成，开始倒车flag
		//printf("startLeftShift:%d ", startLeftShift);//接近车位，开始向左调整flag，留出位置
		//printf("ass:%d ", ass);//是否训练车位。若车位1到5#，则ass=1；否则ass=0；

		//printf("speed:%.1f ", _speed);
		/*printf("lotX:%.2f ", _lotX);
		printf("lotY:%.2f ", _lotY);
		printf("lotAngle:%.1f ", _lotAngle);
		printf("carX:%.1f ", _carX);
		printf("carY:%.1f ", _carY);*/
		//printf("caryaw:%.1f ", _caryaw);
		printf("parkdist:%.3f ", parkdist);//parkdist:车辆中心点与车位方向直线的垂直距离
		printf("distance:%.3f ", distance);//distance车辆中心点与车位中心点的距离
		//printf("haltX:%.1f ", haltX);//haltX,haltY应该是对于_lotX,_lotY的修正？我的理解_lotX,_lotY是目标值；haltX,haltY相当于预瞄点（应该是魔改参数最后调出来的）
		//printf("haltY:%.1f ", haltY);//如果直接用_lotX,_lotY作为预瞄点可能会有问题
		//printf("midlined:%.1f ", midlined);//车辆与预瞄点之间的距离
		printf("angle:%.3f ", angle);//角度误差；angle = _lotAngle - _caryaw;

		//printf("parkDist[0]:%.1f ", parkDist[0]); //parkDist[0]到parkDist[4]保存5个parkdist（车辆中心点与车位方向直线的垂直距离）,parkDist[4]为最新
		//printf("[1]:%.1f ", parkDist[1]);
		//printf("[2]:%.1f ", parkDist[2]);
		//printf("[3]:%.1f ", parkDist[3]);
		//printf("[4]:%.1f  ", parkDist[4]);

		//printf("parkAngle[0]:%.1f ", parkAngle[0]);//parkAngle[0]到parkAngle[4]保存5个angle（角度误差）,parkAngle[4]为最新
		//printf("[1]:%.1f ", parkAngle[1]);
		//printf("[2]:%.1f ", parkAngle[2]);
		//printf("[3]:%.1f ", parkAngle[3]);
		//printf("[4]:%.1f  ", parkAngle[4]);

		//printf("avgPark:%.1f ", avgPark);//parkDist[0]到parkDist[4]的平均值
		//printf("avgAngle:%.1f ", avgAngle);//parkAngle[0]到parkAngle[4]的平均值
		//printf("state :%d ", state);//作用未知1# state全程=0

		/*printf("Acc:%.3f ", *cmdAcc);
		printf("Brake:%.3f ", *cmdBrake);
		printf("*cmdGear:%d ", *cmdGear);
		printf("Steer:%.3f ", *cmdSteer);*/

		//printf("yaw:%.1f ", _yaw);
	}
	printf("\n");
	///=======================================printf functions============================================

	*cmdAcc = constrain(0, 1, *cmdAcc);
	*cmdBrake = constrain(0, 1, *cmdBrake);
	*cmdSteer = constrain(-1, 1, *cmdSteer);
}