/***************************************************************************
<<<<<<< HEAD
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :对学长代码的变量输出、变量解释（在输出部分，未完成）
	 version: 0.0.5
	 modified by Lu at  April/26/2019 10:27
	 https://github.com/henry87653/Engineering-Technological-Innovation-4D
  ***************************************************************************/

  /*
  WARNING !

  DO NOT MODIFY CODES BELOW!
  */
=======

	file                 : user3.cpp
	author            : Xuangui Huang
	email              : stslxg@gmail.com
	description    :  user module for CyberParking

 ***************************************************************************/

 /*
	  WARNING !

	  DO NOT MODIFY CODES BELOW!
 */
>>>>>>> 050c65bde2cc9c5601560cefff3fcfa02e0ab6f6

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
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

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

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

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
}

static int flag = 0;
static float k, b, dist;
static int flagt = 0;

<<<<<<< HEAD
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
=======
>>>>>>> 050c65bde2cc9c5601560cefff3fcfa02e0ab6f6
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05))   //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
		dist = abs(_carX - _lotX);
	else
	{
		//k = tan(_lotAngle);
		//b = (_lotY - k * _lotX);
		dist = (_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);   //奇怪的计算公式？？先不管了
	}


	if (flagt == 1) {  //完成泊车后 倒车出库
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = -1;
		*cmdSteer = (_yaw - atan2(_midline[10][0] + _width / 3, _midline[10][1])) / PI;
	}
	else
	{

		if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 1) {    //用车速判断是否完成泊车
			*cmdSteer = 20 * (_lotAngle - _caryaw + PI) / PI;
			if (*cmdSteer > 1)
				*cmdSteer = 1;
			if (*cmdSteer < -1)
				*cmdSteer = -1;
			if (_speed < 0.01) {
				*bFinished = true;
				flagt = 1;
			}
			else
			{
				*cmdBrake = 0.1; *cmdGear = 1; *cmdAcc = 0;
			}
			flag = 1;


		}
		else if (((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10) && (flagt == 2)) { //接近停车位时，控制车的朝向与车位一致，速度控制在2
		   //*cmdSteer = 0;
			*cmdSteer = 20 * (_lotAngle - _caryaw) / PI;
			if (_speed > 2) { *cmdBrake = 0.1; *cmdGear = 1; *cmdAcc = 0; }
			else if (_speed > 0.07) { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.1; }
			//else{*bFinished = true;flagt=1; }
			flag = 2;


		}
		else if (((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist < 7.2) && (flagt != 2)) {//较接近停车位时，给一个大的转向，速度控制在10			
			*cmdSteer = 1;
			if (_speed > 10) { *cmdBrake = 0.2; *cmdGear = 1; *cmdAcc = 0; }
			else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.1; }
			if (fabs(fabs(_caryaw) - fabs(_lotAngle)) < 0.015) {
				flagt = 2;
			}
			flag = 3;


		}
		else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 5000) { //到一定范围时，将车子调整至道路右侧，增加转弯半径，速度控制在15
			*cmdSteer = (_yaw - atan2(_midline[10][0] + _width / 3, _midline[10][1])) / PI;
			if (_speed > 15) { *cmdBrake = 0.2; *cmdGear = 1; *cmdAcc = 0; }
			else { *cmdBrake = 0; *cmdGear = 1; *cmdAcc = 0.1; }
			flag = 4;


		}
		else {			                                                                         //其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
			*cmdBrake = 0;//无刹车
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / PI;//设定舵机方向
			*cmdGear = 1;//档位始终挂1
			flag = 5;


		}
	}


	if (*bFinished)
	{
		if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10) { //接近停车位时，控制车的朝向与车位一致，速度控制在2
			*cmdSteer = 20 * (_lotAngle - _caryaw) / PI;
			if (*cmdSteer > 1)
				*cmdSteer = 1;
			if (_speed > 2) { *cmdBrake = 0.1; *cmdGear = -1; *cmdAcc = 0; }
			else { *cmdBrake = 0; *cmdGear = -1; *cmdAcc = 0.1; }
		}
		else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist < 7.2) {//较接近停车位时，给一个大的转向，速度控制在10			
			*cmdSteer = 1;
			if (_speed > 10) { *cmdBrake = 0.2; *cmdGear = -1; *cmdAcc = 0; }
			else { *cmdBrake = 0; *cmdGear = -1; *cmdAcc = 0.1; }
		}
		else {			                                                                         //其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
			*cmdBrake = 0;//无刹车
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / PI;//设定舵机方向
			*cmdGear = 1;//档位始终挂1
		}
	}
<<<<<<< HEAD
	///=======================================printf functions============================================
	printf("=== ");
	//printf("bFinished:%d ", *bFinished);//parking is finished?(only change once)
	//printf("backcar:%d ", backcar);//is back car start? (only change once)
	//rintf("Stop:%d ", Stop);//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
	//printf("TurnRight:%d ", TurnRight);//入库之前的右转开始
	//printf("FirstStop:%d ", FirstStop);//右转完成，开始倒车flag
	//printf("startLeftShift:%d ", startLeftShift);//接近车位，开始向左调整flag，留出位置
	//printf("ass:%d ", ass);//没看出来，车位1#，ass全程=1

	/*printf("speed:%.1f ", _speed);
	printf("lotX:%.1f ", _lotX);
	printf("lotY:%.1f ", _lotY);
	printf("_lotAngle:%.1f ", _lotAngle);
	printf("carX:%.1f ", _carX);
	printf("carY:%.1f ", _carY);
	printf("caryaw:%.1f ", _caryaw);*/
	//printf("parkdist:%.1f ", parkdist);
	printf("distance:%.1f ", distance);
	printf("haltX:%.1f ", haltX);
	printf("haltY:%.1f ", haltY);
	/*printf("midlined:%.1f ", midlined);
	printf("angle:%.1f ", angle);

	printf("parkDist[0]:%.1f ", parkDist[0]);
	printf("[1]:%.1f ", parkDist[1]);
	printf("[2]:%.1f ", parkDist[2]);
	printf("[3]:%.1f ", parkDist[3]);
	printf("[4]:%.1f  ", parkDist[4]);

	printf("parkAngle[0]:%.1f ", parkAngle[0]);
	printf("[1]:%.1f ", parkAngle[1]);
	printf("[2]:%.1f ", parkAngle[2]);
	printf("[3]:%.1f ", parkAngle[3]);
	printf("[4]:%.1f  ", parkAngle[4]);

	printf("avgPark:%.1f ", avgPark);
	printf("avgAngle:%.1f ", avgAngle);
	printf("state :%d ", state);*/

	printf("Acc:%.1f ", *cmdAcc);
	printf("Brake:%.1f ", *cmdBrake);
	//printf("*cmdGear:%d ", *cmdGear);
	printf("Steer:%.1f ", *cmdSteer);
	//if(*bFinished)printf("\n============bFinished============\n");

	printf("yaw:%.1f ", _yaw);

	printf("\n");
	///=======================================printf functions============================================

	*cmdAcc = constrain(0, 1, *cmdAcc);
	*cmdBrake = constrain(0, 1, *cmdBrake);
	*cmdSteer = constrain(-1, 1, *cmdSteer);
}
=======



	printf("Steer:%.2f flag:%d speed:%.2f dist:%.2f lotAngle:%.2f caryaw:%.2f\n", *cmdSteer, flag, _speed, dist, _lotAngle, _caryaw);
	printf("Steer:%.2f flag:%d flagt:%d\n", *cmdSteer, flag, flagt);
}
>>>>>>> 050c65bde2cc9c5601560cefff3fcfa02e0ab6f6
