/***************************************************************************

				 file : driver_cruise.cpp
	description : user module for CyberFollow

 ***************************************************************************/

 /*
	  WARNING !

	  DO NOT MODIFY CODES BELOW!
 */

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_lead.h"

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_lead(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_lead";	// name of the module (short).
	modInfo[0].desc = "leader module for CyberFollow";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tLeaderItf *itf = (tLeaderItf *)pt;
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
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _DistanceFromStart;
static int _gearbox, _laps;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps) {
	/* write your own code here */

	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	_DistanceFromStart = DistanceFromStart;
	_laps = laps;

	//printf("speed %f DFS %f lap %d 10m far target(%f, %f)\n", _speed, _DistanceFromStart, _laps, _midline[10][0], _midline[10][1]);

}
const int topGear = 6;									//
static float ki;
static float k;//定义变量

void updateGear(int *cmdGear);

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	/*******************************路径设计********************************/
	switch (_laps)
	{
	case 0:
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = 1;
		ki = 0;
		updateGear(cmdGear);
		break;
	case 1:
		if (_DistanceFromStart < 200)
		{
			*cmdAcc = 0.2;
			*cmdBrake = 0;
			ki = 0; 
			//printf("0-200\n");
		}
		else if (_DistanceFromStart < 700)
		{
			*cmdAcc = 0.1;
			*cmdBrake = 0;
			ki = 0;
			//printf("200 - 1000\n");
		}
		else if(_DistanceFromStart < 817)
		{
			*cmdAcc = 0;
			*cmdBrake = 0;
			ki = 0;
			//printf("200 - 1000\n");
		}
		else if (_DistanceFromStart < 850)
		{
			*cmdAcc = 0;
			*cmdBrake = 0.1;
			ki = 0;
			//printf("200 - 1000\n");
		}
		else if (_DistanceFromStart < 900)
		{
			*cmdAcc = 1;
			*cmdBrake = 0;
			ki = 0;
			//printf("GO!GO!GO\n");
		}
		else if (_DistanceFromStart < 1500)//->1500直道
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			ki = 0;
			//printf("GO!GO!GO\n");
		}
		else
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			ki = 0;
			//printf("GO!GO!GO\n");
		}
		updateGear(cmdGear);
		break;
	case 2: ki += 0.008* PI+0.001; 
		//*cmdAcc = abs(k);
		*cmdGear = 2;
		*cmdAcc = 0.8;
		*cmdBrake = 0;
		break; //第二圈曲线行驶
	case 3: ki = 0;//第三圈的时候进行一定的保护
	default:break;
	}

	k = sin(ki) / 2.0;
	//printf("k %f\n", k);

	/*******************************车辆控制********************************/

	//*cmdAcc = 0.8;//油门给80%
	//*cmdBrake = 0;//无刹车
	*cmdSteer = (_yaw - 8 * atan2(_midline[30][0] + k * _width, _midline[30][1])) / 3.14;//设定舵机方向
	//printf("steer %f\n", *cmdSteer);
	//printf("_speed %.2f\t", _speed);
	//printf("_acc %.3f\t", _acc);
	//printf("distance %f\tspeed %f\n",_DistanceFromStart,_speed);
	//printf("laps %d\n", _laps);

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
