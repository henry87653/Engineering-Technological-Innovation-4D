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
float aacc = 0;
float vvel = 0;
float x[3];
float y[3];
void updateGear(int *cmdGear);
/*
	 WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/*
	define your variables here.
	following are just examples
*/

const int topGear = 6;

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;
float leaderAcc = 0;
float leaderSpeed = 0;
float s = 0; //leader distance
float lastTimeDistance = 0;
float lastTimeLeaderSpeed = 0;
double D_err = 0;// (distance error)
double S_err = 0;//speed error
double D_diff = 0;//difference of D_err
double LastTimeDerr = 0;
double expectdistance;
double LXdiff = 0;//difference of _Leader_X
double LastTimeLX;//LAST TIME _Leader_X
double LXsum = 0;
double kpd = 0;
double kid = 0;
double kdd = 0;
double kAcc = 0;
double L = 0;
double H = 0;

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
	kAcc = 0.0;
	H = 4.2;
	if (_speed < 50)
	{
		L = 0;
	}
	else if (speed < 150)
	{
		L = 0.3*H*(_speed - 50) / 100;
	}
	else if (_speed < 200)
	{
		L = 0.3 * H + 0.7*H*(_speed - 150) / 50;
	}
	else if (_speed < 250)
	{
		L = H;
	}
	else if (_speed > 200)
	{
		L = H;
	}

	if (_speed < 80 && leaderAcc>30)
	{
		L = 0.3;
	}
	else if (_speed > 150 && leaderAcc < -50)
	{
		L = 5.7;
	}

	expectdistance = 10.7 + L;
	/* you can modify the print code here to show what you want */
	//printf("speed %.3f Leader XY(%.3f, %.3f)\n", _speed, _Leader_X, _Leader_Y);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */

	s = pow((_Leader_X*_Leader_X + _Leader_Y * _Leader_Y), 0.5);
	leaderSpeed = _speed + (s - lastTimeDistance) * 180;
	lastTimeDistance = s;
	leaderAcc = (leaderSpeed - lastTimeLeaderSpeed) / 0.02;
	lastTimeLeaderSpeed = leaderSpeed;
	D_err = s - expectdistance;
	S_err = _speed - leaderSpeed;
	D_diff = (D_err - LastTimeDerr) / 0.02;
	LastTimeDerr = D_err;
	LXdiff = (_Leader_X - LastTimeLX) / 0.02;
	LastTimeLX = _Leader_X;
	LXsum += _Leader_X * 0.02;


	kpd = -1 * (_speed*0.01);
	kid = 0.003*0.02;
	kdd = -0.00012;


	if (fabs(_Leader_X) < 0.2)
		LXsum = 0;




	//if (_leader_x < 0 && lxdiff < -1 * _leader_x)
	//{
	//	*cmdsteer = kpd*_leader_x+kid*lxdiff+kdd*lxsum;
	//}
	//if (_leader_x<0 && lxdiff>-1 * _leader_x)
	//{
	//	*cmdsteer = kpd * _leader_x + kid * lxdiff + kdd * lxsum;
	//}
	//if (_leader_x > 0 && lxdiff < -1 * _leader_x)
	//{
	//	*cmdsteer = kpd * _leader_x + kid * lxdiff + kdd * lxsum;
	//}
	//if (_leader_x > 0 && lxdiff > -1 * _leader_x)
	//{
	//	*cmdsteer = kpd * _leader_x + kid * lxdiff + kdd * lxsum;
	//}

	if (s < expectdistance - 0.5&&S_err>0)
	{
		*cmdAcc = 0;
		*cmdBrake = 1;
	}
	else if (s < expectdistance&&S_err < 2 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
	}
	else if (s < expectdistance && S_err>2 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 0;
		*cmdBrake = S_err / 5;
	}

	else if (s > expectdistance&&S_err < 1 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
	}
	else if (s > expectdistance && S_err > 1 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 0;
		*cmdBrake = S_err / 5;
	}

	if (leaderAcc < -50 && s < expectdistance + 0.1)
		*cmdBrake = 1;
	if (fabs(*cmdSteer) > 0.2&&_speed > 150)
	{
		*cmdBrake = S_err / 4.79;
		*cmdAcc = -1 * S_err;
	}

	updateGear(cmdGear);
	if (_speed < 10)
		*cmdGear = 1;


	//if (fabs(_Leader_X )> 1&&*cmdBrake==0)

	//	*cmdBrake = 0.08;



	*cmdSteer = (_yaw - 8 * atan2(_Leader_X, _Leader_Y));



	//if (fabs(*cmdSteer > 0.3))
	//	*cmdBrake = S_err / 5;

	/*if (-1*_midline[0][0] > ((_width / 2) - 6))
	{
		*cmdSteer = (_yaw - 8 * atan2(_Leader_X, _Leader_Y))+0.2;
	}*/



	/*printf("                               s %f \n", s);
	printf("                               expectDistance %f \n", expectdistance);
	printf("                               S_err %f \n", S_err);
	printf("                               D_err %f \n", D_err);
	printf("                               _speed %f \n", _speed);
	//printf("                               leaderSpeed %f \n", leaderSpeed);
	printf("                               leaderAcc %f \n", leaderAcc);
	printf("                               followCMDSteer %f \n", *cmdSteer);
	//printf("                               _Leader_X %f \n", _Leader_X);
	//printf("                               LXdiff %f \n", LXdiff);
	//printf("                               LXsum %f \n", LXsum);

	//printf("                               followcmdAcc %f \n", *cmdAcc);
	//printf("                               D_err %f \n", D_err);
	printf("                               followcmdBrake %f \n", *cmdBrake);
	//printf("                               S_ERR %f \n", S_err);
	//printf("                               followcmdGear %d \n", *cmdGear);*/

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
