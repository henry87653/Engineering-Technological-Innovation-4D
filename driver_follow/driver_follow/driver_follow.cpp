/***************************************************************************
	Copyright (C) 2019
	All rights reserved

	file : driver_follow.cpp
	description :fullLeaderAcc, fullLeaderBrake
			�Ľ��������������ڣ��ٶȱ仯�ϴ�LeadCtrlAcc > 0.5 || <-0.5 ����
			С�ٶ�ģ�Ͳ���׼ȷ
	version: 1.3.3

	modified by Lu at  April/17/2019 18:11
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
float kAcc = 0.5;
float leaderSpeed;
float lastDistance;
float lastLeaderSpeed;
float D_err = 0;//Distance error
float Dr_err = 0;//Direction error
float S_err = 0;//Speed error
float D_errSum = 0;
float Dr_errSum = 0;
float S_errSum = 0;
float D_errDiff = 0;
float Dr_errDiff = 0;
float S_errDiff = 0;
float LastTimeDerr = 0;
float LastTimeSerr = 0;
float LastTimeDr_err = 0;

double offset = 0;
double D_offset = 0;
double threshold = 6;//ԭ����5

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
///==============================X  Y==============================
//float kp_x;	//kp for x      						     //
//float ki_x;	//ki for x      						     //
//float kd_x;	//kd for x      						     //
//float kp_y;	//kp for y      						     //
//float ki_y;	//ki for y      						     //
//float kd_y;	//kd for y      						     //

float kp_a;	//kp for leader_Acc 					     //
float ki_a;	//ki for leader_Acc 						 //
float kd_a;	//kd for leader_Acc 					     //
float kp_h;	//kp for hybrid 					     	 //
float ki_h;	//ki for hybrid 						     //
float kd_h;	//kd for hybrid 					         //


double curError;
double totalError = 0;

double total_T = 0;

double fullLeaderAcc = 0;
double fullLeaderBrake = 0;

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
	distance = pow((_Leader_X*_Leader_X + _Leader_Y * _Leader_Y), 0.5);
	leaderSpeed = _speed + (distance - lastDistance) * 180;//�����ǶԵ�
	lastDistance = distance;
	leaderAcc = (leaderSpeed - lastLeaderSpeed) / 0.02;
	lastLeaderSpeed = leaderSpeed;
	//D_err = distance - expectedDistance;
	S_err = _speed - leaderSpeed;
	//D_errDiff = (D_err - LastTimeDerr) / 0.02;
	LastTimeDerr = D_err;

	//ExpectedDistance
	//Liu's expectedDistance function
	if (_speed < 50)
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
		offset = threshold + 2;
	}

	//if (_speed < 80 && leaderAcc>30)//speed up
	//{
	//	printf("\t\t\tACCELERATE!\t\t\t");
	//	offset = 0.3;
	//}
	//
	//if (leaderAcc < -35)//brake
	//{
	//	printf("\t\t\tBRAKE!\t\t\t");
	//	offset = -0.1 * leaderAcc + 1/(_Leader_Y - 9.899);
	//}
	
	
	//if (fabs(Dr_err) > 0.35) *cmdBrake = 1;
	
	//
	expectedDistance = 9.9 + offset;// 8.9;
	


	//============================¬��ϣ��expectedDistance -----> *cmdAcc & *cmdBrake============================
	//���¶�����D_err D_errSum D_errDiff
	D_err = distance - expectedDistance;
	//D_err = (expectedDistance - distance) / 0.02;
	D_errSum = 0.5 * D_errSum + D_err;
	D_errDiff = D_err - LastTimeSerr;
	LastTimeDerr = D_err;

	kp_d = 0.3;
	ki_d = 0;
	kd_d = 0;

	if (total_T > 0) {//initial ����Liu's code
		if (distance < expectedDistance - 0.5 && S_err>0)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			printf(" 1");
		}
		else if (distance < expectedDistance && S_err < 2 * (D_err + kAcc * leaderAcc))
		{
			*cmdAcc = 0;
			*cmdBrake = 0.2;
			printf(" 2");

		}
		else if (distance < expectedDistance && S_err>2 * (D_err + kAcc * leaderAcc))
		{
			*cmdAcc = 0;
			*cmdBrake = S_err / 5;
			printf(" 3");

		}
		else if (distance > expectedDistance && S_err < 1 * (D_err + kAcc * leaderAcc))
		{
			*cmdAcc = 1;
			*cmdBrake = 0;
			printf(" 4");

		}
		else if (distance > expectedDistance && S_err > 1 * (D_err + kAcc * leaderAcc))
		{
			*cmdAcc = 0;
			*cmdBrake = S_err / 5;
			printf(" 5");

		}
		if (leaderAcc < -50 && distance < expectedDistance + 0.1)
		{
			*cmdBrake = 1;
			printf(" 6");
		}
		if (fabs(*cmdSteer) > 0.2 && _speed > 150)
		{
			*cmdBrake = S_err / 4.79;
			*cmdAcc = -1 * S_err;
			printf(" 7");

		}
		updateGear(cmdGear);
		if (_speed < 10)
		{
			*cmdGear = 1;
			printf(" 8");
		}
	}
	/*else {
		if (leaderAcc >= 0) {
			*cmdAcc = constrain(0, 1, 1.1 * leaderAcc / fullLeaderAcc);
			*cmdBrake = 0;
		}
		else {
			*cmdAcc = 0;
			*cmdBrake = constrain(0, 1, leaderAcc / fullLeaderBrake);
		}
		updateGear(cmdGear);
	}*/
	//else {//��֮����¬��ϣ��from driver_cruise
	//	if (D_err > 0)			//distance too small
	//	{
	//		if (abs(*cmdSteer) <= 1)//-1.0 <= *cmdSteer <=  1.0; when *cmdSteer is small//ԭ����0.6
	//		{
	//			//printf("*cmdSteer small\t");
	//			*cmdAcc = constrain(0.0, 1.0, kp_d * D_err + ki_d * D_errSum + D_offset);
	//			*cmdBrake = 0;
	//		}
	//		else if (abs(*cmdSteer) > 0.70)//when *cmdSteer is large
	//		{
	//			//printf("*cmdSteer large\t");
	//			*cmdAcc = 0.005 + D_offset;
	//			*cmdBrake = 0;
	//		}
	//		else//when *cmdSteer is in the middle ( 0.6 to 0.7 )
	//		{
	//			//printf("*cmdSteer middle\t");
	//			*cmdAcc = 0.11 + D_offset;
	//			*cmdBrake = 0;
	//		}
	//	}
	//	else if (D_err < 0)		//distance too big
	//	{
	//		*cmdBrake = constrain(0.0, 1.0, -kp_d * D_err * 2 - D_offset / 3);
	//		//*cmdBrake = constrain(0.0, 0.8, -kp_d * D_err / 5 - D_offset / 3);
	//		*cmdAcc = 0;
	//	}
	//	updateGear(cmdGear);
	//}
	//============================¬��ϣ��expectedDistance -----> *cmdAcc & *cmdBrake============================

	//��============================Liu��expectedDistance -----> *cmdAcc & *cmdBrake============================
	/*if (distance < expectedDistance - 0.5 && S_err>0)
	{
		*cmdAcc = 0;
		*cmdBrake = 1;
		printf(" 1");
	}
	else if (distance < expectedDistance && S_err < 2 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 0;
		*cmdBrake = 0.2;
		printf(" 2");

	}
	else if (distance < expectedDistance && S_err>2 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 0;
		*cmdBrake = S_err / 5;
		printf(" 3");

	}
	else if (distance > expectedDistance && S_err < 1 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
		printf(" 4");

	}
	else if (distance > expectedDistance && S_err > 1 * (D_err + kAcc * leaderAcc))
	{
		*cmdAcc = 0;
		*cmdBrake = S_err / 5;
		printf(" 5");

	}
	if (leaderAcc < -50 && distance < expectedDistance + 0.1)
	{
		*cmdBrake = 1;
		printf(" 6");
	}
	if (fabs(*cmdSteer) > 0.2 && _speed > 150)
	{
		*cmdBrake = S_err / 4.79;
		*cmdAcc = -1 * S_err;
		printf(" 7");

	}
	updateGear(cmdGear);
	if (_speed < 10)
	{
		*cmdGear = 1;
		printf(" 8");
	}
	*/

	///================================PID for Direction Error ----> cmdSteer================================
	kp_dr = 1;//ԭ����1
	ki_dr = 0.1;
	kd_dr = 1;

	if (_speed < 20)//at the begining (initial)
		Dr_err = -atan2(_Leader_X, _Leader_Y);
	else
		Dr_err = 2 * (1 * _yaw - 7 * atan2(_Leader_X, _Leader_Y));
	//D_err = 2 * (_yaw - 3 * atan2(_Leader_X, _Leader_Y));

	//Dr_errDiff����Ҫ��//��������𣿣���������������//��������𣿣���������������//��������𣿣���������������//��������𣿣���������������//��������𣿣���������������
	Dr_errDiff = Dr_err - Dr_errSum;//��������𣿣���������������
	/*Dr_errDiff = Dr_err - LastTimeDr_err;
	LastTimeDr_err = Dr_err;
	*/
	Dr_errSum = 0.2 * Dr_errSum + Dr_err;

	*cmdSteer = 1 * constrain(-1.0, 1.0, kp_dr * Dr_err + ki_dr * Dr_errSum + kd_dr * Dr_errDiff);

	//*cmdSteer = 0.5 * constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff) + 0.5 * (_yaw - 8 * atan2(_Leader_X, _Leader_Y));
	//*cmdSteer = (_yaw - 8 * atan2(_Leader_X, _Leader_Y));

	/* you can modify the print code here to show what you want */
	//printf(" follow %.3f leader%.3f   XY(%.3f, %.3f)\n", _speed, leaderSpeed, _Leader_X, _Leader_Y);
	///================================PID for Direction Error ----> cmdSteer================================

	///================================PID for Speed Error ----> cmdAcc cmdBrake================================
	/*S_errSum = 1 * S_errSum + S_err;
	S_errDiff = S_err - LastTimeSerr;
	LastTimeSerr = S_err;
	
	kp_s = 1;
	ki_s = 0;
	kd_s = 0;

	if (S_err > 0)			//lackspeed
		{
			if (abs(*cmdSteer) < 0.6)//-1.0 <= *cmdSteer <=  1.0; when *cmdSteer is small
			{
				//printf("*cmdSteer small\t");
				*cmdAcc = constrain(0.0, 1.0, kp_s * S_err + ki_s * S_errSum + D_offset);
				*cmdBrake = 0;
			}
			else if (abs(*cmdSteer) > 0.70)//when *cmdSteer is large
			{
				//printf("*cmdSteer large\t");
				*cmdAcc = 0.005 + D_offset;
				*cmdBrake = 0;
			}
			else//when *cmdSteer is in the middle ( 0.6 to 0.7 )
			{
				//printf("*cmdSteer middle\t");
				*cmdAcc = 0.11 + D_offset;
				*cmdBrake = 0;
			}
	}
	else if (S_err < 0)		//overspeed
	{
		*cmdBrake = constrain(0.0, 0.8, -kp_s * S_err / 5 - D_offset / 3);
		*cmdAcc = 0;
	}*/
	///================================PID for Speed Error ----> cmdAcc cmdBrake================================

	///============================ldx: defined error============================
	curError = sqrt(25 * (_Leader_X * _Leader_X) + (_Leader_Y * _Leader_Y));
	totalError += curError;
	total_T += 1;
	///============================ldx: defined error============================

	///============================ldx: defined fullLeaderAcc; fullLeaderBrake============================
	if (leaderSpeed < 15)			fullLeaderAcc = 31.62684	+ 0.30843 * leaderSpeed;
	else if (leaderSpeed < 50)		fullLeaderAcc = 45.6885		- 0.03638 * leaderSpeed;
	else if (leaderSpeed < 70)		fullLeaderAcc = 47.82208	- 0.07608 * leaderSpeed;
	else if (leaderSpeed < 90)		fullLeaderAcc = -3.41724	+ 0.59887 * leaderSpeed;
	else if (leaderSpeed < 108)		fullLeaderAcc = 89.41145	- 0.44902 * leaderSpeed;
	else if (leaderSpeed < 120)		fullLeaderAcc = 22.35757	+ 0.05803 * leaderSpeed;
	else if (leaderSpeed < 147.25)	fullLeaderAcc = 62.23184	- 0.22849 * leaderSpeed;
	else if (leaderSpeed < 160)		fullLeaderAcc = -41.17475	+ 0.41192 * leaderSpeed;
	else if (leaderSpeed < 188.5)	fullLeaderAcc = 32.2078		- 0.06309 * leaderSpeed;
	else if (leaderSpeed < 234.77)	fullLeaderAcc = 25.91248	- 0.05626 * leaderSpeed;
	else if (leaderSpeed < 250)		fullLeaderAcc = 29.10202	- 0.08663 * leaderSpeed;
	else if (leaderSpeed < 254)		fullLeaderAcc = 14.09901	- 0.02562 * leaderSpeed;
	else							fullLeaderAcc = 7.64;

	fullLeaderBrake = -0.0008 * leaderSpeed * leaderSpeed + 0.0439 * leaderSpeed - 62.618;
	///============================ldx: defined fullLeaderAcc; fullLeaderBrake============================

	///============================printf functions to monitor varieslbes============================
	{
		//printf("curError:%.2f\t", curError);
		//printf("totalError:%.2f\t", totalError);
		//printf("_Leader_X:%.2f\n", _Leader_X);
		//printf("threshold%f\t", threshold);
		//printf("speed:%f\t", _speed);
		printf("_Leader_Y:%.2f\t", _Leader_Y);
		//printf("total_T:%.2f\n", total_T);
		//printf("Direction_error:%f\t", Dr_err);
		//printf("offset:%f\t\t", offset);

		//printf("%.2f\t", leaderSpeed);
		//printf("%.3f\n",leaderAcc);
		printf("leaderSpeed:%.0f\t", leaderSpeed);
		printf("leaderAcc%.1f\t", leaderAcc);
		//printf("cmdAcc:%f\t\t", *cmdAcc);
		//printf("cmdSteer:%f\t\t", *cmdSteer);
		//printf("cmdBrake:%f\n", *cmdBrake);
		//printf("Dr_err:%f\t\t", Dr_err);
		//printf("yaw:%f\n",_yaw);
		//printf("cmdGear:%d\t", *cmdGear);
		//printf("fullLeaderBrake%.2f\t", fullLeaderBrake);//"\t\leaderSpeed:%.0f\t", fullLeaderBrake
		//printf("fullLeaderAcc%.2f\t", fullLeaderAcc);//"\t\leaderSpeed:%.0f\t", fullLeaderAcc
		if(leaderAcc >= 0)	printf("LeadCtrlAcc%.3f\n", leaderAcc / fullLeaderAcc);
		else				printf("LeadCtrlBrake%.3f\n", leaderAcc / fullLeaderBrake);
	}
	///============================printf functions to monitor varieslbes============================
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