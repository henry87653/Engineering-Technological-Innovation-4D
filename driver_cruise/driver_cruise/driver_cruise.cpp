//only for test

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <cmath>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_cruise";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberCruise" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
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

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
//******************************************************//

//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum=0;//sum of direction error(Integration)      //
// Speed Control Variables								     //
circle CircleSpeed;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum=0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta=20;												 //
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 2;									//
double tmp;												//
bool flag=true;											//
double offset=0;										//
double Tmp = 0;
bool BreakFlag = 0;
int theflag1 = 0;
circle CircleNear;
circle CircleMiddle;
circle CircleFar;
circle CircleFoot;
int speedmode = 1;
float expectspeed = 0;
int accuc = 0;
int roadTypeJudge = 0;
int RoadTypeFlag = 0;
#define xerror _midline[0][0]
int Timer = 0;
float StartErrorSum = 0;
float ExpectSpeed = 0;

//******************************************************//

//******************************Helping Functions*******************************//
//Function deviation:
//		Calculate the arctan of the degree between y axis and segment of midline[k][0] and midline[k][1]
float deviation(int k);
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear);													//
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary,double input);		//
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
//******************************************************************************//

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	//***************************************//
	//初始参数设置//
	float thedegree = atan2(_midline[30][0], _midline[30][1]);
	double minc = 0;
	//startPoint = min(160, _speed * speedmode*0.15);
	startPoint = _speed * 0.25;
	CircleSpeed = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
	CircleNear = getR(_midline[10][0], _midline[10][1], _midline[20][0], _midline[20][1], _midline[30][0], _midline[30][1]);
	CircleMiddle = getR(_midline[10][0], _midline[10][1], _midline[30][0], _midline[30][1], _midline[50][0], _midline[50][1]);
	CircleFar = getR(_midline[70][0], _midline[70][1], _midline[90][0], _midline[90][1], _midline[110][0], _midline[110][1]);
	CircleFoot = getR(_midline[1][0], _midline[1][1], _midline[2][0], _midline[2][1], _midline[3][0], _midline[3][1]);

	float BendingFoot = (500 - CircleFoot.r) / 500;
	float BendingNear = (500 - CircleNear.r) / 500;
	float BendingMiddle = (500 - CircleMiddle.r) / 500;
	float BendingFar = (500 - CircleFar.r) / 500;
	float BendingSpeed = (500 - CircleSpeed.r) / 500;
	

	//printf("speed %f BendingFoot %f Xerror %f  deviation[1] %f cmdSteer %f start time %d  start error %f\n",_speed,BendingFoot, _midline[0][0], deviation(1), *cmdSteer,Timer,StartErrorSum);
	//printf("BendingFoot %f BendingNear %f BendingMiddle %f BendingFar %f BendingSpeed %f \n deviation(1) %f cmdSteer %f \n", BendingFoot, BendingNear,BendingMiddle,BendingFar,BendingSpeed,deviation(1), *cmdSteer);
	printf("Foot %f Near %f Middle %f Far %f Speed %f \n speed %f steer %f brake %f\n", BendingFoot, BendingNear, BendingMiddle, BendingFar, BendingSpeed,_speed,*cmdSteer,*cmdBrake);

	*cmdAcc = 0.3;
	*cmdBrake = 0;
	*cmdGear = 1;
	*cmdSteer = -deviation(1)+0.3*_yaw;



	//start up(unfinished)
	/*if (_speed < 20 ||abs(xerror) > 0.5) {
		*cmdAcc = 0.5;
		//*cmdSteer = (-1*deviation(1) + 0.3*_yaw);
		*cmdSteer = -1 * deviation(1) +0.3*_yaw ;
		//Timer++;
		//StartErrorSum += abs(xerror);
	}*/

	//SpeedUp
	if ((ExpectSpeed - _speed) > 10) {
		updateGear(cmdGear);
	}


	//Judge bending
	if (BendingFoot == BendingNear == BendingMiddle == BendingFar == 0) {
		ExpectSpeed = 150;
	}
	else {
		ExpectSpeed = 80;
	}

	if (BendingNear > 0.8 && BendingMiddle > 0.8) {
		ExpectSpeed = 60;
	}
	if (BendingFoot > 0.8 && BendingNear > 0.8&& BendingMiddle > 0.8) {
		ExpectSpeed = 40;
	}
	if (BendingFar > 0.5 && BendingMiddle == 0) {
		ExpectSpeed = 100;
	}
	

	//Brake
	if (_speed > ExpectSpeed) {
		*cmdBrake = (_speed - ExpectSpeed) / 5;
	}
	if (*cmdSteer > 1) {
		ExpectSpeed = 40;
	}






}



//Someone's code is below
/*
	minc = min(min(CircleSpeed.r, CircleMiddle.r), CircleFar.r);
	//printf("%d \n", roadTypeJudge);
	if (_speed < 20) { *cmdGear = 1; *cmdAcc = 1; *cmdBrake = 0; *cmdSteer =0.3*(_yaw - 3.5 * atan2(_midline[1][0],_midline[1][1])); ++roadTypeJudge; }		//起步阶段，what is roadtypejudge?
	else {
		if (roadTypeJudge < 95 || (CircleFoot.r<100 && RoadTypeFlag == 0))						//???????
		{
			//***************************************
			//速度控制模块//
			if (accuc == 0)														
			{
				if (_speed < 80) { speedmode = 1; }						//一般情况
				//else if(c1.r>300 && c2.r > 300 && c.r > 300) { speedmode = 3; }100 100 100
				else if (CircleMiddle.r>60 && CircleFar.r > 60 && CircleSpeed.r > 60) { speedmode = 3; }					//only when the road ahead is straight
				else { speedmode = 1; }
			}
			switch (speedmode)
			{
			case 1:
			{
				expectspeed = min(1.5*CircleNear.r, 80);//max speed
				if (CircleNear.r < 50)*cmdSteer = 2 * (_yaw - 3.0 *atan2(_midline[1][0], _midline[1][1]));
				else if (CircleNear.r < 80) {*cmdSteer = 2 * (_yaw - 2.9 *atan2(_midline[1][0], _midline[1][1]) - 0.1*(_midline[5][0])); expectspeed = min(1.5*CircleNear.r, 80); }
				else if (CircleNear.r > 120){ *cmdSteer = 2 * (_yaw - 2.8 *atan2(_midline[1][0], _midline[1][1]) - 0.2*_midline[5][0]); expectspeed = min(1.5*CircleNear.r, 80) ; }
				else { *cmdSteer = 2 * (_yaw - 2.8 *atan2(_midline[1][0], _midline[1][1]) - 0.2*_midline[5][0]); expectspeed = min(1.5*CircleNear.r, 80); }
				if (CircleNear.r < 65) accuc = 1;
				else accuc = 0;
				break;
			}
			case 3:
			{
				expectspeed = min(0.8 * min(CircleSpeed.r, CircleFar.r), 160);
				*cmdSteer = 2* (_yaw - 3.2  *atan2(_midline[1][0], _midline[1][1])-0.1*_midline[7][0]);
				if (min(CircleMiddle.r, min(CircleSpeed.r, CircleFar.r)) < 350 || _speed > expectspeed)
				{
					speedmode = 1;
					accuc = 1;
				}
				else
					accuc = 0;
				break;
			}
			}
			printf("%f \n", _midline[0][0]);
			//***************************************
			//油门控制模块//
			*cmdAcc = 0.2;
			//************************************
			//刹车控制模块//														//每隔一帧踩一次刹车
			if (_speed > expectspeed  && BreakFlag == 0)
			{
				*cmdBrake = (_speed - expectspeed) / 80;
				BreakFlag = 1;
			}
			else
			{
				*cmdBrake = 0;
				BreakFlag = 0;
			}
			if (abs(*cmdSteer) > 0.2)	*cmdBrake = 2* (*cmdBrake) / 3;				//转向很大，刹车减小
			//***************************************
			updateGear(cmdGear);
			roadTypeJudge = 94;
		}
		else
		{
			RoadTypeFlag = 1;
			//***************************************
			//速度控制模块//

			if (accuc == 0)
			{
				if (_speed < 68) { speedmode = 1; }
				else if (CircleMiddle.r>300 && CircleFar.r > 300 && CircleSpeed.r > 300) { speedmode = 3; }
				else { speedmode = 1; }
			}
			switch (speedmode)
			{
			case 1:
			{
				expectspeed = min(1.9*CircleNear.r, 65);
				if (CircleNear.r<50)*cmdSteer = (_yaw - 9 * atan2(_midline[1][0], _midline[1][1]) - 0.1*_midline[5][0]);
				else if (CircleNear.r < 80) *cmdSteer = _yaw - 8.9 * atan2(_midline[1][0], _midline[1][1]) - 0.1*_midline[5][0];
				else *cmdSteer = (_yaw - 8.8 * atan2(_midline[1][0], _midline[1][1])) - 0.2*_midline[5][0];
				//*cmdSteer = (_yaw - 9 * atan2(_midline[1][0],_midline[1][1]));
				if (CircleNear.r < 65) accuc = 1;
				else accuc = 0;
				break;
			}
			case 3:
			{
				expectspeed = min(0.7*((min(CircleSpeed.r, CircleFar.r)) - 30), 160);
				//*cmdSteer = (_yaw - 3 * atan2(_midline[12][0], _midline[12][1]));
				*cmdSteer = (_yaw - 8.9 * atan2(_midline[1][0], _midline[1][1]) - 0.6*atan2(_midline[8][0], _midline[8][1]));
				if (min(CircleMiddle.r, min(CircleSpeed.r, CircleFar.r)) < 180 || _speed > expectspeed)
				{
					speedmode = 1;
					accuc = 1;
				}
				else
					accuc = 0;
				break;
			}
			}
			//printf("%d \n", speedmode);
			//***************************************
			//油门控制模块//
			*cmdAcc = 0.17;
			//***************************************
			//刹车控制模块//
			if (_speed > expectspeed  && BreakFlag == 0)
			{
				*cmdBrake = (_speed - expectspeed) / 45;
				BreakFlag = 1;
			}
			else
			{
				*cmdBrake = 0;
				BreakFlag = 0;
			}
			if (abs(*cmdSteer) > 0.2)*cmdBrake = 2 * *cmdBrake / 3;
			//***************************************
			updateGear(cmdGear);
		}
	}
	//printf("speedmode %f c1.r %f \n", c.r, c1.r);
	
	*/


//static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
//	if (parameterSet == false)		// Initialization Part
//	{
//		PIDParamSetter();
//	}
//	else
//	{
//		// Speed Control
//		/*
//		You can modify the limited speed in this module
//		Enjoy  -_-
//		*/
//		startPoint = _speed * 0.445;
//		c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
//		if (c.r <= 60)
//		{
//			expectedSpeed = constrain(45, 200, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
//		}
//		else
//		{
//			expectedSpeed = constrain(100, 200, c.r*1.4);
//		}
//		curSpeedErr = expectedSpeed - _speed;+
//		speedErrSum = 0.1 * speedErrSum + curSpeedErr;
//		if (curSpeedErr > 0)
//		{
//
//			if (abs(*cmdSteer)<0.6)
//			{
//				*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
//				*cmdBrake = 0;
//			}
//			else if (abs(*cmdSteer)>0.70)
//			{
//				*cmdAcc = 0.005;
//				*cmdBrake = 0;
//			}
//			else
//			{
//				*cmdAcc = 0.11;
//				*cmdBrake = 0;
//			}
//
//		}
//		else if (curSpeedErr < 0)
//		{
//			*cmdBrake = constrain(0.0, 0.8, -kp_s * curSpeedErr / 5 - offset / 3);
//			*cmdAcc = 0;
//		}
//
//		updateGear(cmdGear);
//
//		/******************************************Modified by Yuan Wei********************************************/
//		/*
//		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
//		Once you have chose the error model , you can rectify the value of PID to improve your control performance.
//		Enjoy  -_-
//		*/
//		// Direction Control		
//		//set the param of PID controller
//		kp_d = 1.6;
//		ki_d = 0.02;
//		kd_d = 3;
//
//		//get the error 
//		D_err = -atan2(_midline[25][0], _midline[25][1]);//only track the aiming point on the middle line
//
//														 //the differential and integral operation 
//		D_errDiff = D_err - Tmp;
//		D_errSum = D_errSum + D_err;
//		Tmp = D_err;
//
//		//set the error and get the cmdSteer
//		*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
//
//		//print some useful info on the terminal
//		printf("D_err : %f \n", D_err);
//		printf("cmdSteer %f \n", *cmdSteer);
//		/******************************************End by Yuan Wei********************************************/
//	}
//}


float deviation(int k) {
	return atan2(_midline[k][0], _midline[k][1]);
}

void PIDParamSetter()
{
	
		kp_s=0.02;
		ki_s=0;
		kd_s=0;
		kp_d=1.35;
		ki_d=0.151;
		kd_d=0.10;
		parameterSet = true;
	
}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
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
		else if (_speed >=105 && topGear >2)
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
		else if (_speed >= 145 && topGear >3)
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
		else if (_speed >= 187 && topGear >4)
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
		else if (_speed >= 234 && topGear >5)
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

double constrain(double lowerBoundary, double upperBoundary,double input)
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
	double a,b,c,d,e,f;
    double r,x,y;
	
	a=2*(x2-x1);
    b=2*(y2-y1);
    c=x2*x2+y2*y2-x1*x1-y1*y1;
    d=2*(x3-x2);
    e=2*(y3-y2);
    f=x3*x3+y3*y3-x2*x2-y2*y2;
    x=(b*f-e*c)/(b*d-e*a);
    y=(d*c-a*f)/(b*d-e*a);
    r=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
	x=constrain(-1000.0,1000.0,x);
	y=constrain(-1000.0,1000.0,y);
	r=constrain(1.0,500.0,r);
	int sign = (x>0)?1:-1;
	circle tmp = {r,sign};
	return tmp;
}

