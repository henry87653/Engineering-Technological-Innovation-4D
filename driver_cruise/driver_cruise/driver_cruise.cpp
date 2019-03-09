/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"

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
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum=0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta=20;												 //
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag=true;											//
double offset=0;										//
double Tmp = 0;
//******************************************************//

//******************************Helping Functions*******************************//
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

<<<<<<< HEAD
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	//***************************************//
	//初始参数设置//
	float thedegree = atan2(_midline[30][0], _midline[30][1]);//they get the degree 30 m ahead (in the midline)
	double minc = 0;
	//startPoint = min(160, _speed * speedmode*0.15);
	startPoint = _speed * 0.25;
	c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
	c0 = getR(_midline[10][0], _midline[10][1], _midline[20][0], _midline[20][1], _midline[30][0], _midline[30][1]);
	c1 = getR(_midline[10][0], _midline[10][1], _midline[30][0], _midline[30][1], _midline[50][0], _midline[50][1]);
	c2 = getR(_midline[70][0], _midline[70][1], _midline[90][0], _midline[90][1], _midline[110][0], _midline[110][1]);
	c3 = getR(_midline[1][0], _midline[1][1], _midline[2][0], _midline[2][1], _midline[3][0], _midline[3][1]);

	//ldx:
	//c startPoint + 0, + delta, + 2 * delta:
	//c0 (10, 20, 30) r:
	//c1 (10, 30, 50) r:
	//c2 (70, 90, 110) r:
	//c3 (1, 2, 3) r:
	
	//ldx:they judge the road type by minimum radius of circles c, c1 and c2. Can we improve here?

	//ldx:print the paraters to see how large they are.
	printf("\n c startPoint + 0, + delta, + 2 * delta r:%6.1f \n", c.r);
	printf("c0 (10, 20, 30) r:%6.1f \n", c0.r);
	printf("c1 (10, 30, 50) r:%6.1f \n", c1.r);
	printf("c2 (70, 90, 110) r:%6.1f \n", c2.r);
	printf("c3 (1, 2, 3) r:%6.1f \n", c3.r);

	minc = min(min(c.r, c1.r), c2.r);
	printf("minc:%6.1f \n", minc);
	printf("roadTypeJudge:%d \n", roadTypeJudge);	//ldx: can we divide roads into several kinds according to a well-defined(better-defined) minc?
	if (_speed < 20) { *cmdGear = 1; *cmdAcc = 1; *cmdBrake = 0; *cmdSteer =0.3*(_yaw - 3.5 * atan2(_midline[1][0],_midline[1][1])); ++roadTypeJudge; }
	else {
		if (roadTypeJudge < 95 || (c3.r<100 && flag1 == 0))            //acc time > 95(hard to accelarate) OR nearest road not straight
		{
			//***************************************//
			//速度控制模块//
			if (accuc == 0)
			{
				if (_speed < 80) { speedmode = 1; }
				//else if(c1.r>300 && c2.r > 300 && c.r > 300) { speedmode = 3; }100 100 100
				else if (c1.r>60 && c2.r > 60 && c.r > 60) { speedmode = 3; }//speedmode = 3 only when road is straight and speed > 80
				else { speedmode = 1; }
			}
			printf("speedmode:%d", speedmode);

			switch (speedmode)
			{
			case 1:
			{
				expectspeed = min(1.5*c0.r, 80);//c0 (10, 20, 30)
				if (c0.r < 50)*cmdSteer = 2 * (_yaw - 3.0 *atan2(_midline[1][0], _midline[1][1]));
				else if (c0.r < 80) {*cmdSteer = 2 * (_yaw - 2.9 *atan2(_midline[1][0], _midline[1][1]) - 0.1*(_midline[5][0])); expectspeed = min(1.5*c0.r, 80); }
				else if (c0.r > 120){ *cmdSteer = 2 * (_yaw - 2.8 *atan2(_midline[1][0], _midline[1][1]) - 0.2*_midline[5][0]); expectspeed = min(1.5*c0.r, 80) ; }
				else { *cmdSteer = 2 * (_yaw - 2.8 *atan2(_midline[1][0], _midline[1][1]) - 0.2*_midline[5][0]); expectspeed = min(1.5*c0.r, 80); }
				if (c0.r < 65) accuc = 1;//ldx:what is accuc??
				else accuc = 0;
				break;
			}
			case 3:
			{
				expectspeed = min(0.8 * min(c.r, c2.r), 160);//c startPoint + 0, + delta, + 2 * delta; c2 (70, 90, 110)
				*cmdSteer = 2* (_yaw - 3.2  *atan2(_midline[1][0], _midline[1][1])-0.1*_midline[7][0]);
				if (min(c1.r, min(c.r, c2.r)) < 350 || _speed > expectspeed)//not straight OR overspeed
				{
					speedmode = 1;
					accuc = 1;
				}
				else
					accuc = 0;
				break;
			}
			}
			printf("expectspeed:%f \n", expectspeed);
			printf("x_error:%f \n", _midline[0][0]);
			//***************************************//
			//油门控制模块//
			*cmdAcc = 0.2;
			//************************************//
			//刹车控制模块//
			if (_speed > expectspeed  && theflag == 0)//overspeed OR what is theflag??
			{
				*cmdBrake = (_speed - expectspeed) / 80;
				theflag = 1;
			}
			else
			{
				*cmdBrake = 0;
				theflag = 0;
			}
			if (abs(*cmdSteer) > 0.2)*cmdBrake = 2* *cmdBrake / 3;//ldx:I do not understand?????
			//***************************************//
			updateGear(cmdGear);
			roadTypeJudge = 94;
=======
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	if(parameterSet==false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		//ldx:we can modify

		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-  
		*/
		startPoint = _speed * 0.445;
		c = getR(_midline[startPoint][0],_midline[startPoint][1],_midline[startPoint+delta][0],_midline[startPoint+delta][1],_midline[startPoint+2*delta][0],_midline[startPoint+2*delta][1]);
		if (c.r<=60)
		{
			expectedSpeed = constrain(45,200,c.r*c.r*(-0.046)+c.r*5.3-59.66);
>>>>>>> parent of 6760827... Replcace to Liu's Code
		}
		else									//easy to accelerate AND nearest road straight
		{
<<<<<<< HEAD
			flag1 = 1;
			//***************************************//
			//速度控制模块//

			if (accuc == 0)
			{
				if (_speed < 68) { speedmode = 1; }
				else if (c1.r>300 && c2.r > 300 && c.r > 300) { speedmode = 3; }
				else { speedmode = 1; }
			}
			switch (speedmode)
			{
			case 1:
			{
				expectspeed = min(1.9*c0.r, 65);
				if (c0.r<50)*cmdSteer = (_yaw - 9 * atan2(_midline[1][0], _midline[1][1]) - 0.1*_midline[5][0]);
				//ldx:Note that yaw and atan2 have negative signs
				else if (c0.r < 80) *cmdSteer = _yaw - 8.9 * atan2(_midline[1][0], _midline[1][1]) - 0.1*_midline[5][0];
				else *cmdSteer = (_yaw - 8.8 * atan2(_midline[1][0], _midline[1][1])) - 0.2*_midline[5][0];
				//*cmdSteer = (_yaw - 9 * atan2(_midline[1][0],_midline[1][1]));
				if (c0.r < 65) accuc = 1;
				else accuc = 0;
				break;
			}
			case 3:
=======
			expectedSpeed = constrain(100,200,c.r*1.4);
		}
		curSpeedErr = expectedSpeed - _speed;
		speedErrSum = 0.1 * speedErrSum + curSpeedErr;
		if (curSpeedErr > 0)
		{
			
			if (abs(*cmdSteer)<0.6)
>>>>>>> parent of 6760827... Replcace to Liu's Code
			{
				*cmdAcc = constrain(0.0,1.0,kp_s * curSpeedErr + ki_s * speedErrSum + offset);
				*cmdBrake = 0;
			}
<<<<<<< HEAD
			printf("speedmode:%d \n", speedmode);
			//***************************************//
			//油门控制模块//
			*cmdAcc = 0.17;
			//***************************************//
			//刹车控制模块//
			if (_speed > expectspeed  && theflag == 0)
=======
			else if (abs(*cmdSteer)>0.70)
>>>>>>> parent of 6760827... Replcace to Liu's Code
			{
				*cmdAcc = 0.005 + offset;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 0.11 + offset;
				*cmdBrake = 0;
			}
		
		}
		else if (curSpeedErr < 0)
		{
			*cmdBrake = constrain(0.0,0.8,-kp_s *curSpeedErr/5 - offset/3);
			*cmdAcc = 0;
		}

		updateGear(cmdGear);//ldx:huan dang
		
		//important algorithm below
		/******************************************Modified by Yuan Wei********************************************/
		/*
		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
		Once you have chose the error model , you can rectify the value of PID to improve your control performance.
		Enjoy  -_-  
		*/
		// Direction Control		
		//set the param of PID controller
        kp_d = 1;
        ki_d = 0;
		kd_d = 0;

		//get the error 
		D_err = -atan2(_midline[5][0],_midline[5][1]);//only track the aiming point on the middle line

		//the differential and integral operation 
		D_errDiff = D_err - Tmp;
		D_errSum = D_errSum + D_err;
		Tmp = D_err;

		//set the error and get the cmdSteer
		*cmdSteer =constrain(-1.0,1.0,kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

		//print some useful info on the terminal
		printf("D_err : %f \n", D_err);
		printf("cmdSteer %f \n", *cmdSteer);	
		/******************************************End by Yuan Wei********************************************/
	}
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

