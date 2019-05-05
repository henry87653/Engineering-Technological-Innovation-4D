 /***************************************************************************
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :
	 version: 1.1.8
	 modified by Lu at  May/5/2019 10:28
	 https://github.com/henry87653/Engineering-Technological-Innovation-4D
  ***************************************************************************/

/*
     WARNING !
    
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_parking";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberParking" ;	// Description of the module (can be long).
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

static int flag = 0;//类型判断，1-5;5：巡线，4：靠右，3：转弯进车位，2：朝向与车位几乎一致
static float k,b,dist;
static int flagt = 0;

int topGear = 2;

//float distance = 0;//车辆与车位中心点距离
bool isEscaping = false;

static float targetX, targetY, targetDistance, vertParkdist, distance, angle;
/***********************变量解释*************************
parkdist//parkdist:车辆中心点与车位方向直线的垂直距离
distance//distance车辆中心点与车位中心点的距离
haltX//haltX,haltY应该是对于_lotX,_lotY的修正？我的理解_lotX,_lotY是目标值；haltX,haltY相当于预瞄点（应该是魔改参数最后调出来的）
haltY//如果直接用_lotX,_lotY作为预瞄点可能会有问题
midlined//车辆与预瞄点之间的距离
angle//角度误差；angle = _lotAngle - _caryaw;
***********************变量解释*************************/

static float parkDist[5], avgPark, parkAngle[5], avgAngle;
/*
parkDist[0]//parkDist[0]到parkDist[4]保存5个parkdist（车辆中心点与车位方向直线的垂直距离）,parkDist[4]为最新
parkAngle[0]//parkAngle[0]到parkAngle[4]保存5个angle（角度误差）,parkAngle[4]为最新
avgPark//parkDist[0]到parkDist[4]的平均值
avgAngle//parkAngle[0]到parkAngle[4]的平均值
*/
static int state = 0;
static bool isBackOff = false;//is back car start? (only change once)
static bool isStop = false, isTurnRight = false, isFirstStop = false, isStartLeftShift = false;
/*
Stop//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
TurnRight//入库之前的右转开始
FirstStop//右转完成，开始倒车flag
startLeftShift);//接近车位，开始向左调整flag，留出位置
*/
//int ass = 0; //是否训练车位。若车位1到5#，则ass=1；否则ass=0；

/*
float
X1 = 168.36, Y1 = 138.54,
X2 = 149.51, Y2 = 138.57,
X3 = 31.55, Y3 = 183.43,
X4 = 29.49, Y4 = 346.81,
X5 = 44.61, Y5 = 397.07;
*/

//引入辅助类，函数
typedef struct Circle
{
	double r;
	int sign;//向左-1，向右1
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3);
void shiftForward(float arr[], float value);
float getAverage(float arr[]);
//circle CircleSpeed, CircleNear, CircleMiddle, CircleFar, CircleFoot;
circle CircleFoot;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;//车位中心点绝对坐标
	_lotY = lotY;//车位中心点绝对坐标
	_lotAngle = lotAngle;//车位绝对朝向
	_bFrontIn = bFrontIn;//似乎无用？
	_carX = carX;//当前车辆绝对坐标
	_carY = carY;//当前车辆绝对坐标
	_caryaw = caryaw;//当前车辆绝对朝向
	//沿道路中线 k 米处的相对于当前车辆的 XY 坐标值
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;//偏航角（弧度）（相对于道路中线）
	_yawrate = yawrate;//角速度（弧度/秒）
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	distance = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
}

static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){   
	/* write your own code here */
	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
		dist = abs(_carX - _lotX);//车位大致沿Y方向
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);//车位延长线方向Y截矩
		dist = abs(k*_carX - _carY +b)/sqrt(k*k + 1);// k*_carX - _carY是车辆沿着车位方向Y截矩的相反数；abs（）/sqrt(k*k + 1) = * cos(_lotAngle)
	}
	distance = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
	targetX = _lotX + 10.8*cos(_lotAngle + 0.15);
	targetY = _lotY + 10.8*sin(_lotAngle + 0.15);
	targetDistance = sqrt((_carX - targetX) * (_carX - targetX) + (_carY - targetY) * (_carY - targetY));
	vertParkdist = (tan(_lotAngle)*(_carX - _lotX) - (_carY - _lotY)) / (sqrt(tan(_lotAngle)*tan(_lotAngle) + 1));
	if (cos(_lotAngle) >= 0) vertParkdist = -vertParkdist;
	angle = _lotAngle - _caryaw;
	if (angle > PI) angle -= 2 * PI;
	if (angle < -PI) angle += 2 * PI;
	shiftForward(parkDist, vertParkdist);
	shiftForward(parkAngle, angle);
	avgPark = getAverage(parkDist);
	avgAngle = getAverage(parkAngle);

	if (distance < sqrt(0.008)) isStop = true;// 0.09
	if (distance < sqrt(4000.0)) isStartLeftShift = true;//63
	if (targetDistance < 20) isTurnRight = true;// 20
	if (targetDistance < 5)  isFirstStop = true;
	
	/*CircleSpeed = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
	CircleNear = getR(_midline[10][0], _midline[10][1], _midline[20][0], _midline[20][1], _midline[30][0], _midline[30][1]);
	CircleMiddle = getR(_midline[10][0], _midline[10][1], _midline[30][0], _midline[30][1], _midline[50][0], _midline[50][1]);
	CircleFar = getR(_midline[70][0], _midline[70][1], _midline[90][0], _midline[90][1], _midline[110][0], _midline[110][1]);*/
	CircleFoot = getR(_midline[0][0], _midline[0][1], _midline[1][0], _midline[1][1], _midline[2][0], _midline[2][1]);

	//Liu-未完成停车时
	if (!*bFinished) {
		if (isStop) {
			printf(" *test1* ");
			*cmdSteer = -1;
			*cmdBrake = 1.0;
			*cmdGear = -1;
			*cmdAcc = 0.0;
			if (fabs(_speed) < 0.2)
				*bFinished = true;
		}
		else if (isBackOff) {
			printf(" *test2* ");
			float k1 = 5.61, k2 = 25, k3 = 4;//k2 = 24;;5.605095541
			
			if (89 < CircleFoot.r && CircleFoot.r < 91) {
				k1 = 5.605095541; k2 = 25; k3 = 4;
			}
			else if (99 < CircleFoot.r && CircleFoot.r < 101) {
				k1 = 5.605095541; k2 = 24; k3 = 4;
			}
			else if (149 < CircleFoot.r && CircleFoot.r < 150) {
				k1 = 5.605095541; k2 = 25; k3 = 4;
			}
			
			if (fabs(vertParkdist) > 0.5)k3 = 2;
			*cmdSteer = -k1 * angle - k2 * avgAngle / 3.14 - 1.404*(vertParkdist)-1.872*avgPark;
			/*if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) {//#1
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
			}*/

			if (fabs(_speed) > k3 * distance + 5) {//5
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
				//if (ass == 0 && fabs(parkdist) > 0.01 && fabs(angle) > 0.01)*cmdAcc = 0.6;
			}
		}
		else if (isTurnRight) {
			//printf(" *test5* ");
			*cmdGear = 1;
			if (!isBackOff && !isFirstStop) {
				printf(" *test6* ");
				*cmdSteer = -0.5*fabs(atan2(targetX - _carX, targetY - _carY));
				if (_speed < targetDistance) *cmdAcc = 0.2, *cmdBrake = 0;
				else *cmdAcc = 0, *cmdBrake = 0.2;
			}
			if (!isBackOff && (isFirstStop || fabs(angle) < 0.3)) {
				printf(" *test7* ");
				float k4 = 1.0, k5 = 0.2, k6 = 0.04;//modified
				
				if (89 < CircleFoot.r && CircleFoot.r < 91) {
					k4 = 1.0; k5 = 0.8998; k6 = 0.04;
				}
				else if (99 < CircleFoot.r && CircleFoot.r < 101) {
					k4 = 0.2; k5 = 0.2; k6 = 0.04;
				}
				else if (149 < CircleFoot.r && CircleFoot.r < 150) {
					k4 = 1.05; k5 = 0.2; k6 = 0.04;
				}
				
				/*
				if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) k5 = 0.1; //#1
				if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) k4 = 1.0201; //#2 
				if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) k5 = 0.8998;  //#3
				if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) k4 = 0.2;  //#4
				if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) k4 = 1.05; //#5
				*/
				*cmdSteer = (k4*_caryaw - (_lotAngle + 0.61)) / 3.14;
				*cmdBrake = k5 * _speed + k6 * targetDistance + 0.2;
				*cmdAcc = 0;

				if (_speed < 1) isBackOff = true;
			}
		}
		else if (isStartLeftShift) {
			printf(" *test8* ");
			float k7 = 1, k8 = 0.0, k9 = 4, k10 = 2.3;//3.5、1.5???ldx
			
			if (89 < CircleFoot.r && CircleFoot.r < 91) {
				k10 = 2.3;
			}
			else if (99 < CircleFoot.r && CircleFoot.r < 101) {
				k10 = 1.779;
			}
			else if (149 < CircleFoot.r && CircleFoot.r < 150) {
				k10 = 2.3;
			}
			
			//*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 2.5, _midline[20][1])) / 3.14;
			/*
			if (fabs(_lotX - X1) < 1 && fabs(_lotY - Y1) < 1) {
				k7 = 1;
				*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 3.5, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X2) < 1 && fabs(_lotY - Y2) < 1) {
				k7 = 1.1, k8 = 0.28;
				*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 1.5, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1) {
				k8 = 0.002;
				*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 2.3, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1) {
				k7 = 1.1, k8 = 0.28;
				*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 1.779, _midline[20][1])) / 3.14;
			}
			if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1) {
				k7 = 0.95, k9 = 4;
				*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - 2.3, _midline[20][1])) / 3.14;
				*cmdAcc = 0.18;
			}
			*/
			*cmdAcc = 0.2;
			
			if (89 < CircleFoot.r && CircleFoot.r < 91) {
				//to do
			}
			else if (99 < CircleFoot.r && CircleFoot.r < 101) {
				*cmdAcc = 0.18;
			}
			else if (149 < CircleFoot.r && CircleFoot.r < 150) {
				//to do
			}
			
			*cmdSteer = (-k9 * atan2(_midline[20][0] - _width * k8 - k10, _midline[20][1])) / 3.14;
			*cmdGear = 2;
			*cmdBrake = 0;
		}
		else{
			printf(" *test9* ");
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdSteer = (_yaw - 8 * atan2(_midline[10][0] - 1.5, _midline[10][1])) / 3.14;
			*cmdGear = 1;
		}
	}
	
	//助教-已经完成停车时
	/*if(*bFinished)//停车完成后，出车位跑
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10 ) 
		{
			printf("  test7  ");
			//接近停车位时，控制车的朝向与车位一致，速度控制在2
			*cmdSteer = -20*(_lotAngle -_caryaw)/ PI;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2 && !isEscaping )
		{
			printf("  test8  ");
			//较接近停车位时，给一个大的转向，速度控制在10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
			if (fabs(_yaw) < 0.8) isEscaping = true;
		}
		else 
		{
			printf("  test9  ");
			//其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
		    *cmdBrake = 0;//无刹车
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/ PI;//设定舵机方向
			updateGear(cmdGear); //*cmdGear = 1;//档位始终挂1
	    }
	}*/
	
	//Liu-已经完成停车时
	if (*bFinished) {//车尾入库，停车完成后，出车位跑
		printf(" *test10* ");
		//CircleFoot.r = ?//1#,2# -> 500;3# -> 90;4# -> 100, 5# -> 150
		float k11 = 1.6;
		/*
		if (fabs(_lotX - X3) < 1 && fabs(_lotY - Y3) < 1)  k11 = 2; //#3
		if (fabs(_lotX - X4) < 1 && fabs(_lotY - Y4) < 1)  k11 = 2; //#4
		if (fabs(_lotX - X5) < 1 && fabs(_lotY - Y5) < 1)  k11 = 2; //#5
		*/
		if(CircleFoot.r < 450) k11 = 2;

		*cmdSteer = (distance > 10) ? 0 : (_yaw - 8 * atan2(_midline[30][0] + k11 * _width, _midline[30][1])) / 3.14;
		*cmdAcc = 1;
		*cmdBrake = 0;
		updateGear(cmdGear);// *cmdGear = 1;
	}
	

	///=======================================printf functions============================================
	if (isTurnRight) {
		printf("== ");
		//printf("bFinished:%d ", *bFinished);//parking is finished?(only change once)
		//printf("flag:%d ", flag);
		//printf("flagt:%d ", flagt); 
		//printf("backcar:%d ", backcar);//is back car start? (only change once)
		//rintf("Stop:%d ", Stop);//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
		//printf("TurnRight:%d ", TurnRight);//入库之前的右转开始
		//printf("FirstStop:%d ", FirstStop);//右转完成，开始倒车flag
		//printf("startLeftShift:%d ", startLeftShift);//接近车位，开始向左调整flag，留出位置
		//printf("ass:%d ", ass);//是否训练车位。若车位1到5#，则ass=1；否则ass=0；

		//printf("speed:%.1f ", _speed);
		//printf("lotX:%.2f ", _lotX);
		//printf("lotY:%.2f ", _lotY);
		//printf("lotAngle:%.5f ", _lotAngle);
		//printf("carX:%.1f ", _carX);
		//printf("carY:%.1f ", _carY);
		//printf("caryaw:%.1f ", _caryaw);
		printf("parkdist:%.3f ", vertParkdist);//parkdist:车辆中心点与车位方向直线的垂直距离
		//printf("dist:%.1f ", dist);
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

		printf("Acc:%.3f ", *cmdAcc);
		printf("Brake:%.3f ", *cmdBrake);
		printf("*cmdGear:%d ", *cmdGear);
		printf("Steer:%.3f ", *cmdSteer);

		//printf("yaw:%.1f ", _yaw);
		//printf("isEscaping:%d ", isEscaping);
		//printf("CircleFoot.r:%.1f ", CircleFoot.r); 
		//if(CircleFoot.r < 450) printf("CircleFoot.sign:%d ", CircleFoot.sign);
	}
	printf("\n");
	///=======================================printf functions============================================

	*cmdAcc = constrain(0, 1, *cmdAcc);
	*cmdBrake = constrain(0, 1, *cmdBrake);
	*cmdSteer = constrain(-1, 1, *cmdSteer);
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
//shiftForward是把value放到arr[4]中，arr中的其他值顺次前移，arr[0]被舍弃
void shiftForward(float arr[], float value) {
	for (int i = 0; i < 4; i++)
		arr[i] = arr[i + 1];
	arr[4] = value;
}
//getAverage:取得数组arr[]中arr[0]到arr[4]一共5个元素的平均值
float getAverage(float arr[]) {
	float sum = 0;
	for (int i = 4; i >= 0; i = i - 1)
		sum = sum + arr[i];
	return sum / 5;
}
///=================helping functions from TA no need to modify============================