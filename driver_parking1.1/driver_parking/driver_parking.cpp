 /***************************************************************************
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :使用车尾入库。用相切圆的方法，考虑道路半径
	 version: 1.1.0
	 modified by Lu at  April/28/2019 16:09
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

int topGear = 6;

float distance = 0;//车辆与车位中心点距离
bool isEscaping = false;

static float haltX, haltY, midlined, parkdist, distance, angle;
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

//引入辅助类，函数
typedef struct Circle
{
	double r;
	int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3);
void push_back(float arr[], float value);
float getMean(float arr[]);
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

	/*CircleSpeed = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
	CircleNear = getR(_midline[10][0], _midline[10][1], _midline[20][0], _midline[20][1], _midline[30][0], _midline[30][1]);
	CircleMiddle = getR(_midline[10][0], _midline[10][1], _midline[30][0], _midline[30][1], _midline[50][0], _midline[50][1]);
	CircleFar = getR(_midline[70][0], _midline[70][1], _midline[90][0], _midline[90][1], _midline[110][0], _midline[110][1]);*/
	CircleFoot = getR(_midline[1][0], _midline[1][1], _midline[2][0], _midline[2][1], _midline[3][0], _midline[3][1]);
	
	if ( flagt == 1 ){//泊车完成，猛冲倒车，出车位
		printf("  test1  ");
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = -1;
		*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/ PI;//ldx:we can modify?
		//if (distance > 12) flagt = 0;
	}else
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.5 ) 
		{    
			printf("  test2  ");
			//用车速判断是否完成泊车//0~0.5
			*cmdSteer = 20*(_lotAngle -_caryaw + PI)/ PI;//ldx:we can modify? eg 15 *  // + PI
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed < 0.01){
			    *bFinished = true;
				flagt = 1;//泊车完成标志flagt = 1
			}
			else
				{*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			flag = 1; 
		}
		else if( ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10)&& ( flagt ==2 )) 
		{ 
			printf("  test3  ");
			//0.5~10
			//接近停车位时，控制车的朝向与车位一致，速度控制在2
			//*cmdSteer = 0;
			*cmdSteer = 5*(_lotAngle -_caryaw + PI)/ PI;//ldx:we can modify? eg 15 *
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			else if(_speed >0.07){*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			else{*bFinished = true;flagt=1; }//ldx: *bFinished = true 明显有问题，要注释掉?
			flag = 2;
		} 
		else if (((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2)&&(flagt!=2))
		{
			printf("  test4  ");
			//
			//较接近停车位时，给一个大的转向，速度控制在10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			if (fabs(fabs(_caryaw)-fabs(_lotAngle))<0.015){
				flagt = 2;
			}
			flag = 3;
		} 
		else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 5000) 
		{ 
			printf("  test5  ");
			//到一定范围时，将车子调整至道路右侧，增加转弯半径，速度控制在15
			*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/ PI;
			if( _speed > 15 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
		}
		else
		{
			printf("  test6  ");
			//其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
		    *cmdBrake = 0;//无刹车
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/ PI;//设定舵机方向//ldx: we can modify? ex. midline[10]
		    *cmdGear = 1;//档位始终挂1
			flag = 5;
	    }
	}

	
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
	
	if (*bFinished) {//车尾入库，停车完成后，出车位跑
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
	printf("== ");
	printf("bFinished:%d ", *bFinished);//parking is finished?(only change once)
	printf("flag:%d ", flag);
	printf("flagt:%d ", flagt); 
	printf("backcar:%d ", backcar);//is back car start? (only change once)
	rintf("Stop:%d ", Stop);//parking is finished?(only change once)比bFinished置1更早一点（约6个周期）
	printf("TurnRight:%d ", TurnRight);//入库之前的右转开始
	printf("FirstStop:%d ", FirstStop);//右转完成，开始倒车flag
	printf("startLeftShift:%d ", startLeftShift);//接近车位，开始向左调整flag，留出位置
	printf("ass:%d ", ass);//是否训练车位。若车位1到5#，则ass=1；否则ass=0；

	printf("speed:%.1f ", _speed);
	printf("lotX:%.2f ", _lotX);
	printf("lotY:%.2f ", _lotY);
	printf("lotAngle:%.5f ", _lotAngle);
	printf("carX:%.1f ", _carX);
	printf("carY:%.1f ", _carY);
	printf("caryaw:%.1f ", _caryaw);
	printf("parkdist:%.1f ", parkdist);//parkdist:车辆中心点与车位方向直线的垂直距离
	printf("dist:%.1f ", dist);
	printf("distance:%.7f ", distance);//distance车辆中心点与车位中心点的距离
	printf("haltX:%.1f ", haltX);//haltX,haltY应该是对于_lotX,_lotY的修正？我的理解_lotX,_lotY是目标值；haltX,haltY相当于预瞄点（应该是魔改参数最后调出来的）
	printf("haltY:%.1f ", haltY);//如果直接用_lotX,_lotY作为预瞄点可能会有问题
	printf("midlined:%.1f ", midlined);//车辆与预瞄点之间的距离
	printf("angle:%.1f ", angle);//角度误差；angle = _lotAngle - _caryaw;

	printf("parkDist[0]:%.1f ", parkDist[0]); //parkDist[0]到parkDist[4]保存5个parkdist（车辆中心点与车位方向直线的垂直距离）,parkDist[4]为最新
	printf("[1]:%.1f ", parkDist[1]);
	printf("[2]:%.1f ", parkDist[2]);
	printf("[3]:%.1f ", parkDist[3]);
	printf("[4]:%.1f  ", parkDist[4]);

	printf("parkAngle[0]:%.1f ", parkAngle[0]);//parkAngle[0]到parkAngle[4]保存5个angle（角度误差）,parkAngle[4]为最新
	printf("[1]:%.1f ", parkAngle[1]);
	printf("[2]:%.1f ", parkAngle[2]);
	printf("[3]:%.1f ", parkAngle[3]);
	printf("[4]:%.1f  ", parkAngle[4]);

	printf("avgPark:%.1f ", avgPark);//parkDist[0]到parkDist[4]的平均值
	printf("avgAngle:%.1f ", avgAngle);//parkAngle[0]到parkAngle[4]的平均值
	printf("state :%d ", state);//作用未知1# state全程=0

	printf("Acc:%.1f ", *cmdAcc);
	printf("Brake:%.1f ", *cmdBrake);
	printf("*cmdGear:%d ", *cmdGear);
	printf("Steer:%.1f ", *cmdSteer);

	printf("yaw:%.1f ", _yaw);
	printf("isEscaping:%d ", isEscaping);

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