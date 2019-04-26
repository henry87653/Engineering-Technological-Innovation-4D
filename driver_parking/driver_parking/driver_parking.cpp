 /***************************************************************************
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :
		助教代码跑不下来的原因：
		1.某些地方逻辑出错（1.0.1， 1.0.2已经修改）
		2.未考虑道路本身半径
		改进方向：用相切圆的方法，考虑道路半径
	 version: 1.0.3
	 modified by Lu at  April/26/2019 0:00
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

	
	if(*bFinished)//停车完成后，出车位跑
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


	}
	
	

	///=======================================printf functions============================================
	//printf("speed %.3f yaw %.2f distance %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	// printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
	//printf("Steer:%.2f\tflag:%d\tspeed:%.2f\tdist:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tflagt:%d\n",*cmdSteer,flag,_speed,dist,_lotAngle,_caryaw,flagt);
	//printf("Steer:%.2f\tflag:%d\tflagt:%d\n",*cmdSteer,flag,flagt);
	printf("bFinished:%d ", *bFinished);
	printf("flag:%d ", flag);
	printf("flagt:%d ", flagt);
	printf("speed:%.1f ", _speed);
	printf("_lotX:%.1f ", _lotX);
	printf("_lotY:%.1f ", _lotY);
	//printf("_lotAngle:%.1f ", _lotAngle);
	printf("carX:%.1f ", _carX);
	printf("carY:%.1f ", _carY);
	printf("caryaw:%.1f ", _caryaw);
	printf("dist:%.1f ", dist);
	printf("distance:%.1f ", distance);

	printf("*Acc:%.1f ", *cmdAcc);
	printf("Brake:%.1f ", *cmdBrake);
	//printf("*cmdGear:%d ", *cmdGear);
	printf("Steer:%.1f ", *cmdSteer);
	//if(*bFinished)printf("\n============bFinished============\n");

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
///=================helping functions from TA no need to modify============================