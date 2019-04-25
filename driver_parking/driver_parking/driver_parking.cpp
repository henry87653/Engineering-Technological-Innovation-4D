 /***************************************************************************
	 Copyright (C) 2019
	 All rights reserved
	 file : driver_parking.cpp
	 description :ע��to TA's code
	 version: 1.0.1
	 modified by Lu at  April/25/2019 13:13
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

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;//��λ���ĵ��������
	_lotY = lotY;//��λ���ĵ��������
	_lotAngle = lotAngle;//��λ���Գ���
	_bFrontIn = bFrontIn;//�ƺ����ã�
	_carX = carX;//��ǰ������������
	_carY = carY;//��ǰ������������
	_caryaw = caryaw;//��ǰ�������Գ���
	//�ص�·���� k �״�������ڵ�ǰ������ XY ����ֵ
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;//ƫ���ǣ����ȣ�������ڵ�·���ߣ�
	_yawrate = yawrate;//���ٶȣ�����/�룩
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	printf("lotX %.6f  lotY %.6f",_lotX,_lotY);
}

static int flag = 0;//�����жϣ�1-5
static float k,b,dist;
static int flagt = 0;

static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){   
	/* write your own code here */
	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))                       //���㳵�������벴��λ����ֱ�ߵľ��룬�����ж��Ƿ�ʼ����
		dist = abs(_carX - _lotX);//��λ������Y����
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);//��λ�ӳ��߷���Y�ؾ�
		dist = abs(k*_carX - _carY +b)/sqrt(k*k + 1);// k*_carX - _carY�ǳ������ų�λ����Y�ؾص��෴����abs����/sqrt(k*k + 1) = * cos(_lotAngle)
	}

	
	if ( flagt == 1 ){//������ɣ��ͳ嵹��������λ
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = -1;
		*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;//ldx:we can modify?
	}else
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.5 ) {    //�ó����ж��Ƿ���ɲ���//0~0.5
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;//ldx:we can modify? eg 15 *
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed < 0.01){
			    *bFinished = true;
				flagt = 1;//������ɱ�־flagt = 1}
			else
			{*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			flag = 1; 
		}else if( ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10)&& ( flagt ==2 )) { 
			//0.5~10
			//�ӽ�ͣ��λʱ�����Ƴ��ĳ����복λһ�£��ٶȿ�����2
			//*cmdSteer = 0;
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;//ldx:we can modify? eg 15 *
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			else if(_speed >0.07){*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			else{*bFinished = true;flagt=1; }//ldx: *bFinished = true ���������⣬Ҫע�͵�
			flag = 2;
		} else if (((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2)&&(flagt!=2)){
			//
			//�Ͻӽ�ͣ��λʱ����һ�����ת���ٶȿ�����10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			if (fabs(fabs(_caryaw)-fabs(_lotAngle))<0.015){
				flagt = 2;
			}
			flag = 3;
		} else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 5000) { //��һ����Χʱ�������ӵ�������·�Ҳ࣬����ת��뾶���ٶȿ�����15
			*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
			if( _speed > 15 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
		}
		else {			                                                                         //����·�ΰ�Ѳ�߷�ʽ��ʻ
			*cmdAcc = 1;//���Ÿ�100%
		    *cmdBrake = 0;//��ɲ��
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;//�趨�������//ldx: we can modify? ex. midline[10]
		    *cmdGear = 1;//��λʼ�չ�1
			flag = 5;
	    }
	}

	
	if(*bFinished)
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10 ) { //�ӽ�ͣ��λʱ�����Ƴ��ĳ����복λһ�£��ٶȿ�����2
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2 ){//�Ͻӽ�ͣ��λʱ����һ�����ת���ٶȿ�����10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else {			                                                                         //����·�ΰ�Ѳ�߷�ʽ��ʻ
			*cmdAcc = 1;//���Ÿ�100%
		    *cmdBrake = 0;//��ɲ��
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;//�趨�������
		    *cmdGear = 1;//��λʼ�չ�1
	    }


	}
	
	*cmdAcc = constrain(0, 1, *cmdAcc);
	*cmdBrake = constrain(0, 1, *cmdBrake);
	*cmdAcc = constrain(-1, 1, *cmdAcc);

	printf("Steer:%.2f\tflag:%d\tspeed:%.2f\tdist:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tflagt:%d\n",*cmdSteer,flag,_speed,dist,_lotAngle,_caryaw,flagt);
	printf("Steer:%.2f\tflag:%d\tflagt:%d\n",*cmdSteer,flag,flagt);
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