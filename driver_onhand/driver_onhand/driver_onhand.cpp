/***************************************************************************

                 file : driver_onhand.cpp
           author : T
             email : donquixote@sjtu.edu.cn
    description : user module for CyberOnHand

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_onhand.h"

static void userDriverShowParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_onhand(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_onhand";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberOnHand" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverShowParam = userDriverShowParam;
	return 0;
}

/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/


static void userDriverShowParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	printf("yaw %f yawrate %f direction %f\n", yaw, yawrate,yaw - atan2(midline[45][0],midline[45][1]));
	printf("45m direction %f\n", atan2(midline[45][0],midline[45][1]));
	printf("20m direction %f\n", atan2(midline[20][0],midline[20][1]));
	printf("speed %f gearbox %d rpm %f 0m(%f, %f) 10m(%f, %f)\n", speed, gearbox, rpm, midline[0][0], midline[0][1], midline[10][0], midline[10][1]);

}
