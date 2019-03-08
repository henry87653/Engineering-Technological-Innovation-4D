/***************************************************************************

    file                 : user0.h
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberOnHand

 ***************************************************************************/

#include "tgf.h"
//#include <cmath>

#ifndef __USER_ITF
#define __USER_ITF

/* CyberOnHand User Interface */
typedef void (*tfudShowParam) (float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);

typedef  struct {
	tfudShowParam userDriverShowParam;
} tUserItf;

#endif