#ifndef __ADRC_H
#define __ADRC_H

#include "mymath.h"

float ADRC_fhan(float x1,float x2);
void ADRC_TD(float x,float *track,float *derivative);
float ADRC_fal(float x);
float ADRC_ESO(float u,float y,float b);
Quaternion Quaternion_Error(Quaternion E,Quaternion P);

#endif
