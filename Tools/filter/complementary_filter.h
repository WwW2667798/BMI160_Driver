#ifndef __COMPLEMENTARY_FILTER_H
#define __COMPLEMENTARY_FILTER_H

#include "stm32f10x.h"
#include <math.h>

#define PI         3.1415926535f
#define RtA        57.2957795f      // 弧度转角度
#define AtR        0.0174532925f    // 角度转弧度

void Complementary_Update(float ax, float ay, float az,
                          float gx_deg, float gy_deg, float gz_deg,
                          float dt, float *pitch, float *roll, float *yaw);

#endif
