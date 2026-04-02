#ifndef __MAHONY_FILTER_H
#define __MAHONY_FILTER_H

void Mahony_Init(void);
void Mahony_UpdateIMU(float ax, float ay, float az,
                      float gx_deg, float gy_deg, float gz_deg, float dt,
					  float *pitch_out, float *roll_out, float *yaw_out);

#endif
