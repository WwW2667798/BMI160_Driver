#ifndef __BMI160_DRIVER_H
#define __BMI160_DRIVER_H

#include "stm32f10x.h"
#include "MyI2C.h"
#include "bmi160.h"
#include "bmi160_reg.h"
#include "complementary_filter.h"
#include "mahony_filter.h"

typedef struct
{
    struct bmi160_accel_t accel;
    struct bmi160_gyro_t gyro;
} bmi160_6axis_t;

typedef struct
{
	s8 (*init)(void);
	s8 (*read_accel)(struct bmi160_accel_t *accel);
	s8 (*read_gyro)(struct bmi160_gyro_t *gyro);
	s8 (*read_6axis)(bmi160_6axis_t *data);
	s8 (*Complementary_Update)(float *pitch, float *roll, float *yaw, float dt);
	s8 (*Mahony_Update)(float *pitch, float *roll, float *yaw, float dt);
} bmi160_drvp_t;

extern bmi160_drvp_t *bmi160_drvp;

//s8 bmi160_driver_init(void);
//s8 bmi160_read_accel(struct bmi160_accel_t *accel);
//s8 bmi160_read_gyro(struct bmi160_gyro_t *gyro);
//s8 bmi160_read_6axis(bmi160_6axis_t *data);
//s8 BMI160_Complementary_Update(float *pitch, float *roll, float *yaw, float dt);
//s8 BMI160_Mahony_Update(float *pitch, float *roll, float *yaw, float dt);

#endif
