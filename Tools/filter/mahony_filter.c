#include "mahony_filter.h"
#include <math.h>

// MahonyAHRS算法实现，适用于IMU数据融合，输出欧拉角（俯仰角、横滚角、偏航角）

#define twoKpDef	(2.0f * 0.5f)	// 默认的比例增益（Kp）
#define twoKiDef	(2.0f * 0.05f)	// 默认的积分增益（Ki）

static float q0 = 1.0f, q1, q2, q3; // 四元数表示的姿态
static float integralFBx, integralFBy, integralFBz; // 积分误差项
static float roll, pitch, yaw;		// 欧拉角（度）
static char anglesComputed;         // 标志位，指示是否已计算欧拉角

//-------------------------------------------------------------------------------------------
// 快速逆平方根函数
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// -------------------------------------------------------------------------------------------
// MahonyAHRS算法初始化，重置四元数和积分误差
void Mahony_Init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// MahonyAHRS算法初始化，使用加速度计和磁力计数据计算初始姿态
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float init_yaw, init_pitch, init_roll;
	float cr2, cp2, cy2, sr2, sp2, sy2;
	float sin_roll, cos_roll, sin_pitch, cos_pitch;
	float magX, magY;

    // 归一化加速度计数据
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// 磁力计数据有效时才进行归一化和初始偏航角计算
	if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
	{
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
	}

	init_pitch = atan2f(-ax, az);
	init_roll  = atan2f(ay, az);

	sin_roll  = sinf(init_roll);
	cos_roll  = cosf(init_roll);
	cos_pitch = cosf(init_pitch);
	sin_pitch = sinf(init_pitch);

	if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
	{
		magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
		magY = my * cos_roll - mz * sin_roll;
		init_yaw = atan2f(-magY, magX);
	}
	else
	{
		init_yaw = 0.0f;
	}

	cr2 = cosf(init_roll * 0.5f);
	cp2 = cosf(init_pitch * 0.5f);
	cy2 = cosf(init_yaw * 0.5f);
	sr2 = sinf(init_roll * 0.5f);
	sp2 = sinf(init_pitch * 0.5f);
	sy2 = sinf(init_yaw * 0.5f);

	q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
	q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
	q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
	q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

	// 归一化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//-------------------------------------------------------------------------------------------
// MahonyAHRS算法更新，融合加速度计和陀螺仪数据，输出欧拉角（度）
static void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// 计算反馈仅当加速度计测量有效时进行
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		// 归一化加速度计测量
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 估计的重力方向
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// 误差是估计方向和测量方向之间的叉积
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

        // 计算并应用积分反馈（如果启用）
		if(twoKiDef > 0.0f)
		{
			integralFBx += twoKiDef * halfex * dt;	// 积分误差累积
			integralFBy += twoKiDef * halfey * dt;
			integralFBz += twoKiDef * halfez * dt;
			gx += integralFBx;	// 应用积分反馈
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;	// 重置积分误差
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 应用比例反馈
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// 积分四元数的变化率
	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 归一化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// 计算欧拉角（度），如果尚未计算过则进行计算
static void computeAngles(void)
{
	roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    // 限制sinp的范围以避免数值错误
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp) * 57.29578f;
    yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
    anglesComputed = 1;
}

//-------------------------------------------------------------------------------------------
// MahonyAHRS算法更新接口，接受加速度计和陀螺仪数据（陀螺仪以度/秒为单位），输出欧拉角（度）
void Mahony_UpdateIMU(float ax, float ay, float az, float gx_deg, float gy_deg, float gz_deg, float dt,
					  float *pitch_out, float *roll_out, float *yaw_out)
{
	// 将陀螺仪数据从度/秒转换为弧度/秒
	float gx_rad = gx_deg * 0.01745329252f;
	float gy_rad = gy_deg * 0.01745329252f;
	float gz_rad = gz_deg * 0.01745329252f;

	MahonyAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, dt);

	if (!anglesComputed)
		computeAngles();

	if (pitch_out) *pitch_out = pitch;
	if (roll_out)  *roll_out  = roll;
	if (yaw_out)   *yaw_out   = yaw;
}

//-------------------------------------------------------------------------------------------
// MahonyAHRS算法更新，融合加速度计、陀螺仪和磁力计数据，输出欧拉角（度）
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// 如果磁力计测量无效，则仅使用IMU更新
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// 计算反馈仅当加速度计测量有效时进行
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
        // 归一化加速度计测量
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 归一化磁力计测量
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// 预计算四元数乘积
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// 参考方向的磁场
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// 估计的重力和磁场方向
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// 误差是估计方向和测量方向之间的叉积
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// 计算并应用积分反馈（如果启用）
		if(twoKiDef > 0.0f)
		{
			integralFBx += twoKiDef * halfex * dt;
			integralFBy += twoKiDef * halfey * dt;
			integralFBz += twoKiDef * halfez * dt;
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 应用比例反馈
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// 积分四元数的变化率
	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 归一化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// 获取当前的欧拉角（度），如果尚未计算过则进行计算
float getRoll(void)
{
	if (!anglesComputed) computeAngles();
	return roll;
}

float getPitch(void)
{
	if (!anglesComputed) computeAngles();
	return pitch;
}

float getYaw(void)
{
	if (!anglesComputed) computeAngles();
	return yaw;
}

//=============================================================================================
// END OF CODE
//=============================================================================================
