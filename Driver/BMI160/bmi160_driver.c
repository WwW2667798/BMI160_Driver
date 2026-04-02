#include "bmi160_driver.h"

#define BMI160_ACCEL_SCALE_2G   (2.0f / 32768.0f)
#define BMI160_ACCEL_SCALE_4G   (4.0f / 32768.0f)
#define BMI160_ACCEL_SCALE_8G   (8.0f / 32768.0f)
#define BMI160_ACCEL_SCALE_16G  (16.0f / 32768.0f)

#define BMI160_GYRO_SCALE_2000DPS  (2000.0f / 32768.0f)
#define BMI160_GYRO_SCALE_1000DPS  (1000.0f / 32768.0f)
#define BMI160_GYRO_SCALE_500DPS   (500.0f / 32768.0f)
#define BMI160_GYRO_SCALE_250DPS   (250.0f / 32768.0f)
#define BMI160_GYRO_SCALE_125DPS   (125.0f / 32768.0f)

struct bmi160_t bmi160_dev;

//-------------------------------------------------------------------------------------------
/*
 *	@brief 	BMI160 延迟函数
 *	@param 	msec：毫秒为单位
 *	@return None
 */
static void bmi160_delay_ms(u32 msec)
{
    Delay_ms(msec);
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  BMI160 读取函数
 *	@param  dev_addr：设备地址
 *	@param  reg_addr：寄存器地址
 *	@param  reg_data：存储读取的数据
 *	@param  cnt：读取的字节数
 *	@return SUCCESS or ERROR
 */
static s8 bmi160_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s8 result = SUCCESS;

    MyI2C_Start();

    MyI2C_SendByte(dev_addr << 1);
    if (MyI2C_ReceiveAck() != 0)
	{
        MyI2C_Stop();
        return E_BMI160_COMM_RES;
    }

    MyI2C_SendByte(reg_addr);
    if (MyI2C_ReceiveAck() != 0)
	{
        MyI2C_Stop();
        return E_BMI160_COMM_RES;
    }

    MyI2C_Start();
    MyI2C_SendByte((dev_addr << 1) | 0x01);
    if (MyI2C_ReceiveAck() != 0)
	{
        MyI2C_Stop();
        return E_BMI160_COMM_RES;
    }

    for (u8 i = 0; i < cnt; i++)
	{
        if (i == cnt - 1)
		{
            reg_data[i] = MyI2C_ReceiveByte();
            MyI2C_SendAck(1);
        } else
		{
            reg_data[i] = MyI2C_ReceiveByte();
            MyI2C_SendAck(0);
        }
    }

    MyI2C_Stop();
    return result;
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  BMI160 写入函数
 *	@param  dev_addr：设备地址
 *	@param  reg_addr：寄存器地址
 *	@param  reg_data：写入的数据
 *	@param  cnt：写入的字节数
 *	@return SUCCESS or ERROR
 */
static s8 bmi160_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s8 result = SUCCESS;

    MyI2C_Start();

    MyI2C_SendByte(dev_addr << 1);
    if (MyI2C_ReceiveAck() != 0)
	{
        MyI2C_Stop();
        return E_BMI160_COMM_RES;
    }

    MyI2C_SendByte(reg_addr);
    if (MyI2C_ReceiveAck() != 0)
	{
        MyI2C_Stop();
        return E_BMI160_COMM_RES;
    }

    for (u8 i = 0; i < cnt; i++)
	{
        MyI2C_SendByte(reg_data[i]);
        if (MyI2C_ReceiveAck() != 0)
		{
            MyI2C_Stop();
            return E_BMI160_COMM_RES;
        }
    }

    MyI2C_Stop();
    return result;
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  BMI160 初始化函数
 *	@param  None
 *	@return SUCCESS or ERROR
 */
s8 bmi160_driver_init(void)
{
    s8 com_rslt = E_BMI160_COMM_RES;

    MyI2C_Init();

    bmi160_dev.dev_addr = BMI160_I2C_ADDR2;
    bmi160_dev.chip_id = 0;
    bmi160_dev.delay_msec = bmi160_delay_ms;
    bmi160_dev.bus_write = bmi160_i2c_write;
    bmi160_dev.bus_read = bmi160_i2c_read;

    com_rslt = bmi160_init(&bmi160_dev);

    if (com_rslt != SUCCESS)
        return com_rslt;

    // 初始化过程已读取验证芯片ID
    // com_rslt = bmi160_i2c_read(bmi160_dev.dev_addr, BMI160_USER_CHIP_ID_ADDR, &bmi160_dev.chip_id, 1);

    // if (com_rslt != SUCCESS)
    //     return com_rslt;

    if (bmi160_dev.chip_id != 0xD1)    // BMI160标准芯片ID为0xD1
        return E_BMI160_COMM_RES;      // ID不匹配，通信错误或设备不存在

    bmi160_set_command_register(0xB6); // 软件复位
    bmi160_dev.delay_msec(100);

    u8 reg_data = 0x09;
    bmi160_write_reg(0x40, &reg_data, 1); // 加速度输出数据率：200Hz
    reg_data = 0x0C;
    bmi160_write_reg(0x41, &reg_data, 1); // 加速度量程：±16g
    bmi160_set_command_register(ACCEL_MODE_NORMAL); // 加速度工作模式：正常模式
    bmi160_dev.delay_msec(5);

    reg_data = 0x09;
    bmi160_write_reg(0x42, &reg_data, 1); // 陀螺仪输出数据率：200Hz
    reg_data = 0x00;
    bmi160_write_reg(0x43, &reg_data, 1); // 陀螺仪量程：±2000°/s
    bmi160_set_command_register(GYRO_MODE_NORMAL); // 陀螺仪工作模式：正常模式
    bmi160_dev.delay_msec(5);

    return SUCCESS;
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  读取BMI160加速度xyz
 *	@param  bmi160_accel_t
 *	@return bmi160_accel_t
 */
s8 bmi160_read_accel(struct bmi160_accel_t *accel)
{
    return bmi160_read_accel_xyz(accel);
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  读取BMI160陀螺仪xyz
 *	@param  bmi160_gyro_t
 *	@return bmi160_gyro_t
 */
s8 bmi160_read_gyro(struct bmi160_gyro_t *gyro)
{
    return bmi160_read_gyro_xyz(gyro);
}

//-------------------------------------------------------------------------------------------
/*
 *	@brief  读取BMI160六轴数据
 *	@param  bmi160_6axis_t
 *	@return SUCCESS or ERROR
 */
s8 bmi160_read_6axis(bmi160_6axis_t *data)
{
    s8 result = SUCCESS;

    result = bmi160_read_accel_xyz(&data->accel);
    if (result != SUCCESS)
        return result;

    result = bmi160_read_gyro_xyz(&data->gyro);
    if (result != SUCCESS)
        return result;

    return SUCCESS;
}

//-------------------------------------------------------------------------------------------
/**
 *	@brief  将BMI160加速度计原始数据转换为物理量（单位：g）
 *	@param  raw_accel：原始加速度计数据（16位有符号整数）
 *	@param  accel_range：加速度计量程设置（例如：±2g、±4g、±8g、±16g）
 *	@return 转换后的加速度值，单位为g
 */
static float bmi160_convert_accel(s16 raw_accel, u8 accel_range)
{
    float accel_g = 0.0f;
    switch (accel_range)
    {
        case BMI160_ACCEL_RANGE_2G:
            accel_g = raw_accel * BMI160_ACCEL_SCALE_2G;
            break;
        case BMI160_ACCEL_RANGE_4G:
            accel_g = raw_accel * BMI160_ACCEL_SCALE_4G;
            break;
        case BMI160_ACCEL_RANGE_8G:
            accel_g = raw_accel * BMI160_ACCEL_SCALE_8G;
            break;
        case BMI160_ACCEL_RANGE_16G:
            accel_g = raw_accel * BMI160_ACCEL_SCALE_16G;
            break;
        default:
            accel_g = raw_accel * BMI160_ACCEL_SCALE_2G;
            break;
    }
    return accel_g;
}

//-------------------------------------------------------------------------------------------
/**
 *	@brief  将BMI160陀螺仪原始数据转换为物理量（单位：°/s）
 *	@param  raw_gyro：原始陀螺仪数据（16位有符号整数）
 *	@param  gyro_range：陀螺仪量程设置（例如：±2000°/s、±1000°/s、±500°/s、±250°/s、±125°/s）
 *	@return 转换后的陀螺仪值，单位为°/s
 */
static float bmi160_convert_gyro(s16 raw_gyro, u8 gyro_range)
{
    float gyro_dps = 0.0f;
    switch (gyro_range)
    {
        case BMI160_GYRO_RANGE_2000_DEG_SEC:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_2000DPS;
            break;
        case BMI160_GYRO_RANGE_1000_DEG_SEC:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_1000DPS;
            break;
        case BMI160_GYRO_RANGE_500_DEG_SEC:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_500DPS;
            break;
        case BMI160_GYRO_RANGE_250_DEG_SEC:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_250DPS;
            break;
        case BMI160_GYRO_RANGE_125_DEG_SEC:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_125DPS;
            break;
        default:
            gyro_dps = raw_gyro * BMI160_GYRO_SCALE_250DPS;
            break;
    }
    return gyro_dps;
}

//-------------------------------------- 欧拉角解算函数 ----------------------------------------

//-------------------------------------------------------------------------------------------
/**
 * @brief  互补滤波更新函数，融合BMI160加速度计和陀螺仪数据计算姿态角
 * @param  pitch  输出俯仰角（度）
 * @param  roll   输出横滚角（度）
 * @param  yaw    输出偏航角（度）
 * @param  dt     采样时间间隔（秒），通常与调用周期一致
 * @return 状态码，SUCCESS (0) 表示成功，其他表示错误
 */
s8 BMI160_Complementary_Update(float *pitch, float *roll, float *yaw, float dt)
{
    s8 result;
    struct bmi160_accel_t accel;
    struct bmi160_gyro_t gyro;
    float ax, ay, az, gx, gy, gz;

    // 读取原始数据
    result = bmi160_read_accel(&accel);
    if (result != SUCCESS)
        return result;
    result = bmi160_read_gyro(&gyro);
    if (result != SUCCESS)
        return result;

    // 转换为物理量
    // 加速度：±16g
    ax = bmi160_convert_accel(accel.x, BMI160_ACCEL_RANGE_16G);
    ay = bmi160_convert_accel(accel.y, BMI160_ACCEL_RANGE_16G);
    az = bmi160_convert_accel(accel.z, BMI160_ACCEL_RANGE_16G);
    // 陀螺仪：±2000°/s
    gx = bmi160_convert_gyro(gyro.x, BMI160_GYRO_RANGE_2000_DEG_SEC);
    gy = bmi160_convert_gyro(gyro.y, BMI160_GYRO_RANGE_2000_DEG_SEC);
    gz = bmi160_convert_gyro(gyro.z, BMI160_GYRO_RANGE_2000_DEG_SEC);

    // 调用互补滤波更新
    Complementary_Update(ax, ay, az, gx, gy, gz, dt, pitch, roll, yaw);

    return SUCCESS;
}

//-------------------------------------------------------------------------------------------
/**
 * @brief  Mahony滤波更新函数，融合BMI160加速度计和陀螺仪数据计算姿态角
 * @param  pitch  俯仰角（度）
 * @param  roll   横滚角（度）
 * @param  yaw    航向角（度）
 * @param  dt     采样时间间隔（秒），通常为两次调用之间的时间差
 * @return 状态码，SUCCESS (0) 表示成功，其他表示错误
 * @note   使用前需要先调用 bmi160_driver_init() 进行初始化，该函数中已包含 Mahony_Init()
 */
s8 BMI160_Mahony_Update(float *pitch, float *roll, float *yaw, float dt)
{
    s8 result;
    struct bmi160_accel_t accel;
    struct bmi160_gyro_t gyro;
    float ax, ay, az, gx, gy, gz;

    // 读取原始数据
    result = bmi160_read_accel(&accel);
    if (result != SUCCESS)
        return result;
    result = bmi160_read_gyro(&gyro);
    if (result != SUCCESS)
        return result;

    // 转换为物理单位
    // 加速度：±16g
    ax = bmi160_convert_accel(accel.x, BMI160_ACCEL_RANGE_16G);
    ay = bmi160_convert_accel(accel.y, BMI160_ACCEL_RANGE_16G);
    az = bmi160_convert_accel(accel.z, BMI160_ACCEL_RANGE_16G);
    // 陀螺仪：±2000°/s
    gx = bmi160_convert_gyro(gyro.x, BMI160_GYRO_RANGE_2000_DEG_SEC);
    gy = bmi160_convert_gyro(gyro.y, BMI160_GYRO_RANGE_2000_DEG_SEC);
    gz = bmi160_convert_gyro(gyro.z, BMI160_GYRO_RANGE_2000_DEG_SEC);

    // 调用Mahony更新函数（注意：Mahony_UpdateIMU 内部已包含四元数更新和欧拉角计算）
    Mahony_UpdateIMU(ax, ay, az, gx, gy, gz, dt, pitch, roll, yaw);

    return SUCCESS;
}
