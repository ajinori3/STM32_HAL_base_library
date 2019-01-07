#include "MPU6050.h"

/*Prototype*/
static float rawToGyro(GyroRange_t gyroRange);
static float rawToAccel(AccelRange_t accelRange);

HAL_StatusTypeDef MPU6050_GetGyroAccelDMA(MPU6050_t* const mpu6050)
{
	return HAL_I2C_Mem_Read_DMA(mpu6050->hi2c1, mpu6050->dev_address, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, mpu6050->rawdata_buff, I2C1_RX_DMA_BUFF_SIZ);
}

HAL_StatusTypeDef MPU6050_GetGyroAccelBlocking(MPU6050_t* const mpu6050)
{
	return HAL_I2C_Mem_Read(mpu6050->hi2c1, mpu6050->dev_address, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, mpu6050->rawdata_buff, I2C1_RX_DMA_BUFF_SIZ, 1000);
}

void MPU6050_UpdateRawData(MPU6050_t* const mpu6050)
{
	mpu6050->accel_int16[ACCEL_X] = (mpu6050->rawdata_buff[0] << 8) + mpu6050->rawdata_buff[1];
	mpu6050->accel_int16[ACCEL_Y] = (mpu6050->rawdata_buff[2] << 8) + mpu6050->rawdata_buff[3];
	mpu6050->accel_int16[ACCEL_Z] = (mpu6050->rawdata_buff[4] << 8) + mpu6050->rawdata_buff[5];
	
	mpu6050->temperature_raw = (mpu6050->rawdata_buff[6] << 8) + mpu6050->rawdata_buff[7];
	
	mpu6050->gyro_int16[GYRO_X] = (mpu6050->rawdata_buff[8] << 8) + mpu6050->rawdata_buff[9];
	mpu6050->gyro_int16[GYRO_Y] = (mpu6050->rawdata_buff[10] << 8) + mpu6050->rawdata_buff[11];
	mpu6050->gyro_int16[GYRO_Z] = (mpu6050->rawdata_buff[12] << 8) + mpu6050->rawdata_buff[13];		
}

void MPU6050_CalcFloatData(MPU6050_t* const mpu6050)
{
	mpu6050->accel_float[ACCEL_X] = (mpu6050->accel_int16[ACCEL_X] - mpu6050->accel_raw_offset[ACCEL_X]) / rawToAccel(mpu6050->accel_scale);
	mpu6050->accel_float[ACCEL_Y] = (mpu6050->accel_int16[ACCEL_Y] - mpu6050->accel_raw_offset[ACCEL_Y]) / rawToAccel(mpu6050->accel_scale);
	mpu6050->accel_float[ACCEL_Z] = (mpu6050->accel_int16[ACCEL_Z] - mpu6050->accel_raw_offset[ACCEL_Z]) / rawToAccel(mpu6050->accel_scale);
	
	mpu6050->gyro_float[GYRO_X] = (mpu6050->gyro_int16[GYRO_X] - mpu6050->gyro_raw_offset[GYRO_X]) / rawToGyro(mpu6050->gyro_scale);
	mpu6050->gyro_float[GYRO_Y] = (mpu6050->gyro_int16[GYRO_Y] - mpu6050->gyro_raw_offset[GYRO_Y]) / rawToGyro(mpu6050->gyro_scale);
	mpu6050->gyro_float[GYRO_Z] = (mpu6050->gyro_int16[GYRO_Z] - mpu6050->gyro_raw_offset[GYRO_Z]) / rawToGyro(mpu6050->gyro_scale);
}

HAL_StatusTypeDef MPU6050_Write1byte(MPU6050_t* const mpu6050, uint16_t memAddress, uint8_t data)
{
	return HAL_I2C_Mem_Write(mpu6050->hi2c, mpu6050->dev_address, memAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);	
}

HAL_StatusTypeDef MPU6050_Read1byte(MPU6050_t* const mpu6050, uint16_t memAddress, uint8_t* rxBuff)
{
	return HAL_I2C_Mem_Read(mpu6050->hi2c1, mpu6050->dev_address, memAddress, I2C_MEMADD_SIZE_8BIT, rxBuff, 1, 1000);
}


void calcAngle_accel(MPU6050_t* const mpu6050)
{	
	mpu6050->roll = atan2f(-mpu6050->accel_int16[ACCEL_Y], mpu6050->accel_int16[ACCEL_Z]) * 180 *  M_1_PI;
	mpu6050->pitch = atan2f(-mpu6050->accel_int16[ACCEL_X], mpu6050->accel_int16[ACCEL_Z]) * 180 *  M_1_PI;
	mpu6050->yaw = atan2f(-mpu6050->accel_int16[ACCEL_X], mpu6050->accel_int16[ACCEL_Y]) * 180 *  M_1_PI;
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_t* mpu6050, I2C_HandleTypeDef *hi2c, uint8_t devAddress, GyroRange_t gyroScale, AccelRange_t accelScale)
{
	uint8_t rxBuff = 0;
	mpu6050 = (MPU6050_t*)memset(mpu6050, 0, sizeof(MPU6050_t));
	
	mpu6050->gyro_scale = gyroScale;
	mpu6050->accel_scale = accelScale;
	mpu6050->dev_address = (devAddress << 1);
	mpu6050->hi2c = hi2c;
	mpu6050->gyro_offset_calc_number = 10;
	
	HAL_StatusTypeDef debug;
		
	HAL_Delay(500);
	debug = MPU6050_Write1byte(mpu6050, MPU6050_PWR_MGMT_1, 0x1 << 7);			//device reset
	HAL_Delay(500);
	MPU6050_Write1byte(mpu6050, MPU6050_PWR_MGMT_1, 0x00);      // internal 8MHz, disabled SLEEP mode, disable CYCLE mode  
	MPU6050_Read1byte(mpu6050, MPU6050_WHO_AM_I, &rxBuff);
		
	if (rxBuff != 0x68) {		//This value is NOT affected by devce address.
		//fprintf(stderr, "[ERROR] MPU6050 Connection Error.\n");
		return HAL_ERROR;
	}
	
	MPU6050_Write1byte(mpu6050, MPU6050_SMPLRT_DIV, 0x03);   	 //sample rate: 8khz
	MPU6050_Write1byte(mpu6050, MPU6050_CONFIG, 0x06);   		 //DLPF disable
	MPU6050_Write1byte(mpu6050, MPU6050_GYRO_CONFIG, mpu6050->gyro_scale << 3);
	MPU6050_Write1byte(mpu6050, MPU6050_ACCEL_CONFIG, mpu6050->accel_scale << 3);
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalcGyroOffset(MPU6050_t* const mpu6050, uint16_t sample_number)
{
	mpu6050->gyro_raw_offset[GYRO_X] = 0.0;
	mpu6050->gyro_raw_offset[GYRO_Y] = 0.0;
	mpu6050->gyro_raw_offset[GYRO_Z] = 0.0;
		
	for (int i = 0; i < sample_number; i++)
	{
		HAL_StatusTypeDef status;
		
		status = MPU6050_GetGyroAccelBlocking(mpu6050);		
		if (status != HAL_OK)
		{
			return status;
		}
		
		MPU6050_UpdateRawData(mpu6050);
		
		mpu6050->gyro_raw_offset[GYRO_X] += mpu6050->gyro_int16[GYRO_X];
		mpu6050->gyro_raw_offset[GYRO_Y] += mpu6050->gyro_int16[GYRO_Y];
		mpu6050->gyro_raw_offset[GYRO_Z] += mpu6050->gyro_int16[GYRO_Z];
	}
	
	if (sample_number > 0)
	{
		mpu6050->gyro_raw_offset[GYRO_X] /= sample_number;
		mpu6050->gyro_raw_offset[GYRO_Y] /= sample_number;
		mpu6050->gyro_raw_offset[GYRO_Z] /= sample_number;
	}
	
	return HAL_OK;
}

static float rawToGyro(GyroRange_t gyroRange)
{
	const float RAW_TO_GYRO[4] = { 65536 / 500.0, 65536 / 1000.0, 65536 / 2000.0, 65536 / 4000.0 };    				//to deg/s	 
	return RAW_TO_GYRO[gyroRange];
}

static float rawToAccel(AccelRange_t accelRange)
{	
	const float rawToAccel[4] = { 16384, 8192, 4096, 2048 };     			//to g
	return rawToAccel[accelRange];
}


void MPU6050_CalcRollPitchYaw(MPU6050_t* const mpu6050)
{
	const float GYRO_RATE = 0.99;
	const float SAMPLE_GET_COUNT = 0.004;		//[Sec]
	
	float gyro_roll, gyro_pitch, gyro_yaw;
	
	gyro_roll = mpu6050->roll - mpu6050->gyro_float[GYRO_X] * SAMPLE_GET_COUNT;
	gyro_pitch = mpu6050->pitch + mpu6050->gyro_float[GYRO_Y] * SAMPLE_GET_COUNT;
	gyro_yaw = mpu6050->yaw - mpu6050->gyro_float[GYRO_Z] * SAMPLE_GET_COUNT;
	
	calcAngle_accel(mpu6050);
	
	mpu6050->roll = gyro_roll * GYRO_RATE + mpu6050->roll * (1 - GYRO_RATE);
	mpu6050->pitch = gyro_pitch * GYRO_RATE + mpu6050->pitch * (1 - GYRO_RATE);
	mpu6050->yaw = gyro_yaw * GYRO_RATE + mpu6050->yaw * (1 - GYRO_RATE);
}