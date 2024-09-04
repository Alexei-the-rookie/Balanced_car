#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define SMPLRT_DIV   0x19  // ?????,???:0x07(125Hz) */
#define CONFIG       0x1A  // ??????,???:0x06(5Hz) */
#define GYRO_CONFIG  0x1B  // ??????????,???:0x18(???,2000deg/s) */
#define ACCEL_CONFIG 0x1C  // ?????????????????,???:0x01(???,2G,5Hz) */

#define ACCEL_XOUT_H 0x3B  // ?????X??Y??Z??????????? */
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H   0x41  // ?????????????? */
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x43  // ?????X??Y??Z??????????? */
#define GYRO_XOUT_L  0x44 
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define PWR_MGMT_1   0x6B   // ????,???:0x00(????) */
#define WHO_AM_I     0x75 	// IIC?????(????0x68,??) */
#define MPU6050_ADDR 0xD0	// MPU6050??????,???????serch?????

typedef struct{
	// ???
	float Accel_X;
	float Accel_Y;
	float Accel_Z;
	// ??
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
	// ??
	float Temp;
}MPU6050DATATYPE;

extern MPU6050DATATYPE Mpu6050_Data;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

int16_t Sensor_I2C2_Serch(void);
int8_t MPU6050_Init(int16_t Addr);

int8_t Sensor_I2C2_ReadOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData);
int8_t Sensor_I2C2_WriteOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData);

void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);

#endif


