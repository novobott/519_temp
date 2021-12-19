/*
 * MPU_6050.h
 *
 * Created: 12/19/2021 10:48:25 AM
 *  Author: arjun
 */ 


#ifndef MPU_6050_H_
#define MPU_6050_H_


uint8_t mpu6050_start(void);


//configure important settings in mpu6050
//subject to change app(ilcation) by app
void mpu6050_init(void);


// read gyro/accel X, Y, Z all at once, high- & low-8-bits combined
// return int16_t (signed) in buff
// buff must have at least 3 available places
// data sequence: (buff)-->X, (buff+1)-->Y, (buff+2)-->Z
// no error handling for too small buff
void mpu6050_read_gyro_ALL(int16_t * buff);
void mpu6050_read_accel_ALL(int16_t * buff);


//read gyro/accel X, Y, Z, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff
void mpu6050_read_gyro_X(uint8_t * buff);
void mpu6050_read_gyro_Y(uint8_t * buff);
void mpu6050_read_gyro_Z(uint8_t * buff);
void mpu6050_read_accel_X(uint8_t * buff);
void mpu6050_read_accel_Y(uint8_t * buff);
void mpu6050_read_accel_Z(uint8_t * buff);


#endif /* MPU_6050_H_ */