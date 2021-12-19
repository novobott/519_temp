/*
 * MPU_6050_reg.h
 *
 * Created: 12/19/2021 10:50:16 AM
 *  Author: arjun
 */ 


#ifndef MPU_6050_REG_H_
#define MPU_6050_REG_H_

#define MPU6050_ADDRESS 0b11010000 // Address with end write bit
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68

#define MPU6050_RA_ACCEL_XOUT_H 0x3B//
#define MPU6050_RA_ACCEL_XOUT_L 0x3C//

#define MPU6050_RA_ACCEL_YOUT_H 0x3D//
#define MPU6050_RA_ACCEL_YOUT_L 0x3E//

#define MPU6050_RA_ACCEL_ZOUT_H 0x3F//
#define MPU6050_RA_ACCEL_ZOUT_L 0x40//

#define MPU6050_RA_GYRO_XOUT_H 0x43//
#define MPU6050_RA_GYRO_XOUT_L 0x44//

#define MPU6050_RA_GYRO_YOUT_H 0x45//
#define MPU6050_RA_GYRO_YOUT_L 0x46//

#define MPU6050_RA_GYRO_ZOUT_H 0x47//
#define MPU6050_RA_GYRO_ZOUT_L 0x48//


#endif /* MPU_6050_REG_H_ */
