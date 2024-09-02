//
// Created by Tom on 22.06.2024.
//

#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_I2C_ADDR 0x68
#define MPU6050_I2C_CLK  348

float read_gyro_x();
float read_gyro_y();
float read_gyro_z();
float read_accel_x();
float read_accel_y();
float read_accel_z();
bool mpu6050_init();

#endif //MPU6050_H
