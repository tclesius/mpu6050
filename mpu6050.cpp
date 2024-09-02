//
// Created by Tom Clesius on 22.06.2024.
//

#include <cstdio>
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "cmath"

// I2C Address default 0x68
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_I2C_CLK  348

#define WHO_AM_I        0x75

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define SELF_TEST_X     0x0D
#define SELF_TEST_Y     0x0E
#define SELF_TEST_Z     0x0F
#define SELF_TEST_A     0x10


#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B

bool mpu6050_init() {
    uint8_t buf[2] = { PWR_MGMT_1, 0};
    int result = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, buf, 2, false);
    return result >= 0;
}

float read_accel_x(){
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = ACCEL_XOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto accel_out = (int16_t)((dst_h << 8) | dst_l);

    return (float) accel_out / 16384;
}

unsigned int bit_slice(uint8_t x, uint8_t start, uint8_t end) {
    int numBits = end - start + 1;
    uint8_t mask = (1 << numBits) - 1;
    uint8_t slice = (x >> start) & mask;
    return slice;
}

bool read_self_test(uint8_t *xa_test, uint8_t *xg_test, uint8_t *ya_test, uint8_t *yg_test, uint8_t *za_test, uint8_t *zg_test){
    //TODO: add error handling
    uint8_t reg = 0;
    uint8_t buffer = 0;
    uint8_t buffer_a = 0;

    *xa_test = 0;
    *xg_test = 0;
    *ya_test = 0;
    *yg_test = 0;
    *za_test = 0;
    *zg_test = 0;

    reg = SELF_TEST_A;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &buffer_a, 1, false);

    reg = SELF_TEST_X;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &buffer, 1, false);

    *xa_test |= bit_slice(buffer, 5,7) << 2;
    *xa_test |= bit_slice(buffer_a, 4,5);

    *xg_test |= bit_slice(buffer, 0, 4);

    reg = SELF_TEST_Y;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &buffer, 1, false);

    *ya_test |= bit_slice(buffer, 5,7) << 2;
    *ya_test |= bit_slice(buffer_a, 2,3);

    *yg_test |=  bit_slice(buffer, 0, 4);

    reg = SELF_TEST_Z;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &buffer, 1, false);

    *za_test |= bit_slice(buffer, 5,7) << 2;
    *za_test |= bit_slice(buffer_a, 0,1);

    *zg_test |=  bit_slice(buffer, 0, 4);

    return true;
}

bool read_accel_config(uint8_t* dst){
    uint8_t reg = ACCEL_CONFIG;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, dst, 1, false);
    return true;
}

bool write_accel_config(const bool xa_st, const bool ya_st, const bool za_st, const uint8_t afs_sel){
    // afs_sel modes:
    // 0 : +- 2g
    // 1 : +- 4g
    // 2 : +- 8g
    // 3 : +- 16g
    // TODO: maybe use enum
    uint8_t buffer[2] = {ACCEL_CONFIG, 0};

    if ( xa_st ){
        buffer[1] |= 0x80;
    }
    if ( ya_st ){
        buffer[1] |= 0x40;
    }
    if ( za_st ){
        buffer[1] |= 0x20;
    }
    if (0 > afs_sel || afs_sel > 3){
        return PICO_ERROR_INVALID_ARG;
    } else {
        buffer[1] |= afs_sel << 3;
    }
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, buffer, 2, false);
    return true;
}

bool read_gyro_config(uint8_t* dst){
    uint8_t reg = GYRO_CONFIG;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, dst, 1, false);
    return true;
}

bool write_gyro_config(const bool xg_st, const bool yg_st, const bool zg_st, const uint8_t fs_sel){
    // fs_sel modes:
    // 0 : +- 250 °/s
    // 1 : +- 500 °/s
    // 2 : +- 1000 °/s
    // 3 : +- 2000 °/s
    // TODO: maybe use enum
    uint8_t buffer[2] = {GYRO_CONFIG, 0};

    if ( xg_st ){
        buffer[1] |= 0x80;
    }
    if ( yg_st ){
        buffer[1] |= 0x40;
    }
    if ( zg_st ){
        buffer[1] |= 0x20;
    }
    if (0 > fs_sel || fs_sel > 3){
        return PICO_ERROR_INVALID_ARG;
    } else {
        buffer[1] |= fs_sel << 3;
    }
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, buffer, 2, false);
    return true;
}

double calc_ft_a(uint8_t x){
    if (x == 0) {
        return 0;
    }
    return 4096 * 0.34 * std::pow((0.92 / 0.34), (x - 1) / (std::pow(2, 5) - 2));
}
double calc_ft_g(uint8_t x){
    if (x == 0) {
        return 0;
    }
    return 25 * 131 * std::pow(1.046,(x - 1));
}

bool self_test(){
    // self test activated response
    uint8_t xa_test = 0;
    uint8_t ya_test = 0;
    uint8_t za_test = 0;

    uint8_t xg_test = 0;
    uint8_t yg_test = 0;
    uint8_t zg_test = 0;


    // setup self test mode
    write_accel_config(true, true, true, 2);
    write_gyro_config(true, true, true, 0);

    sleep_ms(250);

    read_self_test(&xa_test, &xg_test, &ya_test, &yg_test, &za_test, &zg_test);

    // disable self test mode
    write_accel_config(false, false, false, 0);
    write_gyro_config(false, false, false, 0);

    // accelerometer factory trims
    double ft_xa = calc_ft_a(xa_test);
    double ft_ya = calc_ft_a(ya_test);
    double ft_za = calc_ft_a(za_test);


    // accelerometer change from factory trim of the self test response
    double ft_diff_xa =  (xa_test - ft_xa) / ft_xa;
    double ft_diff_ya = (ya_test - ft_ya) / ft_ya;
    double ft_diff_za = (za_test - ft_za) / ft_za;

    // gyroscope factory trims
    double ft_xg = calc_ft_g(xg_test);
    double ft_yg = calc_ft_g(yg_test);
    double ft_zg = calc_ft_g(zg_test);


    // gyroscope change from factory trim of the self test response
    double ft_diff_xg = (xg_test - ft_xg) / ft_xg;
    double ft_diff_yg = (yg_test - ft_yg) / ft_yg;
    double ft_diff_zg = (zg_test - ft_zg) / ft_zg;

    ft_diff_xg = 100.0 + 100.0 * ft_diff_xg;
    ft_diff_yg = 100.0 + 100.0 * ft_diff_yg;
    ft_diff_zg = 100.0 + 100.0 * ft_diff_zg;
    ft_diff_xa = 100.0 + 100.0 * ft_diff_xa;
    ft_diff_ya = 100.0 + 100.0 * ft_diff_ya;
    ft_diff_za = 100.0 + 100.0 * ft_diff_za;


    printf("%f%f%f%f%f%f",ft_diff_xa,ft_diff_ya,ft_diff_za,ft_diff_xg,ft_diff_yg,ft_diff_zg);

    return true;
}

float read_accel_y(){
    uint8_t reg =  0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = ACCEL_YOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = ACCEL_YOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto accel_out = (int16_t)((dst_h << 8) | dst_l);

    return (float) accel_out / 16384;
}

float read_accel_z(){
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = ACCEL_ZOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = ACCEL_ZOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto accel_out = (int16_t)((dst_h << 8) | dst_l);

    return (float) accel_out / 16384;
}

float read_temp(){
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = TEMP_OUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = TEMP_OUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto temp_out = (int16_t)((dst_h << 8) | dst_l);

    return ((float)temp_out / 340.0f) + 36.53f;
}

float read_gyro_x(){
    // outputs the rate of change in degree/s
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = GYRO_XOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = GYRO_XOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto gyro_out = (int16_t)((dst_h << 8) | dst_l);
    return (float) gyro_out / 131.0f; // TODO change based on FS_SEL
}

float read_gyro_y(){
    // outputs the rate of change in degree/s
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = GYRO_YOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = GYRO_YOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto gyro_out = (int16_t)((dst_h << 8) | dst_l);
    return (float) gyro_out / 131.0f; // TODO change based on FS_SEL
}

float read_gyro_z(){
    // outputs the rate of change in degree/s
    uint8_t reg = 0;
    uint8_t dst_h = 0;
    uint8_t dst_l = 0;

    reg = GYRO_ZOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_h, 1, false);

    reg = GYRO_ZOUT_L;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst_l, 1, false);

    auto gyro_out = (int16_t)((dst_h << 8) | dst_l);
    return (float) gyro_out / 131.0f; // TODO change based on FS_SEL
}


uint8_t read_whoami() {
    uint8_t dst = 0;
    uint8_t reg = WHO_AM_I;

    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDR, &reg, 1,true); // Note: `true` for nostop to keep the connection alive for reading
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDR, &dst, 1, false);

    return dst;
}