//
// Created by michal on 11.06.19.
//

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include "BMI160_IMU.h"
#include "bmi160.h"
#include "bmi160_defs.h"
#include <android/log.h>
#include <cstdlib>
#include <stdlib.h>
#include <sys/ioctl.h>      /* ioctl */
#include <sys/time.h>

#define DEVICE_FILE_NAME "/dev/bmi160"

/* IOCTL commands */
#define IOC_MAGIC                       'k'

/* Read/Write Message */
#define BMI160_SENSOR_START             _IOW(IOC_MAGIC, 1, __u8)
#define BMI160_SENSOR_STOP              _IOW(IOC_MAGIC, 2, __u8)
#define BMI160_SENSOR_READ              _IOW(IOC_MAGIC, 3, __u8)
struct sensor_data {
    short acc_x;
    short acc_y;
    short acc_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    unsigned int sensor_time;
};
struct sensor_data sensor_output;
int file_desc;

//Dev file for i2c bus. /dev/i2c-1 for x1a, /dev/i2c-4 for x2xr1 prototype, /dev/i2c-3 for x2xr1 production
const char *BMI160_IMU::i2cFileName = "/dev/i2c-3";
int cnt = 1;
void BMI160_IMU::start() {
//    if(!run){
//        run = true;
//        t = std::thread(&BMI160_IMU::poll, this);
//    }


    int ret_val = 0;
    if((cnt % 2) == 1) {
        cnt++;
        if (!run) {
            run = true;
            t = std::thread(&BMI160_IMU::poll, this);
        }
    } else
    {

#if 1
        bmi160_dev sensor1{
                .id = BMI160_I2C_ADDR,
                .interface = BMI160_I2C_INTF,
                .read = i2c_read,
                .write = i2c_write,
                .delay_ms = i2c_delay_ms
        };
        bmi160_soft_reset(&sensor1);
#else
        ret_val = ioctl(file_desc, BMI160_SENSOR_STOP, &sensor_output);
        if (ret_val < 0) {
            exit(-1);
        }
        close(file_desc);
#endif
        cnt++;
        stop();
    }



}

void BMI160_IMU::stop() {
    if(run){
        run = false;
        t.join();
    }
}

bool BMI160_IMU::isRunning() {
    return run;
}

void BMI160_IMU::poll() {

    pthread_setname_np(pthread_self(), "IMU_BMI");
    long accHzCounterTimestamp = 0;
    int accHzCounter = 0;
    int accHz = 0;
    long tsOffset = 0;
    long lastTS = -1;

    setup();

#if 0
    if(!i2c_assert_permission()){
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "Insufficient permissions for i2c");
        return;
    }

    bmi160_dev sensor{
        .id = BMI160_I2C_ADDR,
        .interface = BMI160_I2C_INTF,
        .read = i2c_read,
        .write = i2c_write,
        .delay_ms = i2c_delay_ms
    };

    auto err = bmi160_init(&sensor);

    if(err){
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "Error during sensor initialization");
        return;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /*  Set the Power mode  */
    err = bmi160_set_power_mode(&sensor);

    if(err){
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "Error during setting sensor power mode");
        return;
    }

    /* Set the sensor configuration */
    err = bmi160_set_sens_conf(&sensor);

    if(err){
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "Error during setting sensor configuration");
        return;
    }

    bmi160_sensor_data acc{};
    bmi160_sensor_data gyr{};

    auto select = BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL;

    while (run) {

        sensor.delay_ms(0);
        bmi160_get_sensor_data((uint8_t)select, &acc, &gyr, &sensor);

        // Sensortime is incremented by 1 every 39 microseconds,
        // so (senortime * 39 * 1000) yields the time since measurements started in nanoseconds.
        long ts = (long)(acc.sensortime * 39000.0);

        auto acc_x = acc.x * ACC_STEP_2G;
        auto acc_y = acc.y * ACC_STEP_2G;
        auto acc_z = acc.z * ACC_STEP_2G;

        auto gyr_x = gyr.x * GYR_STEP_2000DPS;
        auto gyr_y = gyr.y * GYR_STEP_2000DPS;
        auto gyr_z = gyr.z * GYR_STEP_2000DPS;

        if (ts > lastTS) {
//            __android_log_print(ANDROID_LOG_DEBUG,"IMU", "Hello");
            lastTS = ts;
            callback(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, tsOffset+ts);
        } else if (ts < lastTS) {
            // next timestamp is smaller which means that the sensortime register has reset back to 0x000000 when it reached it's max value 0xFFFFFF.
            tsOffset += 654311385000; // == 0xFFFFFF * 39000
            lastTS = ts;
            callback(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, tsOffset+ts);
        }
    }
#else ///// Review above code and see why without the above code callbak function is not
//being called

    timespec tspec;
    int m = 0;
    int k = 0;

    int i;
    int ret_val;
    //long tsOffset = 0;
    //long lastTS = -1;
    long ts = 0;
    long prev_ts = 0;
    printf("Enter\n");

    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Peel: Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }

    ret_val = ioctl(file_desc, BMI160_SENSOR_START, &sensor_output);
    if (ret_val < 0) {
        printf("ioctl_en_pwm_msg failed:%d\n", ret_val);
        exit(-1);
    }
    sleep(1);
    while (run) {
        ret_val = ioctl(file_desc, BMI160_SENSOR_READ, &sensor_output);
        clock_gettime(CLOCK_MONOTONIC, &tspec);
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "tspec.tv_nsec: %3d: %10lu", m, tspec.tv_nsec + tspec.tv_sec*1000000000);
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "sensor_output.sensor_time: %3d: %10u", k, sensor_output.sensor_time);
        m++;
        k++;

        if(m == 100)
            exit(-1);

        if (ret_val < 0) {
            printf("ioctl_en_pwm_msg failed:%d\n", ret_val);
            exit(-1);
        }

//        ts = (long) (sensor_output.sensor_time * 39000.0);
#if 0
        // Enable this section to find out the data rate between HAL and driver
        // Rate is printed in logcat
        if(ts != prev_ts) {
            accHzCounter++;
            if ((ts - accHzCounterTimestamp) > 1000000000) {
                accHz = accHzCounter;
                __android_log_print(ANDROID_LOG_ERROR, "acc1.HZ = ", "%u", accHz);

                accHzCounterTimestamp = ts;
                accHzCounter = 1;
            }
        }
        prev_ts = ts;

#else
        float acc_x = (float) sensor_output.acc_x * ACC_STEP_2G;
        float acc_y = (float) sensor_output.acc_y * ACC_STEP_2G;
        float acc_z = (float) sensor_output.acc_z * ACC_STEP_2G;

        float gyr_x = (float) sensor_output.gyro_x * GYR_STEP_2000DPS;
        float gyr_y = (float) sensor_output.gyro_y * GYR_STEP_2000DPS;
        float gyr_z = (float) sensor_output.gyro_z * GYR_STEP_2000DPS;

        callback(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, tspec.tv_nsec);
//        if (ts > lastTS) {
//            lastTS = ts;
//            callback(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, tsOffset + ts);
//        } else if (ts < lastTS) {
//            // next timestamp is smaller which means that the sensortime register has reset back to 0x000000 when it reached it's max value 0xFFFFFF.
//            tsOffset += 654311385000; // == 0xFFFFFF * 39000
//            lastTS = ts;
//            callback(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, tsOffset + ts);
//        }
#endif

    }
#endif
}

bool BMI160_IMU::i2c_assert_permission() {
    int f;
    bool opn = (f=open(i2cFileName, O_RDWR)) > 0;
    if(!opn) {
        auto s = system("su");
        s = system("chmod 777 /dev");
        std::string cmd("chmod 777 ");
        cmd += i2cFileName;
        s = system(cmd.c_str());
        opn = (f=open(i2cFileName, O_RDWR)) > 0;
    }
    if(opn){
        close(f);
    }
    return opn;
}

int8_t BMI160_IMU::i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {

    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    char buf[10];
    buf[0] = reg_addr;

    int file;

    //open i2c file
    if((file = open(i2cFileName, O_RDWR)) < 0) {
        // ERROR Could not open file
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "*********could not open i2c file\n");
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    //inititate communication with the specific I2C device dev_addr
    if(ioctl(file, I2C_SLAVE_FORCE, dev_addr) < 0) {
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "***could not initiate communication with i2c\n");
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    //write the register address
    if(write(file, buf, 1) != 1) {
        // ERROR could not write register address to i2c
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "***could not write register addr %c to i2c\n", buf[0]);
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }


    if(read(file, data, len) != len) {
        // ERROR could not read from i2c
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "***could not read data from i2c\n");
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    close(file);
    return rslt;
}

int8_t BMI160_IMU::i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {

    int8_t rslt = 0; // Return 0 for Success, non-zero for failure

    int file;

    char buf[10];
    buf[0] = reg_addr;
    buf[1] = *data;

    //open i2c file
    if((file = open(i2cFileName, O_RDWR)) < 0) {
        // ERROR Could not open file
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    //inititate communication with the I2C device
    if(ioctl(file, I2C_SLAVE_FORCE, dev_addr) < 0) {
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    //write the register address followed by the data to be written
    if(write(file, buf, len + 1) != len + 1) {
        // ERROR could not write register address to i2c
        __android_log_print(ANDROID_LOG_DEBUG,"IMU", "%d\n", errno);
        __android_log_print(ANDROID_LOG_DEBUG, "IMU", "ERROR: %s\n", strerror(errno));
        rslt = 1;
        close(file);
        return rslt;
    }

    close(file);

    return rslt;
}

void BMI160_IMU::i2c_delay_ms(uint32_t period) {
    std::this_thread::sleep_for(std::chrono::milliseconds(period));
}