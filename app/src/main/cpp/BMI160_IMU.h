//
// Created by michal on 11.06.19.
//

#ifndef VISIONEYE_BMI160_IMU_H
#define VISIONEYE_BMI160_IMU_H

#include <thread>

class BMI160_IMU {

public:

    BMI160_IMU(const std::function<void()> &setup,
               const std::function<void()> &teardown,
            const std::function<void(float, float, float, float, float, float, long)> &callback)
        : callback(callback), setup(setup), teardown(teardown) {
    }

    virtual void start();
    virtual void stop();
    virtual bool isRunning();

    // Conversion from -/+2g range to m/s^2
    const double ACC_STEP_2G = 0.000598755;

    // Conversion from -/+2000 deg/s range to rad/s
    const double GYR_STEP_2000DPS = 0.001065264;

protected:

    void poll();

    static int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

    static int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

    static void i2c_delay_ms(uint32_t period);

    bool i2c_assert_permission();

    std::thread t;
    std::atomic<bool> run;

    const std::function<void(float, float, float, float, float, float, long)> callback;
    const std::function<void()> setup;
    const std::function<void()> teardown;

private:

    static const char *i2cFileName;

};


#endif //VISIONEYE_BMI160_IMU_H
