#ifndef BMA280_WRAPPER_H_
#define BMA280_WRAPPER_H_

#include "bma2x2.h"
#include <mbed.h>
 
// create BMA cleass

#define ACK 1
#define NACK 0
#define INT_TWO_DECIMALS 100
#define PI_DEGREE 180


typedef struct bma2x2_xyz_calibration_offset{

    int16_t x_offset_error = 0;
    int16_t y_offset_error = 0;
    int16_t z_offset_error = 0;

} bma2x2_xyz_calibration_offset;

typedef struct bma2x2_xyz_calibration_gain{

    int16_t x_gain = 0;
    int16_t y_gain = 0;
    int16_t z_gain = 0;

} bma2x2_xyz_calibration_gain;

typedef struct bma280_accelero_values{

    int16_t x;
    int16_t y;
    int16_t z;

}bma280_accelero_values;

class BMA280_Wrapper
{

    private:
    I2C &i2c;
    struct bma2x2_t *bma2x2;
    bma280_accelero_values accel_val; 
    bma2x2_xyz_calibration_offset offset_for_output;
    bma2x2_xyz_calibration_gain gain_for_output;
    bma2x2_accel_data accel_data_xyz;
    Timer timerNextReading;

    public: 
    // add getters and setters
    int16_t get_accelero_x();
    int16_t get_accelero_y();
    int16_t get_accelero_z();
    

    double compute_angle_three_axes_X();
    BMA280_Wrapper();
    BMA280_Wrapper(I2C &i2c, struct bma2x2_t *bma2x2);
    void print_calibrated_val();
    bool read_and_calibrate_data_bma280();
    s32 bma2x2_shock_setup();
    s8 I2C_routine();
    void calibrate_bma2x2_initial();
    bool BMA280_read_updated_data();

};

s8 BMA2x2_I2C_bus_write(u8 dev_id, u8 reg_addr, u8 *reg_data, u8 len);
s8 BMA2x2_I2C_bus_read(u8 dev_id, u8 reg_addr, u8 *reg_data, u8 len);
void BMA2x2_delay_msek(u32 waiting_ms);

#endif