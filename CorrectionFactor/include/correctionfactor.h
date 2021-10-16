/**
  ******************************************************************************
  * @file    CorrectionFactor.hpp
  * @author  BFMC
  * @version V1.2.0
  * @date    17-04-20
  * @brief   This file contains the class declaration for computing the correction factor 
  * and instant speed based on the encoder values(counts per resolution).
  ******************************************************************************
 */


#ifndef CORRECTION_FACTOR_HPP
#define CORRECTION_FACTOR_HPP

#include <include/Encoders/quadratureencodertask.hpp>
#include <TaskManager/taskmanager.hpp>
#include <mbed.h>


class CorrectionFactor : public task::CTask {

    private: 
        const encoders::CQuadratureEncoderWithFilterTask &local_encoder;                                    // encoder ojbject from main()                                

        /*
         - array that contains the computed correction factor in mm/ms for rps values.
         - values from the encoder for a reference of 0.1 m/s are between 10->20 rps and the correction factor
         - can be found in position 10->20 from the CF_table, same for higher velocities
         - the computed CF go up to 1.2 m/s  
        */
        float CF_table[200] = {0.00, 0.015, 0.0225, 0.03, 0.0375, 0.045, 0.0525, 0.06, 0.0675, 0.075, 0.0825, 0.09, 0.0975, 0.105, 0.1125,
                             0.12, 0.1275, 0.135, 0.1425, 0.15, 0.1575, 0.165, 0.1725, 0.18, 0.1875, 0.195, 0.2025, 0.21, 0.2175, 0.225, 0.2325,
                             0.24, 0.2475, 0.255, 0.2625, 0.27, 0.2775, 0.285, 0.2925, 0.3, 0.3075, 0.315, 0.3225, 0.33, 0.3375, 0.345, 0.3525, 
                             0.36, 0.3675, 0.375, 0.3825, 0.39, 0.3975, 0.405, 0.4125, 0.42, 0.4275, 0.435, 0.4425, 0.45, 0.4575, 0.465, 0.4725, 
                             0.48, 0.4875, 0.495, 0.5025, 0.51, 0.5175, 0.525, 0.5325, 0.54, 0.5475, 0.555, 0.5625, 0.57, 0.5775, 0.585, 0.5925, 
                             0.6, 0.6075, 0.615, 0.6225, 0.63, 0.6375, 0.645, 0.6525, 0.66, 0.6675, 0.675, 0.6825, 0.69, 0.6975, 0.705, 0.7125, 
                             0.72, 0.7275, 0.735, 0.7425, 0.75, 0.7575, 0.765, 0.7725, 0.78, 0.7875, 0.795, 0.8025, 0.81, 0.8175, 0.825, 0.8325, 
                             0.84, 0.8475, 0.855, 0.8625, 0.87, 0.8775, 0.885, 0.8925, 0.9, 0.9075, 0.915, 0.9225, 0.93, 0.9375, 0.945, 0.9525, 
                             0.96, 0.9675, 0.975, 0.9825, 0.99, 0.9975, 1.005, 1.0125, 1.02, 1.0275, 1.035, 1.0425, 1.05, 1.0575, 1.065, 1.0725, 
                             1.08, 1.0875, 1.095, 1.1025, 1.11, 1.1175, 1.125, 1.1325, 1.14, 1.1475, 1.155, 1.1625, 1.17, 1.1775, 1.185, 1.1925, 
                             1.2, 1.2075, 1.215, 1.2225, 1.23, 1.2375, 1.245, 1.2525, 1.26, 1.2675, 1.275, 1.2825, 1.29, 1.2975, 1.305, 1.3125, 
                             1.32, 1.3275, 1.335, 1.3425};                                       
                                         
        float correction_array[10] = {0};
        Timer time_encoder_read;
        int8_t number_readings = 0;                                                 // counts the encoder readings placed into correction_array
        float correction_factor_result = 0;                                         // final correction result
        static float instant_speed;                                                        // the real speed depending on the motor turation

        // main task's callback function. Fcn called each period to implement task's functionality
	    virtual void _run();
    public:
        CorrectionFactor(uint32_t f_period, encoders::CQuadratureEncoderWithFilterTask &localencoder);
        float getCorrectionFactor();
        void apply_filter_mean(int16_t CF_table_index);
        void computeCarVelocity(int16_t motor_instant_rps);
        void compute_correction_factor_and_velocity();
        static float getInstantSpeed();
        
};

#endif