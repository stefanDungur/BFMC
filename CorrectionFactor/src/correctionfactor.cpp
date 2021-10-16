/**
  ******************************************************************************
  * @file    CorrectionFactor.hpp
  * @author  BFMC
  * @version V1.2.0
  * @date    17-04-20
  * @brief   This file contains implementation of methods to compute the correction factor 
  * and instant speed based on the encoder values(rotations per second).
  ******************************************************************************
 */

#include <correctionfactor.h>

#if DEBUG_ON
    extern Serial g_rpi;
#endif

#define READ_PERIOD 5 //ms
#define FILTER_NUMBER_DATA 10.0

float CorrectionFactor::instant_speed = 0;                                                        // the real speed depending on the motor turation

// constructor wich set the value of localEncoder
CorrectionFactor::CorrectionFactor(uint32_t f_period, encoders::CQuadratureEncoderWithFilterTask &encoder) : task::CTask(f_period), local_encoder(encoder) {

}

// compute mean for x values readed from the encoder
void CorrectionFactor::compute_correction_factor_and_velocity(){

    int16_t CF_table_index = this->local_encoder.getSpeedRps();                                                 // take the values of encoder rps

    computeCarVelocity(CF_table_index);                                                                         // put the instant speed in the "instant_speed" data member

    apply_filter_mean(CF_table_index);                                                                          // compute the correction factor making a mean of 10 values

}

// take the instant velocity using the values reded from encoder
void CorrectionFactor::computeCarVelocity(int16_t motor_instant_rps){

    // CorrectionFactor::instant_speed = this->CF_table[motor_instant_rps];                                                     // get the instant velocity    

    int16_t CF_table_index = abs(motor_instant_rps); 
    CorrectionFactor::instant_speed = this->CF_table[CF_table_index] * (motor_instant_rps/CF_table_index);             // get the instant velocity and the polarity, V1.3

    // verify for NaN values
    if(isnan(this->getInstantSpeed())) {
        CorrectionFactor::instant_speed = 0.0f; // reset speed val if nan
    }

#ifdef DEBUG
    static int cnt = 0;
    if (cnt > 100) {
        g_rpi.printf("Instant speed: %f\r\n", CorrectionFactor::instant_speed);
        cnt = 0;
    }
    cnt++;
#endif

}


// compute a mean of 10 correction factor values
void CorrectionFactor::apply_filter_mean(int16_t CF_table_index){
    // apply CF only for positive car velocities
    if(this->getInstantSpeed() >= 0.0f) {
        if(number_readings < 10)                                    // assure that elements in the correction array are added once at 5ms
        {   
#ifdef DEBUG
                // g_rpi.printf("Encoder rps %d\r\n", CF_table_index);
#endif
                correction_array[number_readings] = this->CF_table[CF_table_index];                                      // the value of the correction factor will be found using the value of turation casted to int
#ifdef DEBUG
                // g_rpi.printf("Index array %d\r\n", number_readings);
#endif        
                number_readings++;    
        }
        else
        {
            if(number_readings >= 10){                                                                               // when the correction array is filled, a mean of its elements is made 

                float correction_sum = 0;                                                                            // compute the mean of 10 values readed from encoder
                for(int8_t index = 0; index < 10; index++){
                    correction_sum += correction_array[index];
                }      

                this->correction_factor_result = correction_sum / FILTER_NUMBER_DATA;                                // the mean result will be placed into the class argument
#if DEBUG_ON
            // g_rpi.printf("Corection factor result %.4f\r\n", this->correction_factor_result);
#endif
                number_readings = 0;
            }
        }
    }
    // otherwise reset CF array and reset CF 
    else {
        // reset CF array
        std::fill(correction_array, correction_array+10-1, 0);
        // reset CF value
        this->correction_factor_result = 0;
    }
    

}

// main task's callback function. Fcn called each period to implement task's functionality
void CorrectionFactor::_run() {
    // update correction fact and velocity
    this->compute_correction_factor_and_velocity();
}

// getter for the instant speed
float CorrectionFactor::getInstantSpeed(){                                                                          

    return CorrectionFactor::instant_speed;
}


// getter for the final correction value
float CorrectionFactor::getCorrectionFactor(){

    return this->correction_factor_result;
}

