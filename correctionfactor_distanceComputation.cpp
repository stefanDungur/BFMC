/**
  ******************************************************************************
  * @file    CorrectionFactor.hpp
  * @author  BFMC
  * @version V1.0.0
  * @date    28-03-20
  * @brief   This file contains implementation of methods to compute the correction factor 
  * based on the encoder values(counts per resolution).
  ******************************************************************************
 */

#include <include/CorrectionFactor/correctionfactor.h>
#include <mbed.h>
#define PI 3.141592653589793


// constructor wich set the value of localEncoder
CorrectionFactor::CorrectionFactor(encoders::CQuadratureEncoderWithFilterTask encoder):local_encoder(encoder){

}

// compute mean for 3 values readed from the encoder 
void CorrectionFactor::computeEncoderValue(){

    // make a summ of 5 values from encoder ~ 5ms
    if(this->cnt_mean < 5)
    {    
        this->encoder_counts = local_encoder.getCount();
        this->cnt_summ = this->cnt_summ + this->encoder_counts;
        this->cnt_mean++;

#ifdef DEBUG
    g_rpi.printf("The mean was computed \r\n");
#endif

    }
    else
    {   
        if(this->cnt_mean >= 5){
            // this gives an integer of counts, can be also float for more exact result
            this->mean_encoder = this->cnt_summ / 5; 
            this->mean_computed = true; 
        }

    }


}


// compute the distance traveled by the car in a given period, 1ms in our case
void CorrectionFactor::computeDistance(){

    // must take value of CPR(counts per resolution) and PPR(pulses per resolution) from the main
    // deg_angle represents angle in radians for encoder disc
    
    // declarations
    int16_t CPR_enc,PPR_enc;
    int16_t CPR,PPR,deg_angle;
    float encoder_angle;
    float diameter = 0.02; //meters diameter of the wheel
    float radius = diameter / 2.;
    float arc_length;
    float wheel_radius = 0.0350; // wheel_circumference / ( 2* PI) meters
    float encoder_spin;

    // set the well known values - magic numbers
    CPR = 2048;
    PPR = CPR/4;
    deg_angle = 2*PI/PPR;


    // take the values of counts and rps from the encoder by read data fields
    //this->encoder_counts = local_encoder.getCount();
    //this->encoder_speed = local_encoder.getSpeedRps();
    
    // compute a mean of 5 values from encoder
    computeEncoderValue();

    // check if the mean is done
    if(this->mean_computed)
    {
        //compute encoder PPR and step angle from the mean value
        CPR_enc = mean_encoder;
        PPR_enc = CPR_enc / 4 / 100;
        encoder_angle = PPR_enc * deg_angle;

        // compute how much the encoder disc rotates in given period
        arc_length = radius * encoder_angle;
        
        // compute how much the disc of rotary encoder spins in period
        encoder_spin = (wheel_radius * arc_length)/radius;

        // final length, how much the car travels in given period
        this->real_distance = encoder_spin * wheel_radius / radius;
    }
    else{
        this->real_distance = -1;
    }

}


