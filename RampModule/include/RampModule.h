#ifndef RAMPMODULE_H_
#define RAMPMODULE_H_

#include "BMA280_Wrapper.h"
#include "CarModule.h"

// forward declaration for typedefs
class RampModule;
typedef bool (RampModule::*RMP_Check_Fn)();
typedef void (RampModule::*RMP_Action_Fn)();

// define the trasholds for the module


class RampModule : public CarModule
{
private:
     BMA280_Wrapper& bma280_obj;        // accel sensor wrapper
     Serial& serialPort;                // serial obj used for sending mess to rpi
     double angle;                      // ramp module angle

     // intermediate state
     Timer intDelayTimer;

public:
     // class constr
     RampModule(BMA280_Wrapper& bma280_obj, Serial& serialPort);

     // update components fcn
     void updateComponents();

     // update ramp angle
     void compute_angle_three_axes_X();

     // get inclination state with the horizontal axis
     int8_t get_inclination_state();

     /******     Module checking fcn     ********/

     // check if car is on flat road
     bool check_if_car_is_on_flat_road();

     // check if car is going up ramp
     bool check_if_car_is_up_ramp();

     // check if car is going down ramp
     bool check_if_car_is_down_ramp();

     // check positive car speed
    bool checkPositiveSpeed();

    // check if intermediate delay time passed
    bool checkIntermediateTimePassed();


     /******     Module action fcn     ********/

     // send Rpi mess with ramp status
     void sendRampStatusMess();

     // start intermediate timer
     void startIntermediateTimer();

     // stop intermediate timer
     void stopIntermediateTimer(); 
}; 

#endif