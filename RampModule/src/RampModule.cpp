#include "RampModule/RampModule.h"
#include <correctionfactor.h>

#if DEBUG_ON
// debug purpose only
extern Serial g_rpi;
#endif

#define TIME_TILL_INIT_STATE    5500    // time to wait till next state is reached, in this case init

// car angle thresholds (with ramp) to change state
enum Treshold {
     NEGATIVE_ZERO_TH = -8,
     POSITIVE_ZERO_TH = 8,
};

// car state with respect to the ramp
enum RampState {
     UPHILL_RMP =  1,
     DOWNHILL_RMP = -1,
     FLAT_RMP = 0,
     ERROR_RMP = -2
};

// create a new object with the given parameters and than assign BMA_Wrapper obj from this class  the obtainted object in the
RampModule::RampModule(BMA280_Wrapper &bma280_obj, Serial& serialPort) : 
    CarModule(2, -1),
    bma280_obj(bma280_obj), serialPort(serialPort)
{

}

// update ramp module sensor components
void RampModule::updateComponents() {
    // update ramp angle
    compute_angle_three_axes_X();

    // static int cnt = 0;
    // if (cnt > 100) {
    //     g_rpi.printf("MC: %d\r\n", messCode);
    //     cnt = 0;
    // }
    // cnt++;
}

// update ramp angle
void RampModule::compute_angle_three_axes_X()
{
    // if req, update sensor readings
    // store into the object fields the calibrated data for the angle computation 
	bool rampUpdateStatus;
    rampUpdateStatus = this->bma280_obj.read_and_calibrate_data_bma280();

     if(rampUpdateStatus)
    {
        double angle_in_radians = 0;
        double angle_in_degrees = 0;

        //compute the tilt angle in radians
        double sqrt_coefficient = sqrt((double)(this->bma280_obj.get_accelero_y() * this->bma280_obj.get_accelero_y()) + (this->bma280_obj.get_accelero_z() * this->bma280_obj.get_accelero_z()));
        double angl = ((double)this->bma280_obj.get_accelero_x() / sqrt_coefficient);
        angle_in_radians = atan(angl);

        // transform the value from radians into degrees
        angle_in_degrees = (angle_in_radians * PI_DEGREE) / M_PI;

        // store the angle value in object attribute
        this->angle = angle_in_degrees;
    }
}

// get inclination state with the horizontal axis
int8_t RampModule::get_inclination_state()
{
    if (check_if_car_is_up_ramp())
        return UPHILL_RMP;
    else if (check_if_car_is_down_ramp())
        return DOWNHILL_RMP;
    else if (check_if_car_is_on_flat_road())
        return FLAT_RMP;
    
    // if no state recongnized, return err
    return ERROR_RMP;
}

bool RampModule::check_if_car_is_on_flat_road()
{
    if ((this->angle >= (NEGATIVE_ZERO_TH / 4)) && (this->angle <= (POSITIVE_ZERO_TH / 4)))
        return true;
    
    return false;
}

bool RampModule::check_if_car_is_up_ramp()
{
    if (this->angle > POSITIVE_ZERO_TH)
        return true;
    
    return false;
}

bool RampModule::check_if_car_is_down_ramp()
{
    if (this->angle < NEGATIVE_ZERO_TH)
        return true;
    
    return false;
}

// check positive car speed
bool RampModule::checkPositiveSpeed() {
    if (CorrectionFactor::getInstantSpeed() > 0.0f)
        return true;

    return false;
}

// check if intermediate delay time passed
bool RampModule::checkIntermediateTimePassed() {
    if (intDelayTimer.read_ms() >= TIME_TILL_INIT_STATE)
        return true;

    return false;
}

/******     Module action fcn     ********/

// send Rpi mess with ramp status
void RampModule::sendRampStatusMess()
{
    // retrive current ramp state
    int rampStatus = get_inclination_state();

    // send RPi state mess
    switch (rampStatus) {
        case UPHILL_RMP:
            serialPort.printf("@CUPR:event;;\r\n");     // car up ramp
            break;

        case DOWNHILL_RMP:
            serialPort.printf("@CDNR:event;;\r\n");     // car down ramp
            break;

        case FLAT_RMP:
            serialPort.printf("@CFLT:event;;\r\n");     // car down ramp
            break;

        default:
            // otherwise between final ramp state and init state(intermediate waiting state)
            // no mess req
            break;
    }
}


void RampModule::startIntermediateTimer() {
    this->intDelayTimer.reset();        // assure timer starts cnt fron 0
    this->intDelayTimer.start();        // start timer
}

// stop intermediate timer
void RampModule::stopIntermediateTimer() {
    this->intDelayTimer.stop();         // stop timer
    this->intDelayTimer.reset();        // assure timer starts cnt from 0
}
