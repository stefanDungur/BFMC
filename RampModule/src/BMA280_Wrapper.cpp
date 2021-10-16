#include "RampModule/BMA280_Wrapper.h"

#define TIMING_BUGDET_DEPEND_ON_BANDWIDTH   64

// debug purposes only - Makefile C/CXX_FLAGS
#if DEBUG_ON
	extern Serial g_rpi;		// for console output - debug purposes only
#endif

static I2C *i2cPtr;


int16_t BMA280_Wrapper::get_accelero_x(){

    return this->accel_val.x;

}

int16_t BMA280_Wrapper::get_accelero_y(){

    return this->accel_val.y;

}

int16_t BMA280_Wrapper::get_accelero_z(){

    return this->accel_val.z;

}



BMA280_Wrapper::BMA280_Wrapper(I2C &i2c, struct bma2x2_t *bma2x2) : i2c(i2c), bma2x2(bma2x2) {
    // map I2C obj for I2C related functions
    i2cPtr = &(this->i2c);

    // set I2C freq to fast mode
    this->i2c.frequency(400000);

    // wait x ms
    wait_ms(5);
    

    // set up accel sensor
    BMA2x2_RETURN_FUNCTION_TYPE com_rslt = ERROR_BOSCH;
    com_rslt = bma2x2_shock_setup();

#if (DEBUG_ON == 1)
    if (com_rslt) {
        g_rpi.printf("Error at initialize the sensor\r\n");
    }
#endif

    // start accel timer for TB counting
    timerNextReading.start();
}

s32 BMA280_Wrapper::bma2x2_shock_setup()
{
    /*Setting the default value of the bandwidth*/
    u8 bw_value_u8 = BMA2x2_BW_7_81HZ;
    
    /**
    Initializing the com_rslt that is going to be returned as an ERROR_BOSCH
    It's going to change inside this function to something else
    Otherwise it will return the ERROR_BOSCH, meaning that something went wrong during the setup of the accelerometer
    */
    BMA2x2_RETURN_FUNCTION_TYPE com_rslt = ERROR_BOSCH;
    
    /*Calling the routine function to check if the I2C works correctly and to enable it in the meantime*/
    com_rslt = I2C_routine();
    
    /*Initializing the accelerometer and checking if it works accordingly*/
    com_rslt = bma2x2_init(bma2x2);
#if (DEBUG_ON == 1) 
	if (!com_rslt) {
		g_rpi.printf("Sensor initialized\r\n");
        g_rpi.printf("chip_id %X\r\n", bma2x2->chip_id);
	}
	else {
		g_rpi.printf("Error at BMA initialization. Code: %d\r\n", com_rslt);
        g_rpi.printf("chip_id %X\r\n", bma2x2->chip_id);
	}
#endif

    /*Executing a soft reset before initializing the accelerometer*/
    // required to clear garbage data remained in sensor regs
    u8 soft_reset = 0xB6;
    com_rslt = bma2x2_write_reg(0x14,&soft_reset,1);

#if (DEBUG_ON == 1) 
	if (!com_rslt) {
		g_rpi.printf("BMA reseted!\r\n");
        // g_rpi.printf("chip_id %X\r\n", bma2x2->chip_id);
	}
	else {
		g_rpi.printf("Error BMA reset. Code: %d\r\n", com_rslt);
        // g_rpi.printf("chip_id %X\r\n", bma2x2->chip_id);
	}
#endif

    /*If the is no ERROR_BOSCH then we can set the accelerometer to it's normal mode and set the default bandwith value*/
    if (!com_rslt)
    {
        com_rslt = bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

#if (DEBUG_ON == 1) 
        g_rpi.printf("Error BMA PW Mode. Code: %d\r\n", com_rslt);
#endif
    }
    else {
        return com_rslt;
    }


    if (!com_rslt)
    {
        com_rslt = bma2x2_set_bw(bw_value_u8);

#if (DEBUG_ON == 1) 
        g_rpi.printf("Error BMA BW setting. Code: %d\r\n", com_rslt);
#endif
    }
    
    // set the accelerometer range, in our case the maximum value from the accelerometer is +/-4096 // +/-2G
    com_rslt = bma2x2_set_range(BMA2x2_RANGE_2G);
    // theoretical value depending on number of G's

    // calibrate the sensor by computing a gain and a error offset
    if (!com_rslt) {
        calibrate_bma2x2_initial();
    }

    return com_rslt;
}

s8 BMA280_Wrapper::I2C_routine()
{
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_id
 *--------------------------------------------------------------------------*/
	bma2x2->bus_write = BMA2x2_I2C_bus_write;
	bma2x2->bus_read = BMA2x2_I2C_bus_read;
	bma2x2->delay_msec = BMA2x2_delay_msek;
	bma2x2->dev_addr = BMA2x2_I2C_ADDR1;

	return BMA2x2_INIT_VALUE;
}

#if (DEBUG_ON)

void BMA280_Wrapper::print_calibrated_val(){

    g_rpi.printf("Calibrated value x: %d \r\n",accel_val.x);
    g_rpi.printf("Calibrated value y: %d \r\n",accel_val.y);
    g_rpi.printf("Calibrated value z: %d \r\n",accel_val.z);
}

#endif

void BMA280_Wrapper::calibrate_bma2x2_initial()
{

    // initial values given to function

    // exprerimental data taken by hand
    int16_t x_plus_g_axes = 4129;
    int16_t x_minus_g_axes = -3927;
    int16_t y_plus_g_axes = 4103;
    int16_t y_minus_g_axes = -3931;
    int16_t z_plus_g_axes = 4127;
    int16_t z_minus_g_axes = -4144;
    
    // computation of the offset for each axes
    offset_for_output.x_offset_error = 0.5 * INT_TWO_DECIMALS * (x_plus_g_axes + x_minus_g_axes) / INT_TWO_DECIMALS ;
    offset_for_output.y_offset_error = 0.5 * INT_TWO_DECIMALS * (y_plus_g_axes + y_minus_g_axes) / INT_TWO_DECIMALS;
    offset_for_output.z_offset_error = 0.5 * INT_TWO_DECIMALS * (z_plus_g_axes + z_minus_g_axes) / INT_TWO_DECIMALS; 
    
    // compute the gain
    gain_for_output.x_gain = (0.5 * INT_TWO_DECIMALS * (x_plus_g_axes - x_minus_g_axes) / 4096) / INT_TWO_DECIMALS ;
    gain_for_output.y_gain = (0.5 * INT_TWO_DECIMALS * (y_plus_g_axes - y_minus_g_axes) / 4096) / INT_TWO_DECIMALS ;
    gain_for_output.z_gain = (0.5 * INT_TWO_DECIMALS * (z_plus_g_axes - z_minus_g_axes) / 4096) / INT_TWO_DECIMALS ;

    // if the value is 0 in int, we take the iddeal value of the gain = 1
    if(gain_for_output.x_gain == 0)
        gain_for_output.x_gain = 1;

    if(gain_for_output.y_gain == 0)
        gain_for_output.y_gain = 1;

    if(gain_for_output.z_gain == 0)
        gain_for_output.z_gain = 1;    

}

bool BMA280_Wrapper::read_and_calibrate_data_bma280() {
    bool updateStatus = false;

    // check if accel TB passed to read out data
    updateStatus = BMA280_read_updated_data();

    // perform calibration only if new data available
    if(updateStatus){
        // calibrate readout data on all axis
        this->accel_val.x = ((this->accel_val.x * INT_TWO_DECIMALS) - (offset_for_output.x_offset_error * INT_TWO_DECIMALS)) / (gain_for_output.x_gain * INT_TWO_DECIMALS);
    
        this->accel_val.y = ((this->accel_val.y * INT_TWO_DECIMALS) - (offset_for_output.y_offset_error * INT_TWO_DECIMALS)) / (gain_for_output.y_gain * INT_TWO_DECIMALS);
            
        this->accel_val.z = ((this->accel_val.z * INT_TWO_DECIMALS) - (offset_for_output.z_offset_error * INT_TWO_DECIMALS)) / (gain_for_output.z_gain * INT_TWO_DECIMALS);
    }

    // return updated data status    
    return updateStatus;
}

bool BMA280_Wrapper::BMA280_read_updated_data(){

    bool updatedValue = false;
    BMA2x2_RETURN_FUNCTION_TYPE com_rslt = ERROR_BOSCH;

    // if TB passed update sensor readings
    if(timerNextReading.read_ms() + 2 >= TIMING_BUGDET_DEPEND_ON_BANDWIDTH)
    {
#if DEBUG_ON
        // g_rpi.printf("%f\r\n", timerNextReading.read());
#endif

        // read all axis at once to speed up exec
        updatedValue = true;
        com_rslt = bma2x2_read_accel_xyz(&accel_data_xyz);

#if DEBUG_ON
        if (com_rslt) {
            g_rpi.printf("Error reading code: %d\r\n", com_rslt);
        }
#endif

        this->accel_val.x = accel_data_xyz.x;
        this->accel_val.y = accel_data_xyz.y;
        this->accel_val.z = accel_data_xyz.z;

        timerNextReading.stop();
        timerNextReading.reset();
        timerNextReading.start();
    }

    return updatedValue;

}

// I2C realated functions

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_id : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param len : The no of byte of data to be written
 */
s8 BMA2x2_I2C_bus_write(u8 dev_id, u8 reg_addr, u8 *reg_data, u8 len)
{
    s8 status = 0;          // Return 0 for Success, non-zero for failure 
    char write_data[10];    // data to be written
    int i;                  // array index

    // master start signal
    i2cPtr->start();

    // write i2c addr to write to
    if ((status = i2cPtr->write(dev_id<<1)) != ACK) {
#if DEBUG_ON
        g_rpi.printf("Write i2c addr err: %d\r\n", status);
#endif
        return status;
    }

    
    // send reg addr to write to
    write_data[0] = reg_addr;
    if ((status = i2cPtr->write(write_data[0])) != ACK) {
#if DEBUG_ON
        g_rpi.printf("Write reg addr errs: %d\r\n", status);
#endif
        return status;
    }

    // write the bytes of data
    for (i = 1; i <= len; i++) {
        // set data to be written
        write_data[i] = *reg_data;      // ACKnowledged read bits
        if ((status = i2cPtr->write(write_data[i])) != 1) {
            // if the written bytes not ACK
            return ERROR_BOSCH;
        }
        reg_data++;
    }

    // master stop signal
    i2cPtr->stop();

    if (status == ACK) {
        // all write op went fine
        status = SUCCESS_BOSCH;
    }

    return status;
}


/*   \Brief: The function is used as I2C bus read
 *    \Return : Status of the I2C read
 *    \param dev_id : The device address of the sensor
 *    \param reg_addr : Address of the first register,
 *            will data is going to be read
 *    \param reg_data : This data read from the sensor,
 *            which is hold in an array
 *    \param len : The no of byte of data to be read
 */
s8 BMA2x2_I2C_bus_read(u8 dev_id, u8 reg_addr, u8 *reg_data, u8 len)
{
    s8 status = 0;              // Return 0 for Success, non-zero for failure 

    // write phase of the read seq
    // master start signal
    i2cPtr->start();
    // write i2c dev addr shifted with 1 bit - datasheet(1st bit RW)
    if ((status = i2cPtr->write(dev_id<<1)) != ACK) {
#if DEBUG_ON
        g_rpi.printf("Read-wph err: %d\r\n", status);
#endif
        return status;
    }

    // send add to read from
    if ((status = i2cPtr->write(reg_addr)) != ACK) {
#if DEBUG_ON
        g_rpi.printf("Read-wph errs: %d\r\n", status);
#endif
        return status;
    }

    // read phase of the read seq
    // master start signal
    i2cPtr->start();
    // send i2c dev add and write op bit
    if ((status = i2cPtr->write((dev_id<<1) | 0x01)) != ACK) {
#if DEBUG_ON
        g_rpi.printf("Read-rph err: %d\r\n", status);
#endif
        return status;
    }
    
    // read bytes from slave one by one
    int i = 0;
    for (; i< len -1; i++) {
        *reg_data = i2cPtr->read(ACK);      // ACKnowledged read bits
        reg_data++;
    }
    *reg_data = i2cPtr->read(NACK);         // NACKnowledged read bits

    // master stop signal
    i2cPtr->stop();

    if (status == ACK) {
        // all read op went fine
        status = SUCCESS_BOSCH;
    }

    return status;
}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMA2x2_delay_msek(u32 waiting_ms)
{
	// wait x ms
	wait_ms(waiting_ms);
}