/*
 * ardu_serial.h
 *
 *  Created on: 13-Jul-2022
 *      Author: Mahima Gupta
 */

#ifndef LIBRARIES_AP_ARDUINOI2C_ARDU_SERIAL_H_
#define LIBRARIES_AP_ARDUINOI2C_ARDU_SERIAL_H_

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>

class Ardu_Serial
{
public:

	Ardu_Serial();
    bool init_serial();
    static bool detect();
    bool get_reading();
    bool send_reading();

    bool init_once;

private :
    // pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr;

};

#endif /* LIBRARIES_AP_ARDUINOI2C_ARDU_SERIAL_H_ */
