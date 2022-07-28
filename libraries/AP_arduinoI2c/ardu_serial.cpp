/*
 * ardu_serial.cpp
 *
 *  Created on: 13-Jul-2022
 *      Author: Mahima Gupta
 */

#include <AP_HAL/AP_HAL.h>
#include "ardu_serial.h"
#include <ctype.h>

extern const AP_HAL::HAL& hal;


Ardu_Serial::Ardu_Serial()
{
	init_once = false;
}

bool Ardu_Serial::init_serial()
{

    hal.console->printf("INIT_serial\n");
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Arduino, 0);

	if (_uart == nullptr) {
		hal.console->printf("No serial port found\n");
		return false;
	}
	init_once = true;
	if(!detect())
	{
		init_once = false;
	}
    hal.console->printf("INIT_TRUE\n");
	return true;
}

/*
   detect if ARDUINO is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool Ardu_Serial::detect()
{
    hal.console->printf("detect..\n");
	hal.console->printf("detected = %d\n",AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Arduino, 0));
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Arduino, 0);
}

bool Ardu_Serial::get_reading()
{
	if (!_uart || !_uart->is_initialized()) {
		hal.console->printf("no UART /is not initialized yet");
		return false;
	}
	uint8_t nbytes = _uart->available();
	while (nbytes-- > 0) {
		char c = _uart->read();
		hal.console->printf("%c",c);
	}
	return true;
}

bool Ardu_Serial::send_reading()
{
	if (!_uart || !_uart->is_initialized()) {
		hal.console->printf("no UART /is not initialized yet");
		return false;
	}
	hal.console->printf("Ardu_Serial::send_command() \n");
	_uart->write("Sending Command to arduino?\n");
//	_uart->write("Hello ,How ru ?");
//	_uart->write("E");
//	_uart->write("l");
//	_uart->write("l");
//	_uart->write("o");

	return true;
}
