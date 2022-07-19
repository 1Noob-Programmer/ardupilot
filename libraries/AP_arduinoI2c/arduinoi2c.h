/*
 * arduinoi2c.h
 *
 *  Created on: 24-Jun-2022
 *      Author: Mahima Gupta
 */

#ifndef LIBRARIES_AP_ARDUINOI2C_ARDUINOI2C_H_
#define LIBRARIES_AP_ARDUINOI2C_ARDUINOI2C_H_

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define MAX_INSTANCE 5

class arduinoi2c{
private:
	//I2C initialisation
	AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

	int _num_instances;

	struct arduino_state{
		uint8_t addr;
	};

	arduino_state as;

	//Member Functions
	bool _add_backend_seed(arduinoi2c * _backend);

public:

	//private compiler
	arduinoi2c(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
	arduinoi2c();
	bool init_au;

	arduinoi2c *_drivers[MAX_INSTANCE];

	enum CMDtype {
		printWelcome
	};


	//Member Functions
	bool init();
	arduinoi2c* detect(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
	void sensor_init();
	bool write_on_arduino(CMDtype cmd,uint8_t address);
	bool send_commands(uint8_t address,CMDtype cmd);
};



#endif /* LIBRARIES_AP_ARDUINOI2C_ARDUINOI2C_H_ */
