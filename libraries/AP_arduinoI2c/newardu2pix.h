/*
 * Newardu2pix.h
 *
 *  Created on: 25-Jun-2022
 *      Author: Mahima Gupta
 */

#ifndef LIBRARIES_AP_ARDUINOI2C_NewarDU2PIX_H_
#define LIBRARIES_AP_ARDUINOI2C_NewarDU2PIX_H_

//#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#define changedvalue 0x84

#define check_val	10
#define MAX_INSTANCES 4

class Newardu2pix{

public :

	Newardu2pix();
	Newardu2pix(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

	enum CMDtype {
	    		printgpsCoor,
				readgpsCoor,
				printhello,
				readWelcome,
				readTemp,
	    };

	bool init();
    Newardu2pix *detect(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
	void detect_instance();
	bool _add_backend(Newardu2pix * backend);
	void backend_check(CMDtype cmd,uint8_t address , uint8_t action);
    bool read_from_arduino(CMDtype cmd,uint8_t address);
    bool write_to_arduino(CMDtype cmd,uint8_t address);
    bool checking_arduino();
    void print_arduino_values(uint8_t *readfromarduino,CMDtype cmd);


	Newardu2pix *drivers[MAX_INSTANCES];
    uint8_t num_instances;
    bool init_once;
    bool read_ardu;
    bool write_ardu;

private:
	 // start a reading
	    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif /* LIBRARIES_AP_ARDUINOI2C_NEWARDU2PIX_H_ */
