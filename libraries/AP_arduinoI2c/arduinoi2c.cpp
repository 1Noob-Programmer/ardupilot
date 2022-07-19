/*
 * arduinoi2c.cpp
 *
 *  Created on: 24-Jun-2022
 *      Author: Mahima Gupta
 */

#include "arduinoi2c.h"
#include <AP_HAL/AP_HAL.h>

const extern AP_HAL::HAL& hal;
#define ADDR 0x16	//arduino address

#define GET_I2C_DEVICE(bus, address) _have_i2c_driver(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)


//parametrized constructor for initializing backend!!
arduinoi2c::arduinoi2c(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev):_dev(std::move(dev))
{
	_num_instances = 0;
	init_au = false;
}

arduinoi2c::arduinoi2c()
{
	_num_instances = 0;
	init_au = false;
}

//NEED TO BE CALLED ONLY ONCE
bool arduinoi2c::init()
{
	init_au = true;
	hal.console->printf("arduinoi2c::init\n");
	if (_num_instances != 0) {
		// init called a 2nd time?
		return false;
	}
	//initialize the sensor
	sensor_init();
	hal.console->printf("INIT_TRUE\n");

	return true;
}

void arduinoi2c::sensor_init()
{
	//look for i2c on external buses
	FOREACH_I2C_EXTERNAL(i){_add_backend_seed(
			detect(hal.i2c_mgr->get_device(i-1,ADDR)));
	}

}

bool arduinoi2c::_add_backend_seed(arduinoi2c * backend)
{

	if(!backend)
	{
		init_au = false;
		//no sensor is there
		return false;
	}

	if(_num_instances == MAX_INSTANCE)
	{
		AP_HAL::panic("Too many backends");
	}
	_drivers[_num_instances++] = backend;

	return true;
}

arduinoi2c* arduinoi2c::detect(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev){

	if (!dev) {
		init_au = false;
		hal.console->printf("not found I2C");
		return nullptr;
	}

	arduinoi2c *sensor= new arduinoi2c(std::move(dev));

	if (!sensor) {
		delete sensor;
		return nullptr;
	}

	hal.console->printf("address %u\t \n",as.addr);
	return sensor;
}



bool arduinoi2c::send_commands(uint8_t address,CMDtype cmd)
{
	//some higher number
	int i=-1;
	CMDtype _cmd = cmd;
	hal.console->printf("send_commands\n  \t");

	if(address == ADDR)
	{
		i=0;
	}
	else
	{
		hal.console->printf("SEEED ADDRESS NOT VALID  \n  \t");
		init_au = false;
		return false;
	}

	if(_drivers[i]== nullptr)
	{
		hal.console->printf("CAN'T WRITE\n  \t");
		return false;
	}
	else{
		hal.console->printf("printWelcome \n");
		_drivers[i]->write_on_arduino(_cmd,address);
		return true;
	}
}

bool arduinoi2c::write_on_arduino(CMDtype cmd,uint8_t address)
{
	hal.console->printf("write_on_arduino \n  \t");
	uint8_t CMD_WRITE[] = {0xFC,0xFB,0xFA};
		switch(cmd)
		{
			case printWelcome:
			{
				CMD_WRITE[0]=0xFF;
				CMD_WRITE[1]=0xFF;
				CMD_WRITE[2]=0xFF;
				break;
			}

		default:
			CMD_WRITE[0]=0x00;
			CMD_WRITE[1]=0x00;
			CMD_WRITE[2]=0x00;
			break;
	}

	_dev->get_semaphore()->take_blocking();

	// high retries for init
	_dev->set_retries(5);

//	hal.console->printf("CMD_WRITE A %u\n  \t",CMD_WRITE);
	bool ret;
	ret = _dev->transfer(CMD_WRITE, sizeof(CMD_WRITE), nullptr, 0);
	if (!ret) {
		goto fail;
	}

	_dev->get_semaphore()->give();
	hal.console->printf("Success_CMD_WRTIE \n  \t");
	return true;

	fail:
	hal.console->printf("Fail_CMD_WRITE \n \t");
	_dev->get_semaphore()->give();
	return false;

}


