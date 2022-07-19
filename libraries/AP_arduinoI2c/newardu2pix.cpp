/*
 * Newardu2pix.cpp
 *
 *  Created on: 25-Jun-2022
 *      Author: Mahima Gupta
 */

#include "newardu2pix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <inttypes.h>

const extern AP_HAL::HAL& hal;
#define ardu_addr 0x04	//arduino address

Newardu2pix::Newardu2pix(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev): _dev(std::move(dev))
{
	init_once = false;
	num_instances = 0;
	read_ardu = false;
	write_ardu = false;
	//_available = true;
}

Newardu2pix::Newardu2pix()
{
	//_available = true;
	init_once = false;
	num_instances = 0;
	read_ardu = false;
	write_ardu = false;
}

bool Newardu2pix::init()
{
    if (num_instances != 0) {
    	hal.console->printf("Newardu2pix::init 2nd time\n");
        // init called a 2nd time?
        return false;
    }
    init_once = true;
    detect_instance();
    write_ardu = true;

    hal.console->printf("Newardu2pix::init()\n");
    return true;
}



void Newardu2pix::detect_instance()
{
	hal.console->printf("Newardu2pix::detect_instance \n\n");
	FOREACH_I2C_EXTERNAL(i){_add_backend(
			detect(hal.i2c_mgr->get_device(i-1,ardu_addr)));
	}
}

bool Newardu2pix::_add_backend(Newardu2pix * backend)
{
    if (!backend)
    {
    	hal.console->printf("Newardu2pix::NO BACKEND \n\n");
    	init_once = false;
    	return false;
    }
    if(backend == nullptr)
    {
    	hal.console->printf("Newardu2pix::BACKEND == nullptr\n");
    	init_once = false;
    	return false;
    }
    if (num_instances >= MAX_INSTANCES)
    {
    	hal.console->printf("_num_instances %d \n\t",num_instances);
        AP_HAL::panic("Newardu2pix::Too many instance\n");
    }
    drivers[num_instances++] = backend;
    hal.console->printf("Newardu2pix::BACKEND\n");
    return true;
}

Newardu2pix *Newardu2pix::detect(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
	hal.console->printf("Newardu2pix::detect\n");
    if (!dev)
    {
    	hal.console->printf("Newardu2pix::no I2C\n");
    	init_once = false;
        return nullptr;
    }

    Newardu2pix *sensor = new Newardu2pix(std::move(dev));

    if (!sensor || !sensor->checking_arduino()) {
    	init_once = false;
    	delete sensor;
    	hal.console->printf("Sensor not found\n");
        return nullptr;
    }
    hal.console->printf("Sensor found\n");
    return sensor;
}

bool Newardu2pix::checking_arduino()
{
	hal.console->printf("checking arduino\n");
	const uint8_t check[] = {0xFF,0xFE,0xFF};
	uint8_t val;
	bool ret;

	_dev->get_semaphore()->take_blocking();

	_dev->set_retries(0);

	/*
	 * Check we get a response for firmware version to detect if sensor is there
	 */
	ret = _dev->transfer(check, sizeof(check), nullptr, 0);
	if (!ret) {
		hal.console->printf("checking arduino transfer fail\n");
		hal.console->printf("Fail  \n");
		_dev->get_semaphore()->give();
		return false;
		}

	    hal.scheduler->delay(100);

	if (!_dev->read_registers(0x01 , &val, 1))
		{
		hal.console->printf("checking arduino read fail\n");
		_dev->get_semaphore()->give();
		return false;
		}

	if(val){
		hal.console->printf("arduino is present.  \n");
		hal.console->printf("val ==  %u  \n",val);
		_dev->get_semaphore()->give();
		hal.scheduler->delay(100);
		return true;
	}

	return false;
}

void Newardu2pix::send_cmd(CMDtype cmd,uint8_t address , uint8_t action)
{
	CMDtype _cmd = cmd;
	int i=-1;
	uint8_t temp;
	if(!init_once)
	{
		hal.console->printf("Newardu2pix::not -initialized\n");
		return;
	}
	if(address != ardu_addr)
		{
			return;
		}
	if(address == ardu_addr)
		{
			i=0;
		}
	else
	{
		hal.console->printf("ardu_addrESS NOT VALID  \n  \t");
		init_once = false;
		return;
	}

	if(drivers[i]== nullptr)
	{
		hal.console->printf("CAN'T read\n  \t");
		return;
	}
	else{
		if(action == 1 && write_ardu)
		{
			temp = drivers[i]->write_to_arduino(_cmd,ardu_addr);
			hal.console->printf("successful/fail write===%u\n",temp);
			if(temp)
			{
				write_ardu = false;
				read_ardu = true;
			}
		}
		if(action == 0 && read_ardu)
		{
			temp = drivers[i]->read_from_arduino(_cmd,ardu_addr);
			hal.console->printf("successful/fail read===%u\n",temp);
			if(temp)
			{
				write_ardu = true;
				read_ardu = false;
			}
		}
	}
}
bool Newardu2pix::read_from_arduino(CMDtype cmd,uint8_t address)
{
	hal.console->printf("Newardu2pix:: read\n");

	if(write_ardu)
	{
		hal.console->printf("Newardu2pix::writing now -- CMD send failed\n");
		return false;
	}

	uint8_t CMD_read = 0x00;

	if(cmd == readWelcome)
	{
				CMD_read=0x02;
	}
	if(cmd == readgpsCoor)
	{
				CMD_read=0x03;
	}

	if(!_dev)
	{
		hal.console->printf("no 12c for reading\n");
		return false;
	}
	_dev->get_semaphore()->take_blocking();

	bool result;
	uint8_t read;

	_dev->read_registers(0x00,&read,1);

	result = _dev->read_registers(CMD_read, &read,1);

	if(result){
		hal.console->printf("Newardu2pix:: read CMD received successful\n");
		_dev->get_semaphore()->give();
		print_arduino_values(read,cmd);
			read_ardu = false;
			write_ardu = true;
		return true;
	}
	else{
		hal.console->printf("Newardu2pix:: read CMD send failed\n");
		_dev->get_semaphore()->give();
			read_ardu = true;
			write_ardu = false;
		return false;
	}
}

bool Newardu2pix::write_to_arduino(CMDtype cmd,uint8_t address)
{
	hal.console->printf("Newardu2pix::write\n");

	if(read_ardu)
		{
		hal.console->printf("Newardu2pix::reading now -- CMD send failed\n");
			return false;
	}

	uint8_t CMD_WRITE[] = {0xFF,0xFE,0xFE};

	switch(cmd)
			{
			case printWelcome:
			{
				CMD_WRITE[0]=0xFF;
				CMD_WRITE[1]=0xFF;
				CMD_WRITE[2]=0xFF;
				break;
			}
			case printgpsCoor:
			{
				CMD_WRITE[2]=0xF2;
				break;
			}
			default:
				CMD_WRITE[0]=0x00;
				CMD_WRITE[1]=0x00;
				CMD_WRITE[2]=0x00;
				break;
		}
	if(!_dev)
	{
		hal.console->printf("no 12c for writing\n");
		return false;
	}
	_dev->get_semaphore()->take_blocking();

	_dev->set_retries(5);

	bool result;

	result = _dev->transfer(CMD_WRITE,sizeof(CMD_WRITE), nullptr, 0);

	if(result){
		hal.console->printf("Newardu2pix::CMD send successful write\n");
		_dev->get_semaphore()->give();
			read_ardu = true;
			write_ardu = false;
		return true;
	}
	else{
		hal.console->printf("Newardu2pix::CMD send failed write\n");
		_dev->get_semaphore()->give();
		read_ardu = false;
		write_ardu = true;
		return false;
	}
}

//int32_t readfromarduino
void Newardu2pix::print_arduino_values(uint8_t readfromarduino,CMDtype cmd)
{
	hal.console->printf("reading from Arduino :\\n");
	hal.console->printf("Command = %d\n\n",cmd);
//	hal.console->printf("Value = %f\n\\n",readfromarduino/1e-7);
	char n = char(readfromarduino);
	hal.console->printf("Value = %c\n\n",n);
}
