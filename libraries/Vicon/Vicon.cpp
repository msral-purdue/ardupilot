/*
 * Vicon.cpp
 *
 *  Created on: Jul 14, 2015
 *      Author: DRM
 */


#include <AP_HAL.h>
#include <Vicon.h>

extern const AP_HAL::HAL& hal;

/*************************************************************************************************/
// calling at same frequency of the vicon system, or slightly faster
bool Vicon::read_packet()
{
	uint8_t i, j;
	uint8_t count = 0;

	if(hal.uartC->available() != -1)	// if there are bytes in RX buffer
	{
		count = hal.uartC->available();		// read available bytes in the buffer
		//hal.uartA->printf_P(PSTR("\n %d: "),count);	// for debug

		for(i = 0; i  < count; i++)
		{
			buffer[0] = hal.uartC->read();		// read byte
			//hal.uartA->printf_P(PSTR("%c"),buffer[0]);	// for debug

			if(buffer[0] == '$')	// looking for head of valid packet: '$'
			{
				for(j = 1; j  <= count - i; j++)	// read following bytes and fill the buffer with the packet
				{
					buffer[j] = hal.uartC->read();
					//hal.uartA->printf_P(PSTR("%c"),buffer[j]);	// for debug

					if(buffer[j] == '&')	// looking for packet tail
					{
						packet_length = j+1;
						//hal.uartA->printf_P(PSTR(" %d "), packet_length);		// for debug

						if(packet_length == 14)		// vicon packet construction: [ head signX XH XL signY YH YL signZ ZH ZL signYaw YawH YawL tail ], 14 bytes in total
						{
							analyze_packet();	// process packet information
							return true;	// return success
						}
						else
							return false;
					}
				}
			}
			else if(buffer[0] == '#')	// head of invalid packet
			{
				return false;
			}
		}
	}
	return false;
}

/*************************************************************************************************/
void Vicon::analyze_packet()
{
    uint8_t head, tail;
	uint8_t signX, signY, signZ, signYaw;
	uint16_t tmpX, tmpY, tmpZ, tmpYaw;	/* 	register stores absolute position in 0.1mm	*/
	int16_t yaw_sensor; 				// Added to fix compile errors

	head = buffer[0];
	tail = buffer[13];

	signX = buffer[1];
	signY = buffer[4];
	signZ = buffer[7];
	signYaw = buffer[10];

	// transfer two 8-bit bytes into a 16-bit value
	tmpX = buffer[2] << 8 | buffer[3];
	tmpY = buffer[5] << 8 | buffer[6];
	tmpZ = buffer[8] << 8 | buffer[9];
	tmpYaw = buffer[11] << 8 | buffer[12];

	// for debug
	//hal.uartA->printf_P(PSTR("head %c "),buffer[0]);
	//hal.uartA->printf_P(PSTR("tail %c "),buffer[13]);
	//hal.uartA->printf_P(PSTR("sign %c %c %c %c"),buffer[1],buffer[4],buffer[7],buffer[10]);
	//hal.uartA->printf_P(PSTR(" P %u %u %u %u "), tmpX, tmpY, tmpZ, tmpYaw);

	// check if packet is valid
	if(head == '$' || tail == '&'){
		if(signX == '+' || signX == '-'){
			if(signY == '+' || signY == '-'){
				if(signZ == '+' || signZ == '-'){
					/*	valid data is considered as range X,Y -2.5~2.5m, Z 0~2.5m, Yaw -180~180 degrees	*/
					if(tmpX <= VICON_TRANSLATION_RANGE && tmpY <= VICON_TRANSLATION_RANGE && tmpZ <= VICON_TRANSLATION_RANGE && tmpYaw <= VICON_ORIENTATION_RANGE)
					{
						if(signX == '+')
							_position.x = (float)tmpX;
						else if(signX == '-')
							_position.x = -(float)(tmpX);

						if(signY == '+')
							_position.y = (float)tmpY;
						else if(signY == '-')
							_position.y = -(float)(tmpY);

						if(signZ == '+')
							_position.z = (float)tmpZ;
						else if(signZ == '-')
							_position.z = (float)(-tmpZ);

						if(signYaw == '+')
							yaw_sensor = (int16_t)tmpYaw;
						else if(signYaw == '-')
							yaw_sensor = -(int16_t)(tmpYaw);

						/*	from 0.1mm to cm, int16_t to float	*/
						_position.x = _position.x / 100.0;
						_position.y = _position.y / 100.0;
						_position.z = _position.z / 100.0;
						yaw = radians( yaw_sensor * 0.01f);

						//hal.uartA->printf_P(PSTR(" P %f %f %f %d "), _position.x, _position.y, _position.z, yaw_sensor);

						// record vicon update time
						last_update = hal.scheduler->millis();

						// flag of valid packet
						vicon_success_count++;
					}
				}
			}
		}
	}else
	{
		_velocity.x = 0.0;
		_velocity.y = 0.0;
		_velocity.z = 0.0;
	}

	// reset buffer
	for(uint16_t i = 0; i < 256; i++)
	{
		buffer[i] = 0x00;
	}
}

/*************************************************************************************************/
// calling at 1 Hz, check if a valid link still exists between UAV and vicon system
void Vicon::check_vicon_status()	//1 Hz
{
	// if more than 25 packets have been received in one second, we consider a valid link exists within this second
	if(vicon_success_count > 25)
	{
		vicon_status = true;
		vicon_fail_count = 0;
	}
	else
	{
		// if not, we consider the link failed within this second
		vicon_fail_count++;
	}

	// if the link has been failed more than one second, we consider the link has lost
	if(vicon_fail_count > 1)
	{
		vicon_status = false;
	}

	// Commented out to fix compile errors
	//flag_count = 0;

}
