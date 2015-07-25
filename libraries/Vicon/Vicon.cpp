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

						if(packet_length == VICON_PACKET_LENGTH)		// vicon packet construction: [ head signX XH XL signY YH YL signZ ZH ZL signYaw YawH YawL tail ], 14 bytes in total
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
/* head ID X Y Z VX VY VZ Yaw tail */
/* X,Y,Z sent as mm and VX,VY,VZ as mm/s, Yaw in units of .01 degrees */
void Vicon::analyze_packet()
{
    uint8_t head, tail, ID;
	uint8_t signX, signY, signZ, signYaw;
	uint8_t signVX, signVY, signVZ;
	uint16_t tmpX, tmpY, tmpZ, tmpYaw;	/* 	register stores absolute position in 1 mm	*/
	uint16_t tmpVX, tmpVY, tmpVZ;
	int16_t yaw_sensor; 				// Added to fix compile errors

	head = buffer[0];
	tail = buffer[VICON_PACKET_LENGTH-1];

	ID    = buffer[1];

	signX = buffer[2];
	signY = buffer[5];
	signZ = buffer[8];
	signVX = buffer[11];
	signVY = buffer[14];
	signVZ = buffer[17];
	signYaw = buffer[20];

	// transfer two 8-bit bytes into a 16-bit value
	tmpX   = buffer[3] << 8 | buffer[4];
	tmpY   = buffer[6] << 8 | buffer[7];
	tmpZ   = buffer[9] << 8 | buffer[10];
	tmpVX  = buffer[12] << 8 | buffer[13];
	tmpVY  = buffer[15] << 8 | buffer[16];
	tmpVZ  = buffer[18] << 8 | buffer[19];
	tmpYaw = buffer[21] << 8 | buffer[22];

	// for debug
	//hal.uartA->printf_P(PSTR("head %c "),buffer[0]);
	//hal.uartA->printf_P(PSTR("tail %c "),buffer[13]);
	//hal.uartA->printf_P(PSTR("sign %c %c %c %c"),buffer[1],buffer[4],buffer[7],buffer[10]);
	//hal.uartA->printf_P(PSTR(" P %u %u %u %u "), tmpX, tmpY, tmpZ, tmpYaw);

	// check if packet is valid
	if( head == '$' && tail == '&' && (signX == '+' || signX == '-') &&
		(signY == '+' || signY == '-') && (signZ == '+' || signZ == '-') &&
		(signVX == '+' || signVX == '-') && (signVY == '+' || signVY == '-') &&
		(signVZ == '+' || signVZ == '-') )
	{
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

			if(signVX == '+')
				_velocity.x = (float)tmpVX;
			else if(signZ == '-')
				_velocity.x = (float)(-tmpVX);

			if(signVY == '+')
				_velocity.y = (float)tmpVY;
			else if(signZ == '-')
				_velocity.y = (float)(-tmpVY);

			if(signVZ == '+')
				_velocity.z = (float)tmpVZ;
			else if(signZ == '-')
				_velocity.z = (float)(-tmpVZ);

			if(signYaw == '+')
				yaw_sensor = (int16_t)tmpYaw;
			else if(signYaw == '-')
				yaw_sensor = -(int16_t)(tmpYaw);

			/*	from 1 mm to cm, int16_t to float	*/
			_position.x = _position.x / 10.0;
			_position.y = _position.y / 10.0;
			_position.z = _position.z / 10.0;

			/*	from 1 mm/s to cm/s, int16_t to float	*/
			_velocity.x = _velocity.x / 10.0;
			_velocity.y = _velocity.y / 10.0;
			_velocity.z = _velocity.z / 10.0;

			yaw = radians( yaw_sensor * 0.01f);

			//hal.uartA->printf_P(PSTR(" P %f %f %f %d "), _position.x, _position.y, _position.z, yaw_sensor);

			// record vicon update time
			last_update = hal.scheduler->millis();

			// flag of valid packet
			vicon_success_count++;
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

// return x,y,z in cm (in NEU frame)
Vector3f Vicon::getPosNEU() const
{
	// If Vicon X is north, Vicon Y is West
	Vector3f _position_NEU = _position;
	_position_NEU.y = -_position.y;
	return _position_NEU;
}

// return x,y,z in cm (in NEU frame)
Vector3f Vicon::getVelNEU() const
{
	// If Vicon X is north, Vicon Y is West
	Vector3f _velocity_NEU = _velocity;
	_velocity_NEU.y = -_velocity.y;
	return _velocity_NEU;
}
