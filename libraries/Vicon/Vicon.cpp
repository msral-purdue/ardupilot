/*
 * Vicon.cpp
 *
 *  Created on: Jul 14, 2015
 *      Author: DRM
 */

#include <AP_HAL.h>
#include <Vicon.h>

extern const AP_HAL::HAL& hal;

Vicon::Vicon() :
		vicon_success_count(0),
		vicon_status(false),
		last_update(0),
		last_update_vel(0),
		reset_prev_pos(false),
		_position(0.0f,0.0f,0.0f),
		_position_prev(0.0f,0.0f,0.0f),
		_velocity(0.0f,0.0f,0.0f),
		roll(0.0f),
		pitch(0.0f),
		yaw(0.0f),
		vicon_fail_count(0),
		packet_length(0),
		i_read(0),
		msgStarted(false),
		msgDrop_IGNORE(0),
		msgDrop_BAD_LENGTH(0),
		msgDrop_TOO_MANY_TRIES(0),
		msgDrop_TRANS_OUT_OF_RANGE(0),
		msgDrop_INVALID(0),
		msgStartCount(0),
		msg_incomplete(0)
{ }

void Vicon::initialize(void)
{
	//uint8_t ignore;
	while(XBEE->available() > 0)
	{
		//ignore = originally
		XBEE->read();	// Clear out all data on Rx line
	}
}

/*************************************************************************************************/
// calling at same frequency of the vicon system, or slightly faster
bool Vicon::read_packet()
{
	//uint8_t count = 0;
	while(XBEE->available() > 0 && i_read < VICON_PACKET_LENGTH) // Limit # of read attempts per function call
	{
		//count = XBEE->available();
		buffer[i_read] = XBEE->read();		// Read byte
		//hal.console->printf("Received (%d bytes): '%c'\n",count, (char) buffer[i_read]);
		if(!msgStarted && buffer[i_read] == VICON_MSG_IGNORE) // Ignore any packets starting with 'invalid-packet' character
		{
			i_read++;
			while(XBEE->available() > 0 && i_read < VICON_PACKET_LENGTH)
			{
				// Ignore remaining, consecutive,  'invalid-packet' characters
				buffer[i_read] = XBEE->read();
				if(buffer[i_read] != VICON_MSG_IGNORE)
				{ break; }
				else
				{ i_read++; }
			}

			if(i_read >= VICON_PACKET_LENGTH)
			{
				//hal.console->println("Received full packet of ignore characters...");
				// Reset buffer
				reset_buffer();
				msgDrop_IGNORE++;
				return false;
			}
		}

		if(!msgStarted)
		{
		 	if(buffer[i_read] == VICON_MSG_BEGIN) // Check for start of message
		 	{
				msgStarted = true;
				msgStartCount++;
				buffer[0] = VICON_MSG_BEGIN;	// Begin message in buffer
				i_read = 1;						// Reset message position index
		 	}
		 	else // ignore non-start characters
		 	{
		 		i_read++;
		 	}
		}
		else // continue message
		{
			if(buffer[i_read] == VICON_MSG_END)		// Received the 'end-of-message' character
			{
				if( i_read == (VICON_PACKET_LENGTH - 1) )
				{
					//hal.console->println("Received valid message!");
					analyze_packet();	// Process packet information
					return true;		// Return success
				}
				else	// 'End-of-message' char came too early ... could be part of a number
				{
					//hal.console->println("End of message came too early!");
					// Reset buffer
					// reset_buffer();
					// msgDrop_BAD_LENGTH++;
					// return false;
					i_read++;
				}
			}
			else
			{
				i_read++; 	// increment read counter
			}
		}
	} // End while XBEE->available() > 0 && i_read < VICON_PACKET_LENGTH

	if(i_read >= VICON_PACKET_LENGTH) // Didn't find end of packet
	{
		//hal.console->println("read_packet(): Tried too many times...");
		reset_buffer();
		msgDrop_TOO_MANY_TRIES++;
	}
	else
	{
		//hal.console->println("read_packet(): Nothing to read!");
	}

	msg_incomplete++;
	return false;		// Didn't receive a complete message
}

/*************************************************************************************************/
/* Resets packet Rx buffer */
void Vicon::reset_buffer(void)
{
	for(uint16_t i = 0; i < VICON_MSG_BUFFER_SIZE; i++)
	{
		buffer[i] = 0x00;
	}
	msgStarted = false;
	i_read = 0;
}

/*************************************************************************************************/
/* head ID X Y Z Roll Pitch Yaw tail */
/* X,Y,Z sent as mm, Roll/Pitch/Yaw in units of .01 degrees */
void Vicon::analyze_packet()
{
    uint8_t head, tail, ID_in;
	uint8_t signX, signY, signZ;
	uint8_t signRoll, signPitch, signYaw;
	uint16_t tmpX, tmpY, tmpZ;
	uint16_t tmpRoll, tmpPitch, tmpYaw;
	int16_t roll_sensor = 0;
	int16_t pitch_sensor = 0;
	int16_t yaw_sensor = 0;

	head = buffer[0];
	tail = buffer[VICON_PACKET_LENGTH-1];

	ID_in    = buffer[1];
	//ID = (int) ID_in;

	signX = buffer[2];
	signY = buffer[5];
	signZ = buffer[8];
	signRoll = buffer[11];
	signPitch = buffer[14];
	signYaw = buffer[17];

	// transfer two 8-bit bytes into a 16-bit value
	tmpX   = buffer[3] << 8 | buffer[4];
	tmpY   = buffer[6] << 8 | buffer[7];
	tmpZ   = buffer[9] << 8 | buffer[10];
	tmpRoll = buffer[12] << 8 | buffer[13];
	tmpPitch = buffer[15] << 8 | buffer[16];
	tmpYaw = buffer[18] << 8 | buffer[19];

	// for debug
	//hal.uartA->printf_P(PSTR("head %c "),head);
	//hal.uartA->printf_P(PSTR("tail %c "),tail);
	//hal.uartA->printf_P(PSTR("sign %c %c %c %c %c %c %c"),signX,signY,signZ,signVX,signVY,signVZ,signYaw);
	//hal.uartA->printf_P(PSTR(" P %u %u %u %u "), tmpX, tmpY, tmpZ, tmpYaw);

	//hal.uartA->printf("head %c  ",head);
	//hal.uartA->printf("tail %c  ",tail);
	//hal.uartA->printf("id %c  ",ID);
	//hal.uartA->printf("sign %c %c %c %c %c %c %c",signX,signY,signZ,signVX,signVY,signVZ,signYaw);
	//hal.uartA->printf(" P %u %u %u %u %u %u %u",tmpX,tmpY,tmpZ,tmpVX,tmpVY,tmpVZ,tmpYaw);

	// check if packet is valid
	if( ((uint8_t) ID) == ID_in && head == VICON_MSG_BEGIN && tail == VICON_MSG_END && (signX == '+' || signX == '-') &&
		(signY == '+' || signY == '-') && (signZ == '+' || signZ == '-') &&
		(signRoll == '+' || signRoll == '-') && (signPitch == '+' || signPitch == '-') &&
		(signYaw == '+' || signYaw == '-') )
	{

		// Check for zero position
		if(abs(_position.x) < 1e-9 && abs(_position.y) < 1e-9 && abs(_position.z) < 1e-9)
		{
			reset_prev_pos = true;
		}

		/*	valid data is considered as range X,Y -3.5~3.5m, Z 0~3.5m, Yaw -180~180 degrees	*/
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
				_position.z = -(float)(tmpZ);

			if(signRoll == '+')
				roll_sensor = (int16_t)tmpRoll;
			else if(signRoll == '-')
				roll_sensor = -(int16_t)(tmpRoll);

			if(signPitch == '+')
				pitch_sensor = (int16_t)tmpPitch;
			else if(signPitch == '-')
				pitch_sensor = -(int16_t)(tmpPitch);

			if(signYaw == '+')
				yaw_sensor = (int16_t)tmpYaw;
			else if(signYaw == '-')
				yaw_sensor = -(int16_t)(tmpYaw);

			// Reset previous position after each transition from zero to non-zero position
			if(reset_prev_pos)
			{
				_position_prev.x = _position.x;
				_position_prev.y = _position.y;
				_position_prev.z = _position.z;
				_velocity.x = 0; _velocity.y = 0; _velocity.z = 0;
				reset_prev_pos = false;
			}

			// Calculate velocity
			uint32_t this_time = hal.scheduler->micros();
			float dt = (this_time - last_update_vel)/1000000.0f;	// Change in time since last valid packet (in seconds)
			if( dt <= 0
			    || (abs(_position.x) < 1e-9 && abs(_position.y) < 1e-9 && abs(_position.z) < 1e-9 ))
			{
				// First time through or just lost connection (position zeroed)
				_velocity.x = 0; _velocity.y = 0; _velocity.z = 0;
			}
			else if ( dt < (1.0f/VICON_VEL_HZ) )
			{}	// Limit velocity calculations to VICON_VEL_HZ
			else
			{
				float dx = _position.x - _position_prev.x;
				float dy = _position.y - _position_prev.y;
				float dz = _position.z - _position_prev.z;

				/* Convert from 1 mm/s to cm/s, int16_t to float	*/
				_velocity.x = (dx / dt) / 10.0f;
				_velocity.y = (dy / dt) / 10.0f;
				_velocity.z = (dz / dt) / 10.0f;

				// Update previous position (position still in mm)
				_position_prev.x = _position.x;
				_position_prev.y = _position.y;
				_position_prev.z = _position.z;

				last_update_vel = hal.scheduler->micros();
			}

			// Sanity check on velocity (zero out unreasonably high velocities)
			if(_velocity.x > VICON_VEL_THRESHOLD || _velocity.x < -VICON_VEL_THRESHOLD) { _velocity.x = 0; hal.console->println("\n\nKILLING X VELOCITY!");}
			if(_velocity.y > VICON_VEL_THRESHOLD || _velocity.y < -VICON_VEL_THRESHOLD) { _velocity.y = 0; hal.console->println("\n\nKILLING Y VELOCITY!");}
			if(_velocity.z > VICON_VEL_THRESHOLD || _velocity.z < -VICON_VEL_THRESHOLD) { _velocity.z = 0; hal.console->println("\n\nKILLING Z VELOCITY!");}

			/* Convert from 1 mm to cm, int16_t to float */
			_position.x = _position.x / 10.0f;
			_position.y = _position.y / 10.0f;
			_position.z = _position.z / 10.0f;

			/* Convert from .01 deg to radians */
			roll = radians( roll_sensor * 0.01f);
			pitch = radians( pitch_sensor * 0.01f);
			yaw = radians( yaw_sensor * 0.01f);

			//hal.uartA->printf_P(PSTR(" P %f %f %f %d "), _position.x, _position.y, _position.z, yaw_sensor);

			// record vicon update time
			last_update = hal.scheduler->micros();

			// flag of valid packet
			vicon_success_count++;
		}
		else
		{
			msgDrop_TRANS_OUT_OF_RANGE++;
		}
	}
	else if( ((uint8_t) ID) != ID_in)
	{
		// Ignore messages
	}
	else // Invalid data packet received (zero out velocity)
	{
		msgDrop_INVALID++;
		_velocity.x = 0.0;
		_velocity.y = 0.0;
		_velocity.z = 0.0;
	}

	// reset buffer
	reset_buffer();
}

/*************************************************************************************************/
// calling at 1 Hz, check if a valid link still exists between UAV and vicon system
uint8_t Vicon::check_vicon_status()	//1 Hz
{
	uint8_t msgs = vicon_success_count;	// Return the number of valid messages received

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

	// Reset valid packet counter
	vicon_success_count = 0;

	return msgs;
}

// return x,y,z in cm (in NEU frame)
Vector3f Vicon::getPosNEU() const
{
	// Vicon XYZ pos. is stored in NED...just need to negate Z
	Vector3f _position_NEU = _position;
	_position_NEU.z = -1 * _position_NEU.z;

	// DEBUG: Negate XY for position hold (account for 'home' position)
	_position_NEU.x = -1 * _position_NEU.x;
	_position_NEU.y = -1 * _position_NEU.y;
	return _position_NEU;
}

// return x,y,z in cm (in NEU frame)
Vector3f Vicon::getVelNEU() const
{
	// Vicon XYZ vel. is stored in NED...just need to negate Z
	Vector3f _velocity_NEU = _velocity;
	_velocity_NEU.z = -1 * _velocity_NEU.z;

	// DEBUG: Negate XY for position hold (account for 'home' position)
	_velocity_NEU.x = -1 * _velocity_NEU.x;
	_velocity_NEU.y = -1 * _velocity_NEU.y;
	return _velocity_NEU;
}

//return roll in degrees (NED frame)
float Vicon::get_roll_d(void) const
{
	return degrees(roll);
}

//return pitch in degrees (NED frame)
float Vicon::get_pitch_d(void) const
{
	return degrees(pitch);
}

//return heading in degrees (NED frame)
float Vicon::get_yaw_d(void) const
{
	return degrees(yaw);
}

// Reset all packet reading debug counters
void Vicon::reset_debug_count(void)
{
	msgDrop_IGNORE = 0;
	msgDrop_BAD_LENGTH = 0;
	msgDrop_TOO_MANY_TRIES = 0;
	msgDrop_TRANS_OUT_OF_RANGE = 0;
	msgDrop_INVALID = 0;
	msgStartCount = 0;
	msg_incomplete = 0;
}

void Vicon::print_debug_count(void)
{
	hal.console->printf("\n\nmsgDrop_IGNORE: %d\nmsgDrop_BAD_LENGTH: %d\n\
						msgDrop_TOO_MANY_TRIES: %d\nmsgDrop_INVALID: %d\n\
						\nmsgDrop_TRANS_OUT_OF_RANGE: %d\nmsgStartCount: %d\
			            \nmsg_incomplete: %d\n\n",msgDrop_IGNORE,\
						msgDrop_BAD_LENGTH,msgDrop_TOO_MANY_TRIES,\
						msgDrop_INVALID,msgDrop_TRANS_OUT_OF_RANGE,\
						msgStartCount,msg_incomplete);
}

// Add Vicon ID parameter as a standard parameter (for access in GCS)
const AP_Param::GroupInfo Vicon::var_info[] PROGMEM = {
	// @Param: ID
	// @DisplayName: Vicon ID
	// @Description: This vehicle's index in the array of Vicon data
	// @Range: 0 10
	// @Increment: 1
	// @User: Standard
	AP_GROUPINFO("ID",    0, Vicon, ID, VICON_ID_DEFAULT),

	AP_GROUPEND
};
