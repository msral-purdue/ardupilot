// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * Vicon.h
 *
 *  Created on: Jul 14, 2015
 *      Author: DRM
 */

#ifndef __VICON_H__
#define __VICON_H__

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <string.h>

#define VICON_TRANSLATION_RANGE				3500	/* in 1 mm */
#define VICON_ORIENTATION_RANGE				18000	/* in 0.01 deg */
#define VICON_PACKET_LENGTH					21   	/* head ID X Y Z Roll Pitch Yaw tail */
#define VICON_MSG_BUFFER_SIZE				256		/* Size of Vicon msg Rx buffer*/
#define VICON_MSG_BEGIN						'$'		/* 'Begin-message' character */
#define VICON_MSG_END						'&'		/* 'End-of-message' character */
#define VICON_MSG_IGNORE					'#'		/* 'Invalid-packet' character */
#define VICON_VEL_HZ						10		/* Rate at which to update velocity calculation */
#define VICON_VEL_THRESHOLD					20000	/* Maximum reasonable velocity (in cm/s) */
#define VICON_ID_DEFAULT         			0		/* Index in Vicon data array */

// Use SERIAL4 (UartE) for XBee/Vicon communication
#define XBEE	hal.uartE

/// @class      Vicon
class Vicon
{
public:

	Vicon(void);						// Constructor
	uint8_t vicon_success_count;		// valid packets in one second
	bool vicon_status;					// true if a valid link exists, false if not

	uint32_t last_update;               // time of last update in micro seconds

	void initialize(void);		// Flush UART Rx buffer & set poll rate for XBee serial data
	bool read_packet(void);				// read UART buffer to find packets
	void analyze_packet(void);			// process found packet to get position and orientation information
	uint8_t check_vicon_status(void);		// periodically check if the vicon link is still valid

	bool 		get_vicon_status() const { return vicon_status; }	//return vicon status
	char		get_ID() const { return ID; }
float       get_x() const { return _position.x; }			//return x in cm (NED frame)
	float       get_y() const { return _position.y; }		//return y in cm (NED frame)
	float       get_z() const { return _position.z; }		//return z in cm (NED frame)
	float       get_Vx() const { return _velocity.x; }		//return Vx in cm/s (NED frame)
	float       get_Vy() const { return _velocity.y; }		//return Vy in cm/s (NED frame)
	float       get_Vz() const { return _velocity.z; }		//return Vz in cm/s (NED frame)
	float 		get_roll(void) const { return roll; }		//return roll in radians (NED frame)
	float 		get_pitch(void) const { return pitch; }		//return pitch in radians (NED frame)
	float 		get_yaw(void) const { return yaw; }			//return heading in radians (NED frame)
	float 		get_roll_d(void) const;						//return roll in degrees (NED frame)
	float 		get_pitch_d(void) const;					//return pitch in degrees (NED frame)
	float 		get_yaw_d(void) const;						//return heading in degrees (NED frame)
	Vector3f	getPosNED() const { return _position; }; 	// return x,y,z in cm (in NED frame)
	Vector3f    getVelNED() const { return _velocity; }; 	// return Vx,Vy,Vz in cm/s (in NED frame)
	Vector3f	getPosNEU() const; 							// return x,y,z in cm (in NEU frame)
	Vector3f	getVelNEU() const;					 		// return Vx,Vy,Vz in cm/s (in NEU frame)

	static const struct AP_Param::GroupInfo var_info[];		// To enable Vicon parameters in GCS

	void reset_debug_count(void);	// Resets all debug counters to 0
	void print_debug_count(void);	// Print debug counter values to console

private:
	// Velocity Calculations
	uint32_t last_update_vel;	// Time since last velocity update in microseconds
	bool reset_prev_pos;

	// Vicon Data (stored in NED frame)
	AP_Int8		ID;				// This vehicle's index in the Vicon data array (parameter)
	Vector3f    _position; 		// position in 0.01 cm
	Vector3f	_position_prev;	// previous position (in mm), to calculate velocity
	Vector3f	_velocity;  	// velocity (Added by Daniel July 15 to fix compile errors)
	float roll;					// roll in radians
	float pitch; 				// pitch in radians
	float yaw;					// heading in radians

	// PARAMETER (Accessible from GCS)
	//AP_Int8 vicon_id;			// Vicon id (index) parameter

	// Message handling
	void reset_buffer(void);
	uint8_t vicon_fail_count;
	uint8_t buffer[VICON_MSG_BUFFER_SIZE];
	uint8_t ignored[VICON_MSG_BUFFER_SIZE];
	uint8_t packet_length;
	uint8_t i_read;				// Index of next message byte in buffer
	bool msgStarted; 			// True if read_packet() is in the middle of receiving a message

	// Debug counters
	int msgDrop_IGNORE;
	int msgDrop_BAD_LENGTH;
	int msgDrop_TOO_MANY_TRIES;
	int msgDrop_TRANS_OUT_OF_RANGE;
	int msgDrop_INVALID;
	int msgStartCount;
	int msg_incomplete;
};

#endif /* __VICON_H__ */
