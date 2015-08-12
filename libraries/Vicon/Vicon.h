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

#define VICON_TRANSLATION_RANGE				35000	/* in 0.01 cm */
#define VICON_ORIENTATION_RANGE				18000	/* in 0.01 deg */
#define VICON_PACKET_LENGTH					24   	/* head ID X Y Z VX VY VZ Yaw tail */
#define VICON_MSG_BUFFER_SIZE				256		/* Size of Vicon msg Rx buffer*/
#define VICON_MSG_BEGIN						'$'		/* 'Begin-message' character */
#define VICON_MSG_END						'&'		/* 'End-of-message' character */
#define VICON_MSG_IGNORE					'#'		/* 'Invalid-packet' character */
#define VICON_VEL_HZ						10		/* Rate at which to update velocity calculation */
#define VICON_VEL_THRESHOLD					20000	/* Maximum reasonable velocity */

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
	float       get_x() const { return _position.x; }	//return x in cm
	float       get_y() const { return _position.y; }	//return y in cm
	float       get_z() const { return _position.z; }	//return z in cm
	float       get_Vx() const { return _velocity.x; }	//return Vx in cm/s
	float       get_Vy() const { return _velocity.y; }	//return Vy in cm/s
	float       get_Vz() const { return _velocity.z; }	//return Vz in cm/s
	float 		get_yaw(void) const { return yaw; }		//return heading in radians
	Vector3f	getPosNEU() const; // return x,y,z in cm (in NEU frame)
	Vector3f	getVelNEU() const; // return x,y,z in cm (in NEU frame)


private:
	//Timing
	uint32_t last_update_vel;	// Time since last velocity update in microseconds

	// Vicon Data
	char		ID;
	Vector3f    _position; 		// position in 0.01 cm
	Vector3f	_position_prev;	// previous position (in mm), to calculate velocity
	Vector3f	_velocity;  	// velocity (Added by Daniel July 15 to fix compile errors)
	float yaw;					// heading in radians

	// Message handling
	void reset_buffer(void);
	uint8_t vicon_fail_count;
	uint8_t buffer[VICON_MSG_BUFFER_SIZE];
	uint8_t packet_length;
	uint8_t i_read;				// Index of next message byte in buffer
	bool msgStarted; 			// True if read_packet() is in the middle of receiving a message

};

#endif /* __VICON_H__ */
