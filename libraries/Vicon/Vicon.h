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

#define VICON_TRANSLATION_RANGE				25000	/*	in 0.01 cm	*/
#define VICON_ORIENTATION_RANGE				18000	/*	in 0.01 deg	*/
#define VICON_PACKET_LENGTH					24   	/* head ID X Y Z VX VY VZ Yaw tail */

/// @class      Vicon
class Vicon
{
public:

	uint8_t vicon_success_count;		// valid packets in one second
	bool vicon_status;					// true if a valid link exists, false if not

	uint32_t last_update;               // time of last update in ms

	bool read_packet(void);				// read UART buffer to find packets
	void analyze_packet(void);			// process found packet to get position and orientation information
	void check_vicon_status(void);		// periodically check if the vicon link is still valid


	bool 		get_vicon_status() const { return vicon_status; }	//return vicon status
	float       get_x() const { return _position.x; }	//return x in cm
	float       get_y() const { return _position.y; }	//return y in cm
	float       get_z() const { return _position.z; }	//return z in cm
	float 		get_yaw(void) const { return yaw; }		//return heading in radians
	Vector3f	getPosNEU() const; // return x,y,z in cm (in NEU frame)
	Vector3f	getVelNEU() const; // return x,y,z in cm (in NEU frame)


private:

	Vector3f    _position; 	// position in 0.01 cm
	Vector3f	_velocity;  // velocity (Added by Daniel July 15 to fix compile errors)
	float yaw;			// heading in radians

	uint8_t vicon_fail_count;

	uint8_t buffer[256];
	uint8_t packet_length;
};

#endif /* __VICON_H__ */
