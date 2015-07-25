/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
	XBEE->begin(57600, 512, 256); // begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
	vicon.read_packet();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
	vicon.check_vicon_status();

	// DEBUG print vicon status information to usb console
	if(vicon.vicon_status) {
		hal.uartA->printf_P(PSTR("VICON connected!\n"));
	} else {
		hal.uartA->printf_P(PSTR("VICON disconnected...\n"));
	}
}
#endif
