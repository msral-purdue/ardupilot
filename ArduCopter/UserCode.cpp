/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
	//XBEE->begin(57600, 512, 256); // begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace
	XBEE->begin(57600); // begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace

	hal.console->println("Initializing XBEE Communications...");
	vicon.initialize();
	hal.console->println("\nXBEE Communications ready!");
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
	vicon.read_packet();

	// Log Vicon message in place of GPS
	if (should_log(MASK_LOG_GPS))
	{
		DataFlash.Log_Write_Vicon(vicon);	// Save to log file
	}
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
	//vicon.read_packet();
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
	//vicon.read_packet();
    // put your 1Hz code here
	uint8_t msgs = vicon.check_vicon_status();

	// DEBUG print vicon status information to usb console
	if(vicon.vicon_status) {
		Vector3f vel = vicon.getVelNEU();
		hal.console->printf("\n\nVICON connected (%d msgs)!\n",msgs);
		hal.console->printf("ID: %c\t",vicon.get_ID());
		hal.console->printf("Pos (cm): (%.2f,%.2f,%.2f)\t",
						   vicon.get_x(),vicon.get_y(),vicon.get_z());
		hal.console->printf("Vel (cm/s): (%.2f,%.2f,%.2f)\n",
								   vicon.get_Vx(),vicon.get_Vy(),vicon.get_Vz());
		hal.console->printf("R (rad): %.3f\t",vicon.get_roll());
		hal.console->printf("P (rad): %.3f\t",vicon.get_pitch());
		hal.console->printf("Y (rad): %.3f\n\n",vicon.get_yaw());
	} else {
		hal.console->printf("\n\nVICON disconnected...(%d msgs)\n\n",vicon.vicon_success_count);
	}

	vicon.print_debug_count();	// Print out debug counters
	vicon.reset_debug_count();	// Reset the debug counters

}
#endif
