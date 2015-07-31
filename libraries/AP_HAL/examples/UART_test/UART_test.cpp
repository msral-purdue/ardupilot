/*
  simple test of UART interfaces
 */
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <UARTDriver.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_HAL::UARTDriver* uarts[] = {
    hal.uartA, // console
};
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))
#define XBEE	hal.uartE

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}


void setup(void) 
{
    /*
      start all UARTs at 57600 with default buffer sizes
     */
    setup_uart(hal.uartA, "uartA (console)"); // console
    setup_uart(hal.uartB, "uartB (1st GPS)"); // 1st GPS
    setup_uart(hal.uartC, "uartC (TELEM1)"); // telemetry 1
    setup_uart(hal.uartD, "uartD (TELEM2)"); // telemetry 2
    setup_uart(hal.uartE, "uartE (2nd GPS/SERIAL4)"); // 2nd GPS
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, hal.scheduler->millis()*0.001f);
}

void loop(void) 
{	
    test_uart(hal.uartA, "uartA (console)");
    test_uart(hal.uartB, "uartB (1st GPS)");
    test_uart(hal.uartC, "uartC (TELEM1)");
    test_uart(hal.uartD, "uartD (TELEM2)");
    test_uart(hal.uartE, "uartE (2nd GPS/SERIAL4)");

    // Read from serial
    uint8_t i, j;
    uint8_t count = 0;
    uint8_t buffer[256];
    uint8_t packet_length;

    if(XBEE->available() > 0)	// if there are bytes in RX buffer
    	{
    		count = XBEE->available();		// read available bytes in the buffer

    		hal.uartA->printf("\nReceived %d bytes: \n",count);	// for debug

    		for(i = 0; i  < count; i++)
    		{
    			buffer[i] = XBEE->read();		// read byte
    			hal.uartA->printf("'%c",buffer[0]);	// for debug
    			hal.uartA->printf(" (%d)'\n",(int) buffer[0]);	// for debug

    			if(buffer[0] == '$')	// looking for head of valid packet: '$'
    			{
    				for(j = 1; j  < count - i; j++)	// read following bytes and fill the buffer with the packet
    				{
    					buffer[j] = XBEE->read();
    					//hal.uartA->printf_P(PSTR("%c"),buffer[j]);	// for debug

    					if(buffer[j] == '&')	// looking for packet tail
    					{
    						packet_length = j+1;
    						//hal.uartA->printf_P(PSTR(" %d "), packet_length);		// for debug

							// for debug
							hal.uartA->printf("head %c ",buffer[0]);
							hal.uartA->printf("tail %c \n",buffer[1]);
							//hal.uartA->printf_P(PSTR("sign %c %c %c %c"),buffer[1],buffer[4],buffer[7],buffer[10]);
							//hal.uartA->printf_P(PSTR(" P %u %u %u %u "), tmpX, tmpY, tmpZ, tmpYaw);
						}
						else{}
					}
				}
    			else if(buffer[0] == '#')	// head of invalid packet
    			{
    			}
    		}
    	}

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", hal.scheduler->millis()*0.001f);
#endif

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
