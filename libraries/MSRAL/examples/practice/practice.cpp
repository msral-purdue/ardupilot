/*
 *  A practice sketch for learning/trying new things
 */
 
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Rally.h>
#include <AP_NavEKF.h>
#include <AP_Scheduler.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// A variable to keep track of time
uint32_t timer;

// For UART/Serial tests (based on the UART_test.cpp file)
static AP_HAL::UARTDriver* uarts[] = {
    hal.uartA, // console
};
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))

/*
  Setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}

void setup()
{
    hal.console->println("Practice sketch starting up...");
    hal.scheduler->delay(1000);

    /*
      Start all UARTs at 57600 with default buffer sizes
     */
    setup_uart(hal.uartA, "uartA"); // console
    setup_uart(hal.uartB, "uartB"); // 1st GPS
    setup_uart(hal.uartC, "uartC"); // telemetry 1
    setup_uart(hal.uartD, "uartD"); // telemetry 2
    setup_uart(hal.uartE, "uartE"); // 2nd GPS
    
    timer = hal.scheduler->micros();
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

void loop()
{
    // 1 Hz loop
    if((hal.scheduler->micros() - timer) > 1000000UL) {
        timer = hal.scheduler->micros();
        hal.console->println("Looping @ 1 Hz ...");
        test_uart(hal.uartA, "uartA");
    }
}

AP_HAL_MAIN();
