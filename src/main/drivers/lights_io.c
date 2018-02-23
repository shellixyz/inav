#include "drivers/lights_io.h"
#include "drivers/io.h"

#ifdef USE_LIGHTS

#if (!defined(LIGHTS_USE_PCA9685_OUTPUT)) && (!defined(LIGHTS_OUTPUT_MODE))
    #define LIGHTS_OUTPUT_MODE IOCFG_OUT_PP
#endif


static IO_t wlightsIO = DEFIO_IO(NONE);
static IO_t flightsIO = DEFIO_IO(NONE);

bool lightsHardwareInit()
{
    wlightsIO = IOGetByTag(IO_TAG(WING_LIGHTS_PIN));
    flightsIO = IOGetByTag(IO_TAG(FRONT_LIGHTS_PIN));

    if (wlightsIO && flightsIO) {
        IOInit(wlightsIO, OWNER_LED, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(wlightsIO, LIGHTS_OUTPUT_MODE);
        IOInit(flightsIO, OWNER_LED, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(flightsIO, LIGHTS_OUTPUT_MODE);
        return(true);
    } else
        return(false);
}

void wingLightsHardwareSetStatus(bool status)
{
    if (wlightsIO)
        IOWrite(wlightsIO, status);
}

void frontLightsHardwareSetStatus(bool status)
{
    if (flightsIO)
        IOWrite(flightsIO, status);
}

#endif /* USE_LIGHTS */
