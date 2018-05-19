#include "drivers/lights_io.h"
#include "drivers/io.h"

#ifdef USE_LIGHTS

#if (!defined(LIGHTS_USE_PCA9685_OUTPUT)) && (!defined(LIGHTS_OUTPUT_MODE))
    #define LIGHTS_OUTPUT_MODE IOCFG_OUT_PP
#endif


#ifdef WING_LIGHTS_PIN
static IO_t wlightsIO = DEFIO_IO(NONE);

static void wingLightsInit() {
    wlightsIO = IOGetByTag(IO_TAG(WING_LIGHTS_PIN));
    if (wlightsIO) {
        IOInit(wlightsIO, OWNER_LED, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(wlightsIO, LIGHTS_OUTPUT_MODE);
    }
}

void wingLightsHardwareSetStatus(bool status)
{
    if (wlightsIO)
        IOWrite(wlightsIO, status);
}
#endif

#ifdef FRONT_LIGHTS_PIN
static IO_t flightsIO = DEFIO_IO(NONE);

static void frontLightsInit() {
    flightsIO = IOGetByTag(IO_TAG(FRONT_LIGHTS_PIN));

    if (flightsIO) {
        IOInit(flightsIO, OWNER_LED, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(flightsIO, LIGHTS_OUTPUT_MODE);
    }
}

void frontLightsHardwareSetStatus(bool status)
{
    if (flightsIO)
        IOWrite(flightsIO, status);
}
#endif

void lightsHardwareInit()
{
#ifdef WING_LIGHTS_PIN
    wingLightsInit();
#endif
#ifdef FRONT_LIGHTS_PIN
    frontLightsInit();
#endif
}

#endif /* USE_LIGHTS */
