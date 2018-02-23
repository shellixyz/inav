#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include "platform.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "rx/rx.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "config/feature.h"

/*#include "config/parameter_group.h"*/
/*#include "config/parameter_group_ids.h"*/

#include "io/video_power.h"

#ifdef USE_VIDEO_POWER_SWITCH

#ifndef VIDEO_POWER_SWITCH_OUTPUT_MODE
    #define VIDEO_POWER_SWITCH_OUTPUT_MODE IOCFG_OUT_PP
#endif

/*
PG_REGISTER_WITH_RESET_TEMPLATE(lightsConfig_t, lightsConfig, PG_LIGHTS_CONFIG, 0);

PG_RESET_TEMPLATE(lightsConfig_t, lightsConfig,
        .failsafe = {
            .enabled = true,
            .flash_period = 1000,
            .flash_on_time = 100
        }
);
*/

static IO_t videoIO = DEFIO_IO(NONE);

/*static bool lights_on = false;*/
/*static timeUs_t last_status_change = 0;*/

/*static void videoPowerSwitchSetStatus(bool status, timeUs_t currentTimeUs)*/
static void videoPowerSwitchSetStatus(bool status)
{
    /*if (status != lights_on) {*/
        /*lights_on = status;*/
        /*wingLightsHardwareSetStatus(status);*/
        /*frontLightsHardwareSetStatus(status);*/
        /*last_status_change = currentTimeUs;*/
    /*}*/
    if (videoIO)
        IOWrite(videoIO, status);
}

/*
 * Lights handler function to be called periodically in loop. Updates lights
 * state via time schedule.
 */
void videoPowerSwitchUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    videoPowerSwitchSetStatus(IS_RC_MODE_ACTIVE(BOXVIDEOPWR));
}

void videoPowerSwitchInit()
{
    videoIO = IOGetByTag(IO_TAG(VIDEO_POWER_PIN));

    if (videoIO) {
        IOInit(videoIO, OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(videoIO, VIDEO_POWER_SWITCH_OUTPUT_MODE);
    }
}

#endif /* USE_VIDEO_POWER_SWITCH */
