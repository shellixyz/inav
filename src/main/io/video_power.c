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

#include "io/video_power.h"

#ifdef USE_VIDEO_POWER_SWITCH

#ifndef VIDEO_POWER_SWITCH_OUTPUT_MODE
    #define VIDEO_POWER_SWITCH_OUTPUT_MODE IOCFG_OUT_PP
#endif

typedef enum {
  VTX_PROTECTION_DISABLED,
  VTX_PROTECTION_WAIT_OFF,
  VTX_PROTECTION_ENABLED
} vtx_protection_state_e;

static IO_t videoIO = DEFIO_IO(NONE);
static bool video_power_status = false;

#ifdef VTX_PROTECTION
static vtx_protection_state_e vtx_protection_state = VTX_PROTECTION_ENABLED;
#endif


static void videoPowerSwitchSetStatus(bool status)
{
    if (videoIO && ((video_power_status == false) || ((video_power_status == true) && (!ARMING_FLAG(ARMED))))) {
        IOWrite(videoIO, status);
        video_power_status = status;
    }
}

/*
 * Video power switch handler function to be called periodically in loop. Updates switch
 * state via time schedule.
 */
void videoPowerSwitchUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
#ifdef VTX_PROTECTION
    if (!ARMING_FLAG(ARMED))
        switch (vtx_protection_state) {
            case VTX_PROTECTION_ENABLED:
                if (IS_RC_MODE_ACTIVE(BOXVIDEOPWR))
                    vtx_protection_state = VTX_PROTECTION_WAIT_OFF;
                break;
            case VTX_PROTECTION_WAIT_OFF:
                if (!IS_RC_MODE_ACTIVE(BOXVIDEOPWR))
                    vtx_protection_state = VTX_PROTECTION_DISABLED;
                break;
            default:
                videoPowerSwitchSetStatus(IS_RC_MODE_ACTIVE(BOXVIDEOPWR));
        }
#else
    videoPowerSwitchSetStatus(IS_RC_MODE_ACTIVE(BOXVIDEOPWR));
#endif
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
