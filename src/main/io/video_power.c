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
#include "fc/settings.h"

#include "scheduler/scheduler.h"

#include "config/feature.h"

#include "io/video_power.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#ifdef USE_VIDEO_POWER_SWITCH

#ifndef VIDEO_POWER_SWITCH_OUTPUT_MODE
    #define VIDEO_POWER_SWITCH_OUTPUT_MODE IOCFG_OUT_PP
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(videoPowerConfig_t, videoPowerConfig, PG_VIDEO_POWER_CONFIG, 0);

PG_RESET_TEMPLATE(videoPowerConfig_t, videoPowerConfig,
        .disarmed_video_off_delay = SETTING_DISARMED_VIDEO_OFF_DELAY_DEFAULT
);

typedef enum {
  VTX_PROTECTION_DISABLED,
  VTX_PROTECTION_WAIT_OFF,
  VTX_PROTECTION_ENABLED
} vtx_protection_state_e;

static IO_t videoIO = DEFIO_IO(NONE);
static bool video_power_status = false;

#ifdef VTX_PROTECTION
static timeUs_t disarmed_protection_timestamp = 0;
static vtx_protection_state_e vtx_protection_state = VTX_PROTECTION_ENABLED;
#endif


static void videoPowerSwitchSetStatus(bool status)
{
    if (videoIO) {
        video_power_status = status;
#ifdef VIDEO_POWER_SWITCH_INVERTED
        status = !status;
#endif
        IOWrite(videoIO, status);
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
    if (!ARMING_FLAG(ARMED) && video_power_status) {
        if (disarmed_protection_timestamp) {
            if (cmpTimeUs(currentTimeUs, disarmed_protection_timestamp) >= videoPowerConfig()->disarmed_video_off_delay * 1000000) {
                videoPowerSwitchSetStatus(false);
                vtx_protection_state = VTX_PROTECTION_ENABLED;
            }
        } else
            disarmed_protection_timestamp = currentTimeUs;
    } else
        disarmed_protection_timestamp = 0;

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
            if (!ARMING_FLAG(ARMED) || !video_power_status)
                videoPowerSwitchSetStatus(IS_RC_MODE_ACTIVE(BOXVIDEOPWR));
    }
#else
    videoPowerSwitchSetStatus(IS_RC_MODE_ACTIVE(BOXVIDEOPWR));
#endif
}

void videoPowerSwitchInit(void)
{
    videoIO = IOGetByTag(IO_TAG(VIDEO_POWER_PIN));

    if (videoIO) {
        IOInit(videoIO, OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(videoIO, VIDEO_POWER_SWITCH_OUTPUT_MODE);
        videoPowerSwitchSetStatus(false);
    }
}

#endif /* USE_VIDEO_POWER_SWITCH */
