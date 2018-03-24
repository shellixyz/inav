/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "config/parameter_group.h"
#include "common/time.h"

#ifdef USE_VIDEO_POWER_SWITCH

typedef struct videoPowerConfig_s {
    uint16_t disarmed_video_off_delay;
} videoPowerConfig_t;

PG_DECLARE(videoPowerConfig_t, videoPowerConfig);

void videoPowerSwitchInit();
void videoPowerSwitchUpdate(timeUs_t currentTimeUs);

#endif /* USE_VIDEO_POWER_SWITCH */
