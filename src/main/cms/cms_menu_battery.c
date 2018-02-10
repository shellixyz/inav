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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_CMS

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_misc.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#include "sensors/battery.h"

// Battery menu

static OSD_Entry menuBatteryEntries[]=
{
    { "-- BATTERY --", OME_Label, NULL, NULL, 0 },

#ifdef USE_ADC
    OSD_SETTING_ENTRY("CELL MAX", SETTING_VBAT_MAX_CELL_VOLTAGE),
    OSD_SETTING_ENTRY("CELL WARN", SETTING_VBAT_WARNING_CELL_VOLTAGE),
    OSD_SETTING_ENTRY("CELL MIN", SETTING_VBAT_MIN_CELL_VOLTAGE),
#endif /* USE_ADC */
    OSD_SETTING_ENTRY("CAP UNIT", SETTING_BATTERY_CAPACITY_UNIT),
    OSD_SETTING_ENTRY("CAPACITY", SETTING_BATTERY_CAPACITY),
    OSD_SETTING_ENTRY("CAP WARN", SETTING_BATTERY_CAPACITY_WARNING),
    OSD_SETTING_ENTRY("CAP CRIT", SETTING_BATTERY_CAPACITY_CRITICAL),

    { "BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBattery = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XBATT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuBatteryEntries
};

#endif // CMS
