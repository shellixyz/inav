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

#include <stdint.h>

#include "platform.h"

#include "drivers/max7456_symbols.h"

#include "flight/imu.h"

#include "io/osd.h"

#include "navigation/navigation.h"


#define HUD_DRAWN_MAXCHARS 42 // 7 POI (1 home, 3 aicrafts, 3 waypoints) x 6 chars max for each
static int8_t hud_drawn[HUD_DRAWN_MAXCHARS][2];
static int8_t hud_drawn_pt;
extern displayPort_t *osdDisplayPort;

/* Overwrite all previously written positions on the OSD with a blank
 */

void osdHudClear()
{
    for (int i = 0; i < HUD_DRAWN_MAXCHARS; i++) {
        if (hud_drawn[i][0] > -1) {
            displayWriteChar(osdDisplayPort, hud_drawn[i][0], hud_drawn[i][1], SYM_BLANK);
            hud_drawn[i][0] = -1;
        }
    }
    hud_drawn_pt = 0;
}

/* Write a single char on the OSD, and record its position for the next loop
 */

void osdHudWrite(uint8_t x, uint8_t y, uint16_t symb)
{
    displayWriteChar(osdDisplayPort, x, y, symb);
    hud_drawn[hud_drawn_pt][0] = x;
    hud_drawn[hud_drawn_pt][1] = y;
    hud_drawn_pt++;
    if (hud_drawn_pt >= HUD_DRAWN_MAXCHARS)
        { hud_drawn_pt = 0; }
}


/* Squad, get the nearest POI
 */

int squadGetNearestPoi()
{
    int poi = -1;
    uint16_t min = 10000; // 10kms

    for (int i = 0; i < SQUAD_MAX_POIS; i++) {
         if ((squad_pois[i].state == 1) && ((squad_pois[i].distance) > 0) && ((squad_pois[i].distance) < min)) {
            min = squad_pois[i].distance;
            poi = i;
            }
        }
    return poi;
}

/* Squad, get the farthest POI
 */

int squadGetFarthestPoi()
{
    int poi = -1;
    uint16_t max = 0;

    for (int i = 0; i < SQUAD_MAX_POIS; i++) {
         if ((squad_pois[i].state == 1) && ((squad_pois[i].distance) > max) && ((squad_pois[i].distance) < 10000)) {
            max = squad_pois[i].distance;
            poi = i;
            }
        }
    return poi;
}

/* Display one POI on the hud, centered on crosshair position.
 * poiDistance and poiAltitude in meters, poiAltitude is relative to the aircraft (negative means below)
 */

void osdHudDrawPoi(uint32_t poiDistance, int16_t poiDirection, int32_t poiAltitude,
                          uint16_t poiSymbol)
    {
    int hud_poi_x;
    int hud_poi_y;
    uint8_t hud_center_x;
    uint8_t hud_center_y;
    uint8_t hud_range_x = 8;
    uint8_t hud_range_y = 4;
    bool hud_poi_is_oos = 0;

    int16_t hud_poi_error_x = poiDirection - DECIDEGREES_TO_DEGREES(osdGetHeading());
    osdCrosshairPosition(&hud_center_x, &hud_center_y);

    while (hud_poi_error_x < -179) {
        hud_poi_error_x += 360;
    }
    while (hud_poi_error_x > 180) {
        hud_poi_error_x -= 360;
    }

    if ((hud_poi_error_x > -osdConfig()->camera_fov_h / 2) && (hud_poi_error_x < osdConfig()->camera_fov_h / 2)) { // POI might be in sight, extra geometry needed
        float hud_scaled_x = sin_approx(DEGREES_TO_RADIANS(hud_poi_error_x)) / sin_approx(DEGREES_TO_RADIANS(osdConfig()->camera_fov_h / 2));
        hud_poi_x = hud_center_x + 15 * hud_scaled_x;
        }
    else {
        hud_poi_is_oos = 1; // POI is out of sight for sure
        }

    if ((hud_poi_x > hud_center_x + hud_range_x) || hud_poi_is_oos) { // Out of sight or out of hud area to the right
        hud_poi_x = hud_center_x + hud_range_x;
        hud_poi_y = constrain(poiAltitude / 20, -hud_range_y, hud_range_y);
        hud_poi_y += hud_center_y;
        osdHudWrite(hud_poi_x + 1, hud_poi_y, SYM_AH_RIGHT);
        }
    else if ((hud_poi_x < hud_center_x - hud_range_x) || hud_poi_is_oos) { // Out of sight or out of hud area to the left
        hud_poi_x = hud_center_x - hud_range_x;
        hud_poi_y = constrain(poiAltitude / 20, -hud_range_y, hud_range_y);
        hud_poi_y += hud_center_y;
        osdHudWrite(hud_poi_x + 1, hud_poi_y, SYM_AH_LEFT);
        }
    else { // On camera sight and in hud area
        float hud_poi_angle = atan2_approx(-poiAltitude, poiDistance);
        hud_poi_angle = RADIANS_TO_DEGREES(hud_poi_angle);
        int16_t hud_plane_angle = attitude.values.pitch / 10;
        int hud_camera_angle = osdConfig()->camera_uptilt;
        int16_t hud_poi_error_y = hud_poi_angle - hud_plane_angle + hud_camera_angle;

        float hud_scaled_y = sin_approx(DEGREES_TO_RADIANS(hud_poi_error_y)) / sin_approx(DEGREES_TO_RADIANS(osdConfig()->camera_fov_v / 2));

        // hud_poi_y = (IS_DISPLAY_PAL) ? hud_scaled_y * 8 : hud_scaled_y * 6.5;
        // hud_poi_y = constrain(hud_scaled_y, -hud_range_y, hud_range_y);
        // hud_poi_y += hud_center_y;

        hud_poi_y = constrain(8 * hud_scaled_y, -hud_range_y, hud_range_y);
        hud_poi_y += hud_center_y;
        }

    osdHudWrite(hud_poi_x, hud_poi_y, poiSymbol);

    char buff[3];
    if ((osd_unit_e)osdConfig()->units == OSD_UNIT_IMPERIAL) {
        osdFormatCentiNumber(buff, CENTIMETERS_TO_CENTIFEET(poiDistance * 100), FEET_PER_MILE, 0, 3, 3);
    }
    else {
        osdFormatCentiNumber(buff, poiDistance * 100, METERS_PER_KILOMETER, 0, 3, 3);
    }

    osdHudWrite(hud_poi_x - 1, hud_poi_y + 1, buff[0]);
    osdHudWrite(hud_poi_x , hud_poi_y + 1, buff[1]);
    osdHudWrite(hud_poi_x + 1, hud_poi_y + 1, buff[2]);

}
