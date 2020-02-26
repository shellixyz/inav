/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "io/vtx_vs600.h"
#include "io/serial.h"

#include "telemetry/smartport.h"

typedef struct
{
    struct
    {
        int32_t value;
        uint32_t updateTimeUs;
    } altitude;
    struct
    {
        int32_t value;
        uint32_t updateTimeUs;
    } altitudeRate;
} SmartPortMasterVario;

typedef struct
{
    uint8_t cellCount;
    uint16_t volts[6];
    uint32_t updateTimeMs;
}  SmartPortMasterVoltage;

typedef struct
{
    uint32_t draw;
    uint32_t updateTimeUs;
}  SmartPortMasterCurrent;

typedef struct
{
    SmartPortMasterVario vario;
    SmartPortMasterVoltage voltage;
    SmartPortMasterCurrent current;
    SmartPortMasterVS600 vs600;
}  SmartPortMasterSensorData_t;

typedef struct smartPortMasterPayload_s {
    uint8_t data[7];
    uint8_t crc;
} __attribute__((packed)) smartPortMasterPayload_t;

typedef struct smartPortMasterFrame_s {
    uint8_t frameHead;
    uint8_t physicalID;
    smartPortMasterPayload_t payload;
} __attribute__((packed)) smartPortMasterFrame_t;

extern uint8_t activePhysicalId[];
extern uint8_t activePhysicalIdCnt;
extern uint16_t activeApplicationId[];
extern uint8_t activeApplicationIdCnt;

SmartPortMasterSensorData_t* getSmartPortMasterSensorData(void);

void checkSmartPortMasterTelemetryState(void);
void handleSmartPortMasterTelemetry(uint32_t currentTime);
bool initSmartPortMasterTelemetry(void);
void returnRawSensorData(serialPort_t *instance, smartPortMasterFrame_t* frame);
void writeFportToSerial(serialPort_t *instance, const smartPortPayload_t *payload);