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

#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/adc.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/runtime_config.h"

#include "config/feature.h"

#include "sensors/battery.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"

#include "io/beeper.h"

#include "navigation/navigation.h"

#include "build/debug.h"


#define ADCVREF 3300                                    // in mV (3300 = 3.3V)

#define VBATT_CELL_FULL_MAX_DIFF 14                     // Max difference with cell max voltage for the battery to be considered full (10mV steps)
#define VBATT_PRESENT_THRESHOLD 100                     // Minimum voltage to consider battery present
#define VBATT_STABLE_DELAY 40                           // Delay after connecting battery to begin monitoring
#define VBATT_HYSTERESIS 10                             // Batt Hysteresis of +/-100mV for changing battery state
#define VBATT_LPF_FREQ  1                               // Battery voltage filtering cutoff
#define AMPERAGE_LPF_FREQ  1                            // Battery current filtering cutoff


// Battery monitoring stuff
static uint8_t batteryCellCount = 3;                    // cell count
static uint16_t batteryFullVoltage;
static uint16_t batteryWarningVoltage;
static uint16_t batteryCriticalVoltage;
static uint32_t batteryRemainingCapacity = 0;
static bool batteryUseCapacityThresholds = false;
static bool batteryFullWhenPluggedIn = false;

static uint16_t vbat = 0;                               // battery voltage in 0.1V steps (filtered)
static uint16_t vbatLatestADC = 0;                      // most recent unsmoothed raw reading from vbat ADC
static uint16_t amperageLatestADC = 0;                  // most recent raw reading from current ADC
static uint16_t powerSupplyImpedance = 0;             // calculated impedance in milliohm
static uint16_t sagCompensatedVBat = 0;                       // calculated no load vbat

static int32_t amperage = 0;                            // amperage read by current sensor in centiampere (1/100th A)
static int32_t power = 0;                               // power draw in cW (0.01W resolution)
static int32_t mAhDrawn = 0;                            // milliampere hours drawn from the battery since start
static int32_t mWhDrawn = 0;                            // energy (milliWatt hours) drawn from the battery since start

batteryState_e batteryState;

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 2);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,

    .voltage = {
        .scale = VBAT_SCALE_DEFAULT,
        .cellDetect = 430,
        .cellMax = 420,
        .cellMin = 330,
        .cellWarning = 350
    },

    .current = {
        .offset = CURRENT_METER_OFFSET,
        .scale = CURRENT_METER_SCALE,
        .type = CURRENT_SENSOR_ADC
    },

    .capacity = {
        .value = 0,
        .warning = 0,
        .critical = 0,
        .unit = BAT_CAPACITY_UNIT_MAH,
    },

    .cruise = {
        .speed = 0,
        .power = 0
    },

    .rth_energy_margin = 5,
    .impedanceFiltering = false,
    .sagCompVBatFiltering = false

);

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.01V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 1100 = 11:1 voltage divider (10k:1k)
    return((uint64_t)src * batteryConfig()->voltage.scale * ADCVREF / (0xFFF * 1000));
}

int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t microvolts = ((uint32_t)src * ADCVREF * 100) / 0xFFF * 10 - (int32_t)batteryConfig()->current.offset * 1000;
    return microvolts / batteryConfig()->current.scale; // current in 0.01A steps
}

void batteryInit(void)
{
    batteryState = BATTERY_NOT_PRESENT;
    batteryCellCount = 1;
    batteryFullVoltage = 0;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;
}

static void updateBatteryVoltage(uint32_t vbatTimeDelta)
{
    uint16_t vbatSample;
    static pt1Filter_t vbatFilterState;

    // store the battery voltage with some other recent battery voltage readings
    vbatSample = vbatLatestADC = adcGetChannel(ADC_BATTERY);
    vbatSample = pt1FilterApply4(&vbatFilterState, vbatSample, VBATT_LPF_FREQ, vbatTimeDelta * 1e-6f);
    vbat = batteryAdcToVoltage(vbatSample);
}

void batteryUpdate(uint32_t vbatTimeDelta)
{
    updateBatteryVoltage(vbatTimeDelta);

    /* battery has just been connected*/
    if (batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD)
    {
        /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
        batteryState = BATTERY_OK;
        /* wait for VBatt to stabilise then we can calc number of cells
        (using the filtered value takes a long time to ramp up)
        We only do this on the ground so don't care if we do block, not
        worse than original code anyway*/
        delay(VBATT_STABLE_DELAY);
        updateBatteryVoltage(vbatTimeDelta);

        unsigned cells = (batteryAdcToVoltage(vbatLatestADC) / batteryConfig()->voltage.cellDetect) + 1;
        if (cells > 8) cells = 8; // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)

        batteryCellCount = cells;
        batteryFullVoltage = batteryCellCount * batteryConfig()->voltage.cellMax;
        batteryWarningVoltage = batteryCellCount * batteryConfig()->voltage.cellWarning;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->voltage.cellMin;

        batteryFullWhenPluggedIn = batteryAdcToVoltage(vbatLatestADC) >= (batteryFullVoltage - cells * VBATT_CELL_FULL_MAX_DIFF);
        batteryUseCapacityThresholds = feature(FEATURE_CURRENT_METER) && batteryFullWhenPluggedIn && (batteryConfig()->capacity.value > 0) &&
                                           (batteryConfig()->capacity.warning > 0) && (batteryConfig()->capacity.critical > 0);

    }
    /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD */
    else if (batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD) {
        batteryState = BATTERY_NOT_PRESENT;
        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
    }

    if (batteryState != BATTERY_NOT_PRESENT) {

        if ((batteryConfig()->capacity.value > 0) && batteryFullWhenPluggedIn) {
            uint32_t capacityDiffBetweenFullAndEmpty = batteryConfig()->capacity.value - batteryConfig()->capacity.critical;
            int32_t drawn = (batteryConfig()->capacity.unit == BAT_CAPACITY_UNIT_MWH ? mWhDrawn : mAhDrawn);
            batteryRemainingCapacity = (drawn > (int32_t)capacityDiffBetweenFullAndEmpty ? 0 : capacityDiffBetweenFullAndEmpty - drawn);
        }

        if (batteryUseCapacityThresholds) {
            if (batteryRemainingCapacity == 0)
                batteryState = BATTERY_CRITICAL;
            else if (batteryRemainingCapacity <= batteryConfig()->capacity.warning - batteryConfig()->capacity.critical)
                batteryState = BATTERY_WARNING;
        } else {
            switch (batteryState)
            {
                case BATTERY_OK:
                    if (vbat <= (batteryWarningVoltage - VBATT_HYSTERESIS))
                        batteryState = BATTERY_WARNING;
                    break;
                case BATTERY_WARNING:
                    if (vbat <= (batteryCriticalVoltage - VBATT_HYSTERESIS)) {
                        batteryState = BATTERY_CRITICAL;
                    } else if (vbat > (batteryWarningVoltage + VBATT_HYSTERESIS)){
                        batteryState = BATTERY_OK;
                    }
                    break;
                case BATTERY_CRITICAL:
                    if (vbat > (batteryCriticalVoltage + VBATT_HYSTERESIS))
                        batteryState = BATTERY_WARNING;
                    break;
                default:
                    break;
            }
        }

        // handle beeper
        if (ARMING_FLAG(ARMED) || !ARMING_FLAG(WAS_EVER_ARMED))
            switch (batteryState) {
                case BATTERY_WARNING:
                    beeper(BEEPER_BAT_LOW);
                    break;
                case BATTERY_CRITICAL:
                    beeper(BEEPER_BAT_CRIT_LOW);
                    break;
                default:
                    break;
            }
    }
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

bool batteryWasFullWhenPluggedIn(void)
{
    return batteryFullWhenPluggedIn;
}

bool batteryUsesCapacityThresholds(void)
{
    return batteryUseCapacityThresholds;
}

bool isBatteryVoltageConfigured(void)
{
    return feature(FEATURE_VBAT);
}

uint16_t getBatteryVoltage(void)
{
    return vbat;
}

uint16_t getSagCompensatedBatteryVoltage(void)
{
    return sagCompensatedVBat;
}

uint16_t getBatteryVoltageLatestADC(void)
{
    return vbatLatestADC;
}

uint16_t getBatteryWarningVoltage(void)
{
    return batteryWarningVoltage;
}

uint8_t getBatteryCellCount(void)
{
    return batteryCellCount;
}

uint16_t getBatteryAverageCellVoltage(void)
{
    if (batteryCellCount > 0) {
        return vbat / batteryCellCount;
    }
    return 0;
}

uint32_t getBatteryRemainingCapacity(void)
{
    return batteryRemainingCapacity;
}

bool isAmperageConfigured(void)
{
    return batteryConfig()->current.type != CURRENT_SENSOR_NONE;
}

int32_t getAmperage(void)
{
    return amperage;
}

int32_t getAmperageLatestADC(void)
{
    return amperageLatestADC;
}

int32_t getPower(void)
{
    return power;
}

int32_t getMAhDrawn(void)
{
    return mAhDrawn;
}

int32_t getMWhDrawn(void)
{
    return mWhDrawn;
}


void currentMeterUpdate(int32_t timeDelta)
{
    static pt1Filter_t amperageFilterState;
    static int64_t mAhdrawnRaw = 0;

    switch (batteryConfig()->current.type) {
        case CURRENT_SENSOR_ADC:
            amperageLatestADC = adcGetChannel(ADC_CURRENT);
            amperageLatestADC = pt1FilterApply4(&amperageFilterState, amperageLatestADC, AMPERAGE_LPF_FREQ, timeDelta * 1e-6f);
            amperage = currentSensorToCentiamps(amperageLatestADC);
            break;
        case CURRENT_SENSOR_VIRTUAL:
            amperage = batteryConfig()->current.offset;
            if (ARMING_FLAG(ARMED)) {
                throttleStatus_e throttleStatus = calculateThrottleStatus();
                int32_t throttleOffset = ((throttleStatus == THROTTLE_LOW) && feature(FEATURE_MOTOR_STOP)) ? 0 : (int32_t)rcCommand[THROTTLE] - 1000;
                int32_t throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
                amperage += throttleFactor * batteryConfig()->current.scale / 1000;
            }
            break;
        case CURRENT_SENSOR_NONE:
            amperage = 0;
            break;
    }

    mAhdrawnRaw += (amperage * timeDelta) / 1000;
    mAhDrawn = mAhdrawnRaw / (3600 * 100);
}

void powerMeterUpdate(int32_t timeDelta)
{
    static int64_t mWhDrawnRaw = 0;
    uint32_t power_mW = amperage * vbat / 10;
    power = amperage * vbat / 100; // power unit is cW (0.01W resolution)
    mWhDrawnRaw += (power_mW * timeDelta) / 10000;
    mWhDrawn = mWhDrawnRaw / (3600 * 100);
}

void sagCompensatedVBatUpdate(timeUs_t currentTime)
{
    static timeUs_t recordTimestamp = 0;
    static int32_t amperageRecord;
    static uint16_t vbatRecord;
    static timeUs_t lastUpdate = 0;
    static pt1Filter_t powerSupplyImpedanceFilterState;
    static pt1Filter_t sagCompVBatFilterState;
    static pt1Filter_t sagCompVBatFilterState2;


    if (batteryState == BATTERY_NOT_PRESENT) {

        recordTimestamp = 0;
        powerSupplyImpedance = 0;
        pt1FilterReset(&powerSupplyImpedanceFilterState, 0);
        pt1FilterReset(&sagCompVBatFilterState, vbat);
        pt1FilterInitRC(&sagCompVBatFilterState2, 10, 0);
        pt1FilterReset(&sagCompVBatFilterState2, vbat);

        sagCompensatedVBat = vbat;

    } else {

        if (cmpTimeUs(currentTime, recordTimestamp) > 20000000)
            recordTimestamp = 0;

        if (!recordTimestamp) {
            amperageRecord = amperage;
            vbatRecord = vbat;
            recordTimestamp = currentTime;
        /*} else if (ABS(amperage - amperageRecord) > 100) {*/
        } else if ((amperage - amperageRecord >= 400) && ((int16_t)vbatRecord - vbat >= 10)) {
            /*powerSupplyImpedanceRaw = (int32_t)(vbatRecord - vbat) * 1000 / (amperage - amperageRecord);*/
            uint16_t powerSupplyImpedanceSample = (int32_t)(vbatRecord - vbat) * 1000 / (amperage - amperageRecord);
            if (batteryConfig()->impedanceFiltering)
                powerSupplyImpedance = pt1FilterApply4(&powerSupplyImpedanceFilterState, powerSupplyImpedanceSample, VBATT_LPF_FREQ, cmpTimeUs(currentTime, lastUpdate) * 1e-6f);
            else
                powerSupplyImpedance = powerSupplyImpedanceSample;
            amperageRecord = amperage;
            vbatRecord = vbat;
            recordTimestamp = currentTime;
        }

        uint16_t sagCompensatedVBatSample = vbat + powerSupplyImpedance * amperage / 1000;
        if (batteryConfig()->sagCompVBatFiltering)
            sagCompensatedVBat = pt1FilterApply4(&sagCompVBatFilterState, sagCompensatedVBatSample, VBATT_LPF_FREQ, cmpTimeUs(currentTime, lastUpdate) * 1e-6f);
        else
            sagCompensatedVBat = sagCompensatedVBatSample;

        sagCompensatedVBat = MIN(batteryFullVoltage, sagCompensatedVBat);
    }

    DEBUG_SET(DEBUG_SAG_COMP_VOLTAGE, 0, powerSupplyImpedance);
    DEBUG_SET(DEBUG_SAG_COMP_VOLTAGE, 1, sagCompensatedVBat);
    DEBUG_SET(DEBUG_SAG_COMP_VOLTAGE, 2, pt1FilterApply3(&sagCompVBatFilterState2, sagCompensatedVBat, cmpTimeUs(currentTime, lastUpdate) * 1e-6f));

    // TODO: use unfiltered VBAT and amperage ?
    // TODO: filter sagCompensatedVBat
    lastUpdate = currentTime;
}

uint8_t calculateBatteryPercentage(void)
{
    if (batteryState == BATTERY_NOT_PRESENT)
        return 0;

    if (batteryFullWhenPluggedIn && feature(FEATURE_CURRENT_METER) && (batteryConfig()->capacity.value > 0) && (batteryConfig()->capacity.critical > 0)) {
        uint32_t capacityDiffBetweenFullAndEmpty = batteryConfig()->capacity.value - batteryConfig()->capacity.critical;
        return constrain(batteryRemainingCapacity * 100 / capacityDiffBetweenFullAndEmpty, 0, 100);
    } else
        return constrain((vbat - batteryCriticalVoltage) * 100L / (batteryFullVoltage - batteryCriticalVoltage), 0, 100);
}

int32_t calculateAveragePower() {
 return (int64_t)mWhDrawn * 360000000 / flyTime; // cW (0.01W)
}

#if defined(USE_ADC) && defined(USE_GPS)

int32_t remainingFlyTimeBeforeRTH() {
    if (!(feature(FEATURE_VBAT) && feature(FEATURE_CURRENT_METER) && navigationPositionEstimateIsHealthy() && (power != 0) && (batteryConfig()->cruise.power > 0) && (flyTime != 0) &&
            (batteryConfig()->cruise.speed > 0) && (batteryConfig()->capacity.unit == BAT_CAPACITY_UNIT_MWH) && (batteryConfig()->capacity.value > 0) && batteryFullWhenPluggedIn))
        return -1;

    int16_t wind_speed = 0; // m/s (unknown for the moment)

    if (wind_speed >= batteryConfig()->cruise.speed)
        return -2; // wind is too strong

    uint16_t time_to_home = GPS_distanceToHome / (batteryConfig()->cruise.speed - wind_speed);
    uint16_t energy_to_home = batteryConfig()->cruise.power * time_to_home / 3600;
    uint16_t energy_margin_abs = (batteryConfig()->capacity.value - batteryConfig()->capacity.critical) * batteryConfig()->rth_energy_margin / 100;
    int32_t remaining_energy_before_rth = batteryRemainingCapacity - energy_margin_abs - energy_to_home;

    if (remaining_energy_before_rth < 0) // No energy left = No time left
        return 0;

    int32_t averagePower = calculateAveragePower();

    if (averagePower == 0)
        return -1;

    uint32_t time_before_rth = remaining_energy_before_rth * 360 / averagePower;

    if (time_before_rth > 0x7FFFFFFF) // int32 overflow
        return -1;

    return time_before_rth;
}

#endif
