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
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"


#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/fport.h"
#include "rx/pwm.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/rx_spi.h"
#include "rx/crsf.h"
#include "rx/eleres.h"
#include "rx/uib_rx.h"


//#define DEBUG_RX_SIGNAL_LOSS

const char rcChannelLetters[] = "AERT5678";

static uint16_t rssi = 0;                  // range: [0;1023]
static timeUs_t lastMspRssiUpdateUs = 0;

#define MSP_RSSI_TIMEOUT_US 1500000   // 1.5 sec

rssiSource_e rssiSource;

static bool rxDataProcessingRequired = false;
static bool auxiliaryProcessingRequired = false;

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static bool rxIsInFailsafeMode = true;

static timeUs_t rxNextUpdateAtUs = 0;
static uint32_t needRxSignalBefore = 0;
static uint32_t needRxSignalMaxDelayUs;
static uint32_t suspendRxSignalUntil = 0;
static uint8_t skipRxSamples = 0;

int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULS_TIME    300

#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeConfig_t rxRuntimeConfig;
static uint8_t rcSampleIndex = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 3);

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#ifndef DEFAULT_RX_TYPE
#define DEFAULT_RX_TYPE   RX_TYPE_NONE
#endif

#define RX_MIDRC 1500
#define RX_MIN_USEX 885
PG_RESET_TEMPLATE(rxConfig_t, rxConfig,
    .receiverType = DEFAULT_RX_TYPE,
    .rcmap = {0, 1, 3, 2, 4, 5, 6, 7},      // Default to AETR5678 map
    .halfDuplex = 0,
    .serialrx_provider = SERIALRX_PROVIDER,
    .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
    .spektrum_sat_bind = 0,
    .serialrx_inverted = 0,
    .midrc = RX_MIDRC,
    .mincheck = 1100,
    .maxcheck = 1900,
    .rx_min_usec = RX_MIN_USEX,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = 2115,         // any of first 4 channels above this value will trigger rx loss detection
    .rssi_channel = 0,
    .rssi_scale = RSSI_SCALE_DEFAULT,
    .rssiInvert = 0,
    .rcSmoothing = 1,
);

void resetAllRxChannelRangeConfigurations(void)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigsMutable(i)->min = PWM_RANGE_MIN;
        rxChannelRangeConfigsMutable(i)->max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);

void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    return RX_FRAME_PENDING;
}

static bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
    rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeConfig.rxSignalTimeout = DELAY_10_HZ;
    rxRuntimeConfig.requireFiltering = false;
    rcSampleIndex = 0;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        if (modeActivationConditions(i)->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationConditions(i)->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationConditions(i)->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    switch (rxConfig()->receiverType) {
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        case RX_TYPE_PWM:
        case RX_TYPE_PPM:
            rxPwmInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_SERIAL_RX
        case RX_TYPE_SERIAL:
            if (!serialRxInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

#ifdef USE_RX_MSP
        case RX_TYPE_MSP:
            rxMspInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_UIB
        case RX_TYPE_UIB:
            rxUIBInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_SPI
        case RX_TYPE_SPI:
            if (!rxSpiInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

        case RX_TYPE_NONE:
        default:
            // Already configured for NONE
            break;
    }

    if (rxConfig()->rssi_channel > 0) {
        rssiSource = RSSI_SOURCE_RX_CHANNEL;
    }
}

static uint8_t calculateChannelRemapping(const uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
{
    if (channelToRemap < channelMapEntryCount) {
        return channelMap[channelToRemap];
    }
    return channelToRemap;
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxSignal(void)
{
    failsafeOnRxSuspend();
    suspendRxSignalUntil = micros() + SKIP_RC_ON_SUSPEND_PERIOD;
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
}

void resumeRxSignal(void)
{
    suspendRxSignalUntil = micros();
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    failsafeOnRxResume();
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    if (rxSignalReceived) {
        if (currentTimeUs >= needRxSignalBefore) {
            rxSignalReceived = false;
        }
    }

#if defined(USE_PWM) || defined(USE_PPM)
    if (feature(FEATURE_RX_PPM)) {
        if (isPPMDataBeingReceived()) {
            rxDataProcessingRequired = true;
            rxSignalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            resetPPMDataReceivedState();
        }
    } else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        if (isPWMDataBeingReceived()) {
            rxDataProcessingRequired = true;
            rxSignalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    } else
#endif
    {
        const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig);
        if (frameStatus & RX_FRAME_COMPLETE) {
            rxDataProcessingRequired = true;
            rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
            rxSignalReceived = !rxIsInFailsafeMode;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }

        if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
            auxiliaryProcessingRequired = true;
        }
    }

    if (cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

    return rxDataProcessingRequired || auxiliaryProcessingRequired; // data driven or 50Hz
}

#define FILTERING_SAMPLE_COUNT  5
static uint16_t applyChannelFiltering(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RC_CHANNEL_COUNT][FILTERING_SAMPLE_COUNT];
    static bool rxSamplesCollected = false;

    // Update the recent samples
    rcSamples[chan][rcSampleIndex % FILTERING_SAMPLE_COUNT] = sample;

    // Until we have enough data - return unfiltered samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < FILTERING_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    // Assuming a step transition from 1000 -> 2000 different filters will yield the following output:
    //  No filter:              1000, 2000, 2000, 2000, 2000        - 0 samples delay
    //  3-point moving average: 1000, 1333, 1667, 2000, 2000        - 2 samples delay
    //  3-point median:         1000, 1000, 2000, 2000, 2000        - 1 sample delay
    //  5-point median:         1000, 1000, 1000, 2000, 2000        - 2 sample delay

    // For the same filters - noise rejection capabilities (2 out of 5 outliers
    //  No filter:              1000, 2000, 1000, 2000, 1000, 1000, 1000
    //  3-point MA:             1000, 1333, 1333, 1667, 1333, 1333, 1000    - noise has reduced magnitude, but spread over more samples
    //  3-point median:         1000, 1000, 1000, 2000, 1000, 1000, 1000    - high density noise is not removed
    //  5-point median:         1000, 1000, 1000, 1000, 1000, 1000, 1000    - only 3 out of 5 outlier noise will get through

    // Apply 5-point median filtering. This filter has the same delay as 3-point moving average, but better noise rejection
    return quickMedianFilter5_16(rcSamples[chan]);
}

void calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    const timeMs_t currentTimeMs = millis();

    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeConfig.rcProcessFrameFn(&rxRuntimeConfig);
    }

    rxFlightChannelsValid = true;
    
    // Read and process channel data
    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
        const uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration to flight channel
        if (channel < NON_AUX_CHANNEL_COUNT && sample != PPM_RCVR_TIMEOUT) {
            sample = scaleRange(sample, rxChannelRangeConfigs(channel)->min, rxChannelRangeConfigs(channel)->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
            sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);
        }

        // Store as rxRaw
        rcRaw[channel] = sample;

        // Apply invalid pulse value logic
        if (!isPulseValid(sample)) {
            sample = rcData[channel];   // hold channel, replace with old value
            if ((currentTimeMs > rcInvalidPulsPeriod[channel]) && (channel < NON_AUX_CHANNEL_COUNT)) {
                rxFlightChannelsValid = false;
            }
        } else {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        }

        // Update rcData channel value
        if (rxRuntimeConfig.requireFiltering) {
            rcData[channel] = sample;
        } else {
            rcData[channel] = applyChannelFiltering(channel, sample);
        }
    }

    // Update failsafe
    if (rxFlightChannelsValid && rxSignalReceived) {
        failsafeOnValidDataReceived();
    } else {
        failsafeOnValidDataFailed();
    }

    rcSampleIndex++;
}

void parseRcChannels(const char *input)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
            rxConfigMutable()->rcmap[s - rcChannelLetters] = c - input;
    }
}

void setRssiFiltered(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssi = newRssi;
}

#define RSSI_SAMPLE_COUNT 16
#define RSSI_MAX_VALUE 1023

void setRssiUnfiltered(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    static uint16_t rssiSamples[RSSI_SAMPLE_COUNT];
    static uint8_t rssiSampleIndex = 0;
    static unsigned sum = 0;

    sum = sum + rssiValue;
    sum = sum - rssiSamples[rssiSampleIndex];
    rssiSamples[rssiSampleIndex] = rssiValue;
    rssiSampleIndex = (rssiSampleIndex + 1) % RSSI_SAMPLE_COUNT;

    int16_t rssiMean = sum / RSSI_SAMPLE_COUNT;

    rssi = rssiMean;
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if (rssiSource == RSSI_SOURCE_MSP) {
        rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

#ifdef USE_RX_ELERES
static bool updateRSSIeleres(uint32_t currentTime)
{
    UNUSED(currentTime);
    rssi = eleresRssi();
    return true;
}
#endif // USE_RX_ELERES

static bool updateRSSIPWM(void)
{
    int16_t pwmRssi = 0;
    // Read value of AUX channel as rssi
    pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    rssi = (uint16_t)((constrain(pwmRssi - 1000, 0, 1000) / 1000.0f) * 1023.0f);

    return true;
}

#define RSSI_ADC_SAMPLE_COUNT 16

static bool updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
    return false;
#else
    static uint16_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
    static uint16_t adcRssiSampleIndex = 0;
    static timeUs_t rssiUpdateAtUs = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAtUs) < 0) {
        return false;
    }
    rssiUpdateAtUs = currentTimeUs + DELAY_50_HZ;

    adcRssiSampleIndex = (adcRssiSampleIndex + 1) % RSSI_ADC_SAMPLE_COUNT;
    adcRssiSamples[adcRssiSampleIndex] = adcGetChannel(ADC_RSSI);

    uint32_t adcRssiMean = 0;
    for (int sampleIndex = 0; sampleIndex < RSSI_ADC_SAMPLE_COUNT; sampleIndex++) {
        adcRssiMean += adcRssiSamples[sampleIndex];
    }

    rssi = (adcRssiMean / RSSI_ADC_SAMPLE_COUNT) / 4;    // Reduce to [0;1023]
    return true;
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{
    bool rssiUpdated = false;

    // Read RSSI
    switch (rssiSource) {
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIPWM();
        break;
    case RSSI_SOURCE_ADC:
        updateRSSIADC(currentTimeUs);
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > MSP_RSSI_TIMEOUT_US) {
            rssi = 0;
        }
        break;
    default:
#ifdef USE_RX_ELERES
        if (rxConfig()->receiverType == RX_TYPE_SPI && rxConfig()->rx_spi_protocol == RFM22_ELERES) {
            rssiUpdated = updateRSSIeleres(currentTimeUs);
        }
#endif
        break;
    }

    if (rssiUpdated) {
        // Apply RSSI inversion
        if (rxConfig()->rssiInvert) {
            rssi = 1023 - rssi;
        }

        // Apply scaling
        rssi = constrain((uint32_t)rssi * rxConfig()->rssi_scale / 100, 0, 1023);
    }
}

uint16_t getRSSI(void)
{
    return rssi;
}

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}
