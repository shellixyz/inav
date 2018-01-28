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

#include "platform.h"

#ifdef USE_SERIALRX_SBUS

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
<<<<<<<
=======
#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif
>>>>>>>

#include "rx/rx.h"
#include "rx/sbus.h"
<<<<<<<

#include "telemetry/telemetry.h"
=======
#include "rx/sbus_channels.h"
>>>>>>>

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_TIME_NEEDED_PER_FRAME 3000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#define SBUS_BAUDRATE 100000
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};

static uint16_t sbusStateFlags = 0;

<<<<<<<
=======
#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};

static uint16_t sbusStateFlags = 0;

static bool sbusFrameDone = false;

struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
>>>>>>>
static bool sbusFrameDone = false;

struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

static sbusFrame_t sbusFrame;

// Receive ISR callback
static void sbusDataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    static uint8_t sbusFramePosition = 0;
    static uint32_t sbusFrameStartAt = 0;
    timeUs_t now = micros();

    timeDelta_t sbusFrameTime = cmpTimeUs(now, sbusFrameStartAt);

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFramePosition = 0;
    }

    if (sbusFramePosition == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameStartAt = now;
    }

    if (sbusFramePosition < SBUS_FRAME_SIZE) {
        sbusFrame.bytes[sbusFramePosition++] = (uint8_t)c;
        if (sbusFramePosition == SBUS_FRAME_SIZE) {
            // endByte currently ignored
            sbusFrameDone = true;
#ifdef DEBUG_SBUS_PACKETS
            debug[2] = sbusFrameTime;
#endif
        } else {
<<<<<<<
            sbusFrameDone = false;
=======
            sbusFrameDone = true;
            DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
>>>>>>>
        }
    }
}

uint8_t sbusFrameStatus(void)
{
    if (!sbusFrameDone) {
        return RX_FRAME_PENDING;
    }
    sbusFrameDone = false;

    sbusStateFlags = 0;
    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS, sbusFrame.frame.channels.flags);

    if (sbusFrame.frame.channels.flags & SBUS_FLAG_SIGNAL_LOSS) {
        sbusStateFlags |= SBUS_STATE_SIGNALLOSS;
        DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_STATE_FLAGS, sbusStateFlags);
    }
    if (sbusFrame.frame.channels.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        sbusStateFlags |= SBUS_STATE_FAILSAFE;
        DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_STATE_FLAGS, sbusStateFlags);
    }

    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_STATE_FLAGS, sbusStateFlags);

<<<<<<<
static uint16_t sbusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
    return (0.625f * sbusChannelData[chan]) + 880;
=======
    return sbusChannelsDecode(&sbusFrame.frame.channels);
>>>>>>>
}

bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    sbusChannelsInit(rxConfig);

    rxRuntimeConfig->channelCount = SBUS_MAX_CHANNEL;
    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcReadRawFn = sbusChannelsReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = sbusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *sBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sbusDataReceive,
        NULL,
        SBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        SBUS_PORT_OPTIONS | (rxConfig->sbus_inversion ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = sBusPort;
    }
#endif

    return sBusPort != NULL;
}
#endif // USE_SERIALRX_SBUS
