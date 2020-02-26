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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_FPORT2

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#include "telemetry/smartport_master.h"
#endif

#include "rx/rx.h"
#include "rx/sbus_channels.h"
#include "rx/fport.h"


#define FPORT_TIME_NEEDED_PER_FRAME_US 3000
#define FPORT_MAX_TELEMETRY_RESPONSE_DELAY_US 1000
#define FPORT_MIN_TELEMETRY_RESPONSE_DELAY_US 500
#define FPORT_MAX_TELEMETRY_AGE_MS 500

#define FPORT_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES 2


#define FPORT_FRAME_MARKER 0x7E
#define FPORT_FC_REP_ID 0xB7

#define FPORT_ESCAPE_CHAR 0x7D
#define FPORT_ESCAPE_MASK 0x20

#define FPORT_CRC_VALUE 0xFF

#define FPORT_BAUDRATE 115200

#define FPORT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)

enum {
    DEBUG_FPORT_FRAME_INTERVAL = 0,
    DEBUG_FPORT_FRAME_ERRORS,
    DEBUG_FPORT_FRAME_LAST_ERROR,
    DEBUG_FPORT_TELEMETRY_INTERVAL,
};

enum {
    DEBUG_FPORT_NO_ERROR = 0,
    DEBUG_FPORT_ERROR_TIMEOUT,
    DEBUG_FPORT_ERROR_OVERSIZE,
    DEBUG_FPORT_ERROR_SIZE,
    DEBUG_FPORT_ERROR_CHECKSUM,
    DEBUG_FPORT_ERROR_TYPE,
    DEBUG_FPORT_ERROR_TYPE_SIZE,
};

enum {
    FPORT_FRAME_TYPE_CONTROL = 0xFF,
    FPORT_FRAME_TYPE_TELEMETRY_REQUEST = 0x01,
    FPORT_FRAME_TYPE_TELEMETRY_RESPONSE = 0x81,

};

enum {
    FPORT_FRAME_ID_NULL = 0x00,     // (master/slave)
    FPORT_FRAME_ID_DATA = 0x10,     // (master/slave)
    FPORT_FRAME_ID_READ = 0x30,     // (master)
    FPORT_FRAME_ID_WRITE = 0x31,    // (master)
    FPORT_FRAME_ID_RESPONSE = 0x32, // (slave)
};

typedef struct fportControlData_s {
    sbusChannels_t channels;
    uint8_t rssi;
} fportControlData_t;

typedef union fportData_s {
    fportControlData_t controlData;
    smartPortPayload_t telemetryData;
} fportData_t;

typedef struct fportFrame_s {
    uint8_t type;
    fportData_t data;
} fportFrame_t;

#ifdef USE_TELEMETRY_SMARTPORT
//static const smartPortPayload_t emptySmartPortFrame = { .frameId = 0, .valueId = 0, .data = 0 };
#endif

#define FPORT_REQUEST_FRAME_LENGTH sizeof(fportFrame_t)
#define FPORT_RESPONSE_FRAME_LENGTH (sizeof(uint8_t) + sizeof(smartPortPayload_t))

#define FPORT_FRAME_PAYLOAD_LENGTH_CONTROL (sizeof(fportControlData_t))
#define FPORT_FRAME_PAYLOAD_LENGTH_TELEMETRY_REQUEST (sizeof(uint8_t) + sizeof(smartPortPayload_t))

#define NUM_RX_BUFFERS 6
#define BUFFER_SIZE (FPORT_REQUEST_FRAME_LENGTH + 2 * sizeof(uint8_t))

typedef struct fportBuffer_s {
    uint8_t data[BUFFER_SIZE];
    uint8_t length;
} fportBuffer_t;

static fportBuffer_t rxBuffer[NUM_RX_BUFFERS];

static volatile uint8_t rxBufferWriteIndex = 0;
static volatile uint8_t rxBufferReadIndex = 0;

static volatile timeUs_t lastTelemetryFrameReceivedUs;
static volatile bool clearToSend = false;
static volatile bool sendZeroFrame = false;

static volatile uint8_t framePosition = 0;
static volatile uint8_t telemFramPosition = 0;

static smartPortPayload_t *mspPayload = NULL;
static timeUs_t lastRcFrameReceivedMs = 0;

static serialPort_t *fportPort;
#ifdef USE_TELEMETRY_SMARTPORT
static bool telemetryEnabled = false;
#endif

static void reportFrameError(uint8_t errorReason) {
    static volatile uint16_t frameErrors = 0;

    frameErrors++;

    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT_FRAME_ERRORS, frameErrors);
    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT_FRAME_LAST_ERROR, errorReason);
}

void receiveFrSkyByte(uint16_t c)
{
    static volatile uint8_t rxStreamCnt = 0;
    static volatile uint8_t telemStreamCnt = 0;
    static volatile bool escapeNext = false;
    static volatile timeUs_t lastRxByteTimestamp = 0;
    const timeUs_t currentTimeUs = micros();
    const timeUs_t timeSincePreviousRxByte = lastRxByteTimestamp ? currentTimeUs - lastRxByteTimestamp : 0;
    lastRxByteTimestamp = currentTimeUs;

    uint8_t val = (uint8_t)c;

    if ((timeSincePreviousRxByte > FPORT_TIME_NEEDED_PER_FRAME_US + 500) && (val == 0x18)) { // must be new packet, control packet starts with 0x18
        rxStreamCnt = 0;
        telemStreamCnt = 0;
        rxBuffer[rxBufferWriteIndex].data[rxStreamCnt++] = val;
        rxBuffer[rxBufferWriteIndex].length = 1;
        escapeNext = false;
    } else if ((rxStreamCnt < 27) && (timeSincePreviousRxByte < 120)) { // valid time for next byte, not a new packet, in rx stream
        if (escapeNext) {
            val = val ^ FPORT_ESCAPE_MASK;
            escapeNext = false;
        } else if (val == FPORT_ESCAPE_CHAR) {
            escapeNext = true;
            return;
        }
        rxBuffer[rxBufferWriteIndex].data[rxStreamCnt++] = val; // add to rx
        if (rxStreamCnt == 27) {
            rxBuffer[rxBufferWriteIndex].length = 27;
            const uint8_t nextWriteIndex = (rxBufferWriteIndex + 1) % NUM_RX_BUFFERS;
            if (nextWriteIndex != rxBufferReadIndex) {
                rxBufferWriteIndex = nextWriteIndex;
            }
        }
    } else if ((rxStreamCnt == 27) && (val == 0x08) && (timeSincePreviousRxByte < 120)) { // valid time for next byte, in a new packet, in telem stream
        rxStreamCnt++;
        telemStreamCnt = 0;
        rxBuffer[rxBufferWriteIndex].data[telemStreamCnt++] = val; // add to rx
        rxBuffer[rxBufferWriteIndex].length = 1;
        escapeNext = false;
    } else if ((rxStreamCnt > 27) && (telemStreamCnt < 11) && (timeSincePreviousRxByte < 120)) { // valid time for next byte, in a new packet, in telem stream
        rxStreamCnt++;
        if (escapeNext) {
            val = val ^ FPORT_ESCAPE_MASK;
            escapeNext = false;
        } else if (val == FPORT_ESCAPE_CHAR) {
            escapeNext = true;
            return;
        }
        rxBuffer[rxBufferWriteIndex].data[telemStreamCnt++] = val; // add to rx
        if (telemStreamCnt == 10)
        {
            rxBuffer[rxBufferWriteIndex].length = 10;
            const uint8_t nextWriteIndex = (rxBufferWriteIndex + 1) % NUM_RX_BUFFERS;
            if (nextWriteIndex != rxBufferReadIndex) {
                rxBufferWriteIndex = nextWriteIndex;
            }
        }
    }
}

// Receive ISR callback
static void fportDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint8_t val = (uint8_t)c;

    clearToSend = false;
    sendZeroFrame = false;

    receiveFrSkyByte(val);
}

#if defined(USE_TELEMETRY_SMARTPORT)
static void smartPortWriteFrameFport(const smartPortPayload_t *payload)
{
    UNUSED(payload);
#if defined(USE_TELEMETRY_SMARTPORT_MASTER)
    if(sendZeroFrame) {
        writeFportToSerial(fportPort, NULL);
        sendZeroFrame = false;
    } else {
        writeFportToSerial(fportPort, payload);
    }
#endif
}
#endif

static bool checkChecksum(uint8_t *data, uint8_t length)
{
    uint16_t checksum = 0;
    for(int i = 0; i < length; ++i) {
        checksum += *data++;
        checksum += (checksum >> 8);
        checksum &= 0xFF;
    }

    return checksum == 0xFF;
}

static uint8_t fportFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    static bool hasTelemetryRequest = false;

#ifdef USE_TELEMETRY_SMARTPORT
    static smartPortPayload_t payloadBuffer;
    static bool rxDrivenFrameRate = false;
    static uint8_t consecutiveTelemetryFrameCount = 0;
#endif

    uint8_t result = RX_FRAME_PENDING;


    if (rxBufferReadIndex != rxBufferWriteIndex) {
        uint8_t bufferLength = rxBuffer[rxBufferReadIndex].length;
        uint8_t frameLength = rxBuffer[rxBufferReadIndex].data[0];
        if (frameLength == 24 && frameLength != bufferLength - 3) {
            reportFrameError(DEBUG_FPORT_ERROR_SIZE);
        } else if ((frameLength == 8) && (frameLength != bufferLength - 2)) {
            reportFrameError(DEBUG_FPORT_ERROR_SIZE);
        } else {
            if ((frameLength == 24) && !checkChecksum(&rxBuffer[rxBufferReadIndex].data[1], bufferLength-1)) {
                reportFrameError(DEBUG_FPORT_ERROR_CHECKSUM);
            } else if ((frameLength == 8) && !checkChecksum(&rxBuffer[rxBufferReadIndex].data[1], bufferLength-1)) {
                reportFrameError(DEBUG_FPORT_ERROR_CHECKSUM);
            } else {
                fportFrame_t *frame = (fportFrame_t *)&rxBuffer[rxBufferReadIndex].data[1];

#if defined(USE_TELEMETRY_SMARTPORT) && defined(USE_TELEMETRY_SMARTPORT_MASTER)
                if (frameLength == 0x08) { // Downlink Data Frame
                    if (rxBuffer[rxBufferReadIndex].data[1] == FPORT_FC_REP_ID) {

                        if ((rxBuffer[rxBufferReadIndex].data[2] == FPORT_FRAME_ID_DATA) || (rxBuffer[rxBufferReadIndex].data[2] == FPORT_FRAME_ID_NULL)) {
                            clearToSend = true;
                            hasTelemetryRequest = true;
                            lastTelemetryFrameReceivedUs = micros();
                        }

                        if (rxBuffer[rxBufferReadIndex].data[2] == FPORT_FRAME_ID_DATA) {
                            sendZeroFrame = true;
                        }
                    }
                }
#endif

                switch (frame->type) {
                    case FPORT_FRAME_TYPE_CONTROL:
                        if (frameLength != FPORT_FRAME_PAYLOAD_LENGTH_CONTROL) {
                            reportFrameError(DEBUG_FPORT_ERROR_TYPE_SIZE);
                        } else {
                            result = sbusChannelsDecode(rxRuntimeConfig, &frame->data.controlData.channels);
                            lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(frame->data.controlData.rssi, 0, 100, 0, RSSI_MAX_VALUE));
                            lastRcFrameReceivedMs = millis();
                        }
                        break;

                    case FPORT_FRAME_TYPE_TELEMETRY_REQUEST:
                        if (frameLength != FPORT_FRAME_PAYLOAD_LENGTH_TELEMETRY_REQUEST) {
                            reportFrameError(DEBUG_FPORT_ERROR_TYPE_SIZE);
                        } else {
#if defined(USE_TELEMETRY_SMARTPORT)
                            if (!telemetryEnabled) {
                                break;
                            }

                            switch(frame->data.telemetryData.frameId) {
                                case FPORT_FRAME_ID_DATA:
                                    if (!rxDrivenFrameRate) {
                                        rxDrivenFrameRate = true;
                                    }

                                    hasTelemetryRequest = true;
                                    break;

                                case FPORT_FRAME_ID_NULL:
                                    if (!rxDrivenFrameRate) {
                                        if (consecutiveTelemetryFrameCount >= FPORT_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES && !(mspPayload && smartPortPayloadContainsMSP(mspPayload))) {
                                            consecutiveTelemetryFrameCount = 0;
                                        } else {
                                            hasTelemetryRequest = true;

                                            consecutiveTelemetryFrameCount++;
                                        }
                                    }
                                    break;

                                case FPORT_FRAME_ID_READ:
                                case FPORT_FRAME_ID_WRITE: // never used
                                    memcpy(&payloadBuffer, &frame->data.telemetryData, sizeof(smartPortPayload_t));
                                    mspPayload = &payloadBuffer;
                                    break;

                                default:
                                    break;
                            }
#endif /* USE_TELEMETRY_SMARTPORT */
                        }

                        break;

                    default:
                        reportFrameError(DEBUG_FPORT_ERROR_TYPE);
                        break;
                }
            }

        }

        rxBufferReadIndex = (rxBufferReadIndex + 1) % NUM_RX_BUFFERS;
    }

    if ((mspPayload || hasTelemetryRequest) && cmpTimeUs(micros(), lastTelemetryFrameReceivedUs) >= FPORT_MIN_TELEMETRY_RESPONSE_DELAY_US) {
        hasTelemetryRequest = false;

        result = (result & ~RX_FRAME_PENDING) | RX_FRAME_PROCESSING_REQUIRED;
    }

    if (lastRcFrameReceivedMs && ((millis() - lastRcFrameReceivedMs) > FPORT_MAX_TELEMETRY_AGE_MS)) {
        lqTrackerSet(rxRuntimeConfig->lqTracker, 0);
        lastRcFrameReceivedMs = 0;
    }

    return result;
}

static bool fportProcessFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

#if defined(USE_TELEMETRY_SMARTPORT)
    static timeUs_t lastTelemetryFrameSentUs;

    timeUs_t currentTimeUs = micros();
    if (cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) > FPORT_MAX_TELEMETRY_RESPONSE_DELAY_US) {
       clearToSend = false;
       sendZeroFrame = false;
    }

    if (clearToSend) {
        if(!sendZeroFrame)
        {
            smartPortWriteFrameFport(NULL); //send 0x00 reply
            clearToSend = false;
        }
        else
        {
            processSmartPortTelemetry(mspPayload, &clearToSend, NULL); //send data
        }

        DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT_TELEMETRY_INTERVAL, currentTimeUs - lastTelemetryFrameSentUs);
        lastTelemetryFrameSentUs = currentTimeUs;
    }

    mspPayload = NULL;
#endif

    return true;
}

bool fport2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];
    rxRuntimeConfig->channelData = sbusChannelData;
    sbusChannelsInit(rxRuntimeConfig);

    rxRuntimeConfig->channelCount = SBUS_MAX_CHANNEL;
    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcFrameStatusFn = fportFrameStatus;
    rxRuntimeConfig->rcProcessFrameFn = fportProcessFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    fportPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        fportDataReceive,
        NULL,
        FPORT_BAUDRATE,
        MODE_RXTX,
        FPORT_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
    );

    if (fportPort) {
#if defined(USE_TELEMETRY_SMARTPORT)
        telemetryEnabled = initSmartPortTelemetryExternal(smartPortWriteFrameFport);
#endif

    }

    return fportPort != NULL;
}
#endif
