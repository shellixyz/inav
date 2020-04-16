/*
 * This file is part of iNav.
 *
 * iNav are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * iNav is distributed in the hope that it
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

#include "common/log.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"
#include "io/smartport_master.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#endif

#include "rx/rx.h"
#include "rx/sbus_channels.h"
#include "rx/fport2.h"


#define FPORT2_MAX_TELEMETRY_RESPONSE_DELAY_US 3000
#define FPORT2_MIN_TELEMETRY_RESPONSE_DELAY_US 500
#define FPORT2_MAX_TELEMETRY_AGE_MS 500
#define FPORT2_FC_COMMON_ID 0x1B
#define FPORT2_FC_MSP_ID 0x0D
#define FPORT2_CRC_VALUE 0xFF
#define FPORT2_BAUDRATE 115200
#define FPORT2_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define FPORT2_RX_TIMEOUT 120 // µs
#define FPORT2_CONTROL_FRAME_LENGTH 24
#define FPORT2_DOWNLINK_FRAME_LENGTH 8
#define FPORT2_UPLINK_FRAME_LENGTH 8
#define FPORT2_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES 2

enum {
    DEBUG_FPORT2_FRAME_INTERVAL = 0,
    DEBUG_FPORT2_FRAME_ERRORS,
    DEBUG_FPORT2_FRAME_LAST_ERROR,
    DEBUG_FPORT2_TELEMETRY_INTERVAL,
};

enum {
    DEBUG_FPORT2_NO_ERROR = 0,
    DEBUG_FPORT2_ERROR_TIMEOUT,
    DEBUG_FPORT2_ERROR_OVERSIZE,
    DEBUG_FPORT2_ERROR_SIZE,
    DEBUG_FPORT2_ERROR_CHECKSUM,
    DEBUG_FPORT2_ERROR_PHYID_CRC,
    DEBUG_FPORT2_ERROR_TYPE,
    DEBUG_FPORT2_ERROR_TYPE_SIZE,
};

typedef enum {
    FT_DOWNLINK = 0,
    FT_CONTROL = 0xFF
} frame_type;

typedef enum {
    FS_CONTROL_FRAME_START,
    FS_CONTROL_FRAME_TYPE,
    FS_CONTROL_FRAME_DATA,
    FS_DOWNLINK_FRAME_START,
    FS_DOWNLINK_FRAME_DATA
} frame_state_e;

enum {
    FPORT2_FRAME_ID_NULL = 0x00,     // (master/slave)
    FPORT2_FRAME_ID_DATA = 0x10,     // (master/slave)
    FPORT2_FRAME_ID_READ = 0x30,     // (master)
    FPORT2_FRAME_ID_WRITE = 0x31,    // (master)
    FPORT2_FRAME_ID_RESPONSE = 0x32, // (slave)
};

typedef struct {
    sbusChannels_t channels;
    uint8_t rssi;
} controlData_t;

typedef struct {
    uint8_t phyID;
    /*uint8_t phyID : 5;*/
    /*uint8_t phyXOR : 3;*/
    smartPortPayload_t telemetryData;
} __attribute__((__packed__)) downlinkData_t;

typedef union {
    controlData_t controlData;
    downlinkData_t downlinkData;
} fportData_t;

typedef struct {
    uint8_t type;
    fportData_t data;
} fportFrame_t;

// RX frames ring buffer
#define NUM_RX_BUFFERS 6
#define BUFFER_SIZE 27

typedef struct fportBuffer_s {
    uint8_t data[BUFFER_SIZE];
    uint8_t length;
} fportBuffer_t;

static fportBuffer_t rxBuffer[NUM_RX_BUFFERS];
static volatile uint8_t rxBufferWriteIndex = 0;
static volatile uint8_t rxBufferReadIndex = 0;
// ^^^^^^^^^^^^^^^^^^^^^

static smartPortPayload_t *mspPayload = NULL;

static serialPort_t *fportPort;

#ifdef USE_TELEMETRY_SMARTPORT
static bool telemetryEnabled = false;
static volatile timeUs_t lastTelemetryFrameReceivedUs;
static volatile bool clearToSend = false;
static volatile bool sendNullFrame = false;
static uint8_t downlinkPhyID;
static const smartPortPayload_t emptySmartPortFrame = { .frameId = 0, .valueId = 0, .data = 0 };
#endif

#pragma GCC push_options
#pragma GCC optimize ("O0")

static void reportFrameError(uint8_t errorReason) {
    static volatile uint16_t frameErrors = 0;

    frameErrors++;

    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT2_FRAME_ERRORS, frameErrors);
    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT2_FRAME_LAST_ERROR, errorReason);
}

static void clearWriteBuffer(void)
{
    rxBuffer[rxBufferWriteIndex].length = 0;
}

static bool nextWriteBuffer(void)
{
    const uint8_t nextWriteIndex = (rxBufferWriteIndex + 1) % NUM_RX_BUFFERS;
    if (nextWriteIndex != rxBufferReadIndex) {
        rxBufferWriteIndex = nextWriteIndex;
        clearWriteBuffer();
        return true;
    } else {
        clearWriteBuffer();
        return false;
    }
}

static uint8_t writeBuffer(uint8_t byte)
{
    uint8_t * const buffer = rxBuffer[rxBufferWriteIndex].data;
    uint8_t * const buflen = &rxBuffer[rxBufferWriteIndex].length;
    buffer[*buflen] = byte;
    *buflen += 1;
    return *buflen;
}

// UART RX ISR
static void fportDataReceive(uint16_t byte, void *callback_data)
{
    UNUSED(callback_data);

    static volatile frame_state_e state = FS_CONTROL_FRAME_START;
    static volatile timeUs_t lastRxByteTimestamp = 0;
    const timeUs_t currentTimeUs = micros();
    const timeUs_t timeSincePreviousRxByte = lastRxByteTimestamp ? currentTimeUs - lastRxByteTimestamp : 0;

    lastRxByteTimestamp = currentTimeUs;
    clearToSend = false;

    if ((state != FS_CONTROL_FRAME_START) && (timeSincePreviousRxByte > FPORT2_RX_TIMEOUT)) {
        clearWriteBuffer();
        state = FS_CONTROL_FRAME_START;
    }

    switch (state) {

        case FS_CONTROL_FRAME_START:
            if (byte == FPORT2_CONTROL_FRAME_LENGTH) {
                state = FS_CONTROL_FRAME_TYPE;
            }
            break;

        case FS_CONTROL_FRAME_TYPE:
            if (byte == FT_CONTROL) {
                writeBuffer(FT_CONTROL);
                state = FS_CONTROL_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_CONTROL_FRAME_DATA:
            if (writeBuffer(byte) > (FPORT2_CONTROL_FRAME_LENGTH + 1)) {
                nextWriteBuffer();
                state = FS_DOWNLINK_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_START:
            if (byte == FPORT2_DOWNLINK_FRAME_LENGTH) {
                writeBuffer(FT_DOWNLINK);
                state = FS_DOWNLINK_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_DATA:
            if (writeBuffer(byte) > (FPORT2_DOWNLINK_FRAME_LENGTH + 1)) {
                nextWriteBuffer();
                lastTelemetryFrameReceivedUs = currentTimeUs;
                state = FS_CONTROL_FRAME_START;
            }
            break;

        default:
            state = FS_CONTROL_FRAME_START;
            break;

    }
}

#if defined(USE_TELEMETRY_SMARTPORT)
static void writeUplinkFramePhyID(uint8_t phyID, const smartPortPayload_t *payload)
{
    serialWrite(fportPort, FPORT2_UPLINK_FRAME_LENGTH);
    serialWrite(fportPort, phyID);

    uint16_t checksum = phyID;
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); ++i, ++data) {
        serialWrite(fportPort, *data);
        checksum += *data;
    }
    checksum = 0xff - ((checksum & 0xff) + (checksum >> 8));
    serialWrite(fportPort, checksum);
}

static void writeUplinkFrame(const smartPortPayload_t *payload)
{
    writeUplinkFramePhyID(FPORT2_FC_COMMON_ID, payload);
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

    return checksum == FPORT2_CRC_VALUE;
}

static uint8_t frameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
#ifdef USE_TELEMETRY_SMARTPORT
    static smartPortPayload_t payloadBuffer;
    static bool hasTelemetryRequest = false;
#endif

    static timeMs_t frameReceivedTimestamp = 0;
    uint8_t result = RX_FRAME_PENDING;

    if (rxBufferReadIndex != rxBufferWriteIndex) {

        uint8_t *buffer = rxBuffer[rxBufferReadIndex].data;
        uint8_t buflen = rxBuffer[rxBufferReadIndex].length;

        if (!checkChecksum(buffer + 1, buflen - 1)) {
            reportFrameError(DEBUG_FPORT2_ERROR_CHECKSUM);
        } else {

            fportFrame_t *frame = (fportFrame_t *)buffer;

            switch (frame->type) {

                case FT_CONTROL:
                    result = sbusChannelsDecode(rxRuntimeConfig, &frame->data.controlData.channels);
                    lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(frame->data.controlData.rssi, 0, 100, 0, RSSI_MAX_VALUE));
                    frameReceivedTimestamp = millis();
                    result |= RX_FRAME_COMPLETE;
                    break;

                case FT_DOWNLINK:
#if defined(USE_TELEMETRY_SMARTPORT)
                    if (!telemetryEnabled) {
                        break;
                    }

                    downlinkPhyID = frame->data.downlinkData.phyID;

                    switch (frame->data.downlinkData.telemetryData.frameId) {

                        case FPORT2_FRAME_ID_NULL:
                            sendNullFrame = true;
                            break;

                        case FPORT2_FRAME_ID_READ:
                        case FPORT2_FRAME_ID_WRITE:
                            if (frame->data.downlinkData.phyID == FPORT2_FC_MSP_ID) {
                                memcpy(&payloadBuffer, &frame->data.downlinkData.telemetryData, sizeof(payloadBuffer));
                                mspPayload = &payloadBuffer;
                            } else {
#if defined(USE_SMARTPORT_MASTER)
                                smartportMasterForward(frame->data.downlinkData.phyID & 0xF, &frame->data.downlinkData.telemetryData);
#endif
                            }
                            FALLTHROUGH;

                        default:
                            break;

                    }

                    hasTelemetryRequest = true;
#endif
                    break;

                default:
                    reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                    break;

            }

        }

        rxBufferReadIndex = (rxBufferReadIndex + 1) % NUM_RX_BUFFERS;
    }

#if defined(USE_TELEMETRY_SMARTPORT)
    if ((mspPayload || hasTelemetryRequest) && cmpTimeUs(micros(), lastTelemetryFrameReceivedUs) >= FPORT2_MIN_TELEMETRY_RESPONSE_DELAY_US) {
        hasTelemetryRequest = false;
        clearToSend = true;
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }
#endif

    if (frameReceivedTimestamp && ((millis() - frameReceivedTimestamp) > FPORT2_MAX_TELEMETRY_AGE_MS)) {
        lqTrackerSet(rxRuntimeConfig->lqTracker, 0);
        frameReceivedTimestamp = 0;
    }

    return result;
}

static bool processFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

#if defined(USE_TELEMETRY_SMARTPORT)
    static timeUs_t lastTelemetryFrameSentUs;

    timeUs_t currentTimeUs = micros();
    if (cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) > FPORT2_MAX_TELEMETRY_RESPONSE_DELAY_US) {
       clearToSend = false;
    }

    if (clearToSend) {
        if ((downlinkPhyID == FPORT2_FC_COMMON_ID) || (downlinkPhyID == FPORT2_FC_MSP_ID)) {
            if ((downlinkPhyID == FPORT2_FC_MSP_ID) && !mspPayload) {
                clearToSend = false;
            } else if (!sendNullFrame) {
                processSmartPortTelemetry(mspPayload, &clearToSend, NULL);
                mspPayload = NULL;
            }
        } else {
#if defined(USE_SMARTPORT_MASTER)
            uint8_t smartportMasterPhyID = downlinkPhyID & 0x0F; // remove check bits as smartport master functions expect naked PhyIDs
            uint8_t phyIDCheck = smartportMasterPhyID;
            smartportMasterPhyIDFillCheckBits(&phyIDCheck);

            if (downlinkPhyID == phyIDCheck) {
                if (sendNullFrame) {
                    if (!smartportMasterPhyIDIsActive(smartportMasterPhyID)) { // send null frame only if the sensor is active
                        clearToSend = false;
                    }
                } else {
                    smartPortPayload_t forwardPayload;
                    if (smartportMasterNextForwardResponse(smartportMasterPhyID, &forwardPayload) || smartportMasterGetSensorPayload(smartportMasterPhyID, &forwardPayload)) {
                        writeUplinkFramePhyID(downlinkPhyID, &forwardPayload);
                    }
                    clearToSend = false; // either we answered or the sensor is not active, do not send null frame
                }
            } else {
                clearToSend = false;
            }
#else
            clearToSend = false;
#endif
        }

        if (clearToSend) {
            writeUplinkFramePhyID(downlinkPhyID, &emptySmartPortFrame);
            clearToSend = false;
        }

        sendNullFrame = false;

        DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT2_TELEMETRY_INTERVAL, currentTimeUs - lastTelemetryFrameSentUs);
        lastTelemetryFrameSentUs = currentTimeUs;

    }
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

    rxRuntimeConfig->rcFrameStatusFn = frameStatus;
    rxRuntimeConfig->rcProcessFrameFn = processFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    fportPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        fportDataReceive,
        NULL,
        FPORT2_BAUDRATE,
        MODE_RXTX,
        FPORT2_PORT_OPTIONS | (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
    );

    if (fportPort) {
#if defined(USE_TELEMETRY_SMARTPORT)
        telemetryEnabled = initSmartPortTelemetryExternal(writeUplinkFrame);
#endif

    }

    return fportPort != NULL;
}

#endif
