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
    DEBUG_FPORT2_ERROR_OTA_BAD_ADDRESS,
};

typedef enum {
    CFT_RC = 0xFF,
    CFT_OTA_START = 0xF0,
    CFT_OTA_DATA = 0xF1,
    CFT_OTA_STOP = 0xF2
} fport2_control_frame_type_e;

typedef enum {
    FT_CONTROL,
    FT_DOWNLINK
} frame_type_e;

typedef enum {
    FS_CONTROL_FRAME_START,
    FS_CONTROL_FRAME_TYPE,
    FS_CONTROL_FRAME_DATA,
    FS_DOWNLINK_FRAME_START,
    FS_DOWNLINK_FRAME_DATA
} frame_state_e;

enum {
    FPORT2_FRAME_ID_NULL = 0x00,
    FPORT2_FRAME_ID_DATA = 0x10,
    FPORT2_FRAME_ID_READ = 0x30,
    FPORT2_FRAME_ID_WRITE = 0x31,
    FPORT2_FRAME_ID_RESPONSE = 0x32,
    FPORT2_FRAME_ID_OTA_START = 0xF0,
    FPORT2_FRAME_ID_OTA_DATA = 0xF1,
    FPORT2_FRAME_ID_OTA_STOP = 0xF2
};

typedef struct {
    sbusChannels_t channels;
    uint8_t rssi;
} rcData_t;

typedef struct {
    uint8_t phyID;
    /*uint8_t phyID : 5;*/
    /*uint8_t phyXOR : 3;*/
    smartPortPayload_t telemetryData;
} PACKED fportDownlinkData_t;

typedef struct {
    uint8_t type;
    union {
        rcData_t rc;
        uint8_t ota[16];
    };
} PACKED fportControlFrame_t;

/*typedef union {*/
    /*rcData_t control;*/
    /*downlinkData_t downlink;*/
/*} fportData_u;*/

typedef struct {
    uint8_t type;
    /*fportData_u data;*/
    union {
        fportControlFrame_t control;
        fportDownlinkData_t downlink;
    };
} fportFrame_t;

// RX frames ring buffer
#define NUM_RX_BUFFERS 50
#define BUFFER_SIZE 30 // XXX

typedef struct fportBuffer_s {
    /*uint8_t data[sizeof(fportFrame_t)+1]; // +1 for CRC*/
    uint8_t data[BUFFER_SIZE]; // +1 for CRC
    uint8_t length;
} fportBuffer_t;

#pragma GCC push_options
#pragma GCC optimize ("O0")

static volatile fportBuffer_t rxBuffer[NUM_RX_BUFFERS];
static volatile uint8_t rxBufferWriteIndex = 0;
static volatile uint8_t rxBufferReadIndex = 0;

static serialPort_t *fportPort;

#ifdef USE_TELEMETRY_SMARTPORT
static smartPortPayload_t *mspPayload = NULL;
static bool telemetryEnabled = false;
static volatile timeUs_t lastTelemetryFrameReceivedUs;
static volatile bool clearToSend = false;
static volatile bool sendNullFrame = false;
static uint8_t downlinkPhyID;
static const smartPortPayload_t emptySmartPortFrame = { .frameId = 0, .valueId = 0, .data = 0 };
static smartPortPayload_t *otaResponsePayload = NULL;
static bool otaMode = false;
static timeUs_t otaFrameEndTimestamp = 0;
#endif

static volatile uint16_t frameErrors = 0;

static void reportFrameError(uint8_t errorReason) {

    frameErrors++;

    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT2_FRAME_ERRORS, frameErrors);
    DEBUG_SET(DEBUG_FPORT, DEBUG_FPORT2_FRAME_LAST_ERROR, errorReason);
}

static uint8_t bufferCount(void)
{
    if (rxBufferReadIndex > rxBufferWriteIndex) {
        return NUM_RX_BUFFERS - rxBufferReadIndex + rxBufferWriteIndex;
    } else {
        return rxBufferWriteIndex - rxBufferReadIndex;
    }
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

    /*debug[4] = MAX(bufferCount(), debug[4]);*/

    if ((state != FS_CONTROL_FRAME_START) && (timeSincePreviousRxByte > FPORT2_RX_TIMEOUT)) {
        clearWriteBuffer();
        state = FS_CONTROL_FRAME_START;
    }

    switch (state) {

        case FS_CONTROL_FRAME_START:
            if (byte == FPORT2_CONTROL_FRAME_LENGTH) {
                writeBuffer(FT_CONTROL);
                state = FS_CONTROL_FRAME_TYPE;
            }
            break;

        case FS_CONTROL_FRAME_TYPE:
            if ((byte == CFT_RC) || ((byte >= CFT_OTA_START) && (byte <= CFT_OTA_STOP))) {
                writeBuffer(byte);
                state = FS_CONTROL_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_CONTROL_FRAME_DATA:
            if (writeBuffer(byte) > (FPORT2_CONTROL_FRAME_LENGTH + 2)) { // +2 = General frame type + CRC 
                /*LOG_E(SYSTEM, "p%X", byte);*/
                nextWriteBuffer();
                state = FS_DOWNLINK_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_START:
            /*LOG_E(SYSTEM, "n%X", byte);*/
            if (byte == FPORT2_DOWNLINK_FRAME_LENGTH) {
                writeBuffer(FT_DOWNLINK);
                state = FS_DOWNLINK_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_DATA:
            if (writeBuffer(byte) > (FPORT2_DOWNLINK_FRAME_LENGTH + 1)) {
                /*if ((rxBuffer[rxBufferWriteIndex].data[2] >= FPORT2_FRAME_ID_OTA_START) && (rxBuffer[rxBufferWriteIndex].data[2] <= FPORT2_FRAME_ID_OTA_STOP)) {*/
                    /*otaFrameEndTimestamp = currentTimeUs;*/
                /*}*/
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
    static uint32_t otaPrevDataAddress;
#endif
    static bool otaJustStarted = false;
    static bool otaGotData = false;
    static uint8_t otaDataBuffer[16];
    static timeMs_t frameReceivedTimestamp = 0;
    uint8_t result = RX_FRAME_PENDING;

#if 1
    if (rxBufferReadIndex != rxBufferWriteIndex) {

        uint8_t *xbuffer = rxBuffer[rxBufferReadIndex].data;
        uint8_t xbuflen = rxBuffer[rxBufferReadIndex].length;

        fportFrame_t *frame = (fportFrame_t *)xbuffer;

#if 0
        if (!checkChecksum(buffer + 1, buflen - 1)) {
            reportFrameError(DEBUG_FPORT2_ERROR_CHECKSUM);
        } else {

            switch (frame->type) {

                case FT_CONTROL:
                    switch (frame->control.type) {

                        case CFT_RC:
                            result = sbusChannelsDecode(rxRuntimeConfig, &frame->control.rc.channels);
                            lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(frame->control.rc.rssi, 0, 100, 0, RSSI_MAX_VALUE));
                            frameReceivedTimestamp = millis();
                            break;

                        case CFT_OTA_START:
                            otaMode = true;
                            break;

                        case CFT_OTA_DATA:
                            if (otaMode) {
                                memcpy(otaDataBuffer, frame->control.ota, 16);
                                otaGotData = true;
                            } else {
                                reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                            }
                            break;

                        case CFT_OTA_STOP:
                            if (!otaMode) {
                                reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                            }
                            break;

                        default:
                            reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                            break;

                    }
                    break;

                case FT_DOWNLINK:
#if defined(USE_TELEMETRY_SMARTPORT)
                    if (!telemetryEnabled) {
                        break;
                    }

                    downlinkPhyID = frame->downlink.phyID;
                    uint8_t frameId = frame->downlink.telemetryData.frameId;

                    switch (frameId) {

                        case FPORT2_FRAME_ID_NULL:
                            hasTelemetryRequest = true;
                            sendNullFrame = true;
                            break;

                        case FPORT2_FRAME_ID_DATA:
                            hasTelemetryRequest = true;
                            break;

                        /*case FPORT2_FRAME_ID_OTA_START:*/
                            /*if (otaMode) {*/
                                /*otaJustStarted = true;*/
                                /*hasTelemetryRequest = true;*/
                            /*}*/
                            /*break;*/

                        case FPORT2_FRAME_ID_OTA_START:
                        case FPORT2_FRAME_ID_OTA_DATA:
                        case FPORT2_FRAME_ID_OTA_STOP:
                            switch (frameId) {
                                case FPORT2_FRAME_ID_OTA_START:
                                    if (otaMode) {
                                        otaJustStarted = true;
                                        otaPrevDataAddress = 0;
                                        hasTelemetryRequest = true;
                                    } else {
                                        reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                                    }
                                    break;

                                case FPORT2_FRAME_ID_OTA_DATA:
                                    if (otaMode) {
                                        uint32_t otaDataAddress = frame->downlink.telemetryData.data;
                                        if (otaGotData && (otaDataAddress == (otaJustStarted ? 0 : otaPrevDataAddress + 16))) { // check that we got a control frame with data and check address
                                            hasTelemetryRequest = true;
                                            otaPrevDataAddress = otaDataAddress;
                                            otaGotData = false;
                                            debug[5] += 16; // XXX do something with the data
                                        } else {
                                            reportFrameError(DEBUG_FPORT2_ERROR_OTA_BAD_ADDRESS);
                                        }
                                        otaJustStarted = false;
                                    } else {
                                        reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                                    }
                                    break;

                                case FPORT2_FRAME_ID_OTA_STOP:
                                    if (otaMode) {
                                        hasTelemetryRequest = true;
                                    } else {
                                        reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                                    }
                                    break;
                            }
                            if (hasTelemetryRequest) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                otaResponsePayload = &payloadBuffer;
                            }
                            break;

                        /*case FPORT2_FRAME_ID_OTA_STOP:*/
                            /*if (otaMode) {*/
                                /*hasTelemetryRequest = true;*/
                            /*}*/
                            /*break;*/

                        default:
                            if ((frameId == FPORT2_FRAME_ID_READ) && (downlinkPhyID == FPORT2_FC_MSP_ID)) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                mspPayload = &payloadBuffer;
                                hasTelemetryRequest = true;
                            } else if (downlinkPhyID != FPORT2_FC_COMMON_ID) {
#if defined(USE_SMARTPORT_MASTER)
                                int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);
                                if (smartportPhyID != -1) {
                                    smartportMasterForward(smartportPhyID, &frame->downlink.telemetryData);
                                    hasTelemetryRequest = true;
                                }
#endif
                            }
                            break;

                    }
#endif
                    break;

                default:
                    reportFrameError(DEBUG_FPORT2_ERROR_TYPE);
                    break;

            }

        }

        rxBufferReadIndex = (rxBufferReadIndex + 1) % NUM_RX_BUFFERS;
#endif
    }
#endif

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
        if (otaResponsePayload) {
            debug[6] = currentTimeUs - otaFrameEndTimestamp;
            writeUplinkFramePhyID(downlinkPhyID, otaResponsePayload);
            if (otaResponsePayload->frameId == FPORT2_FRAME_ID_OTA_STOP) {
                otaMode = false;
            }
            otaResponsePayload = NULL;
            clearToSend = false;

        } else if ((downlinkPhyID == FPORT2_FC_COMMON_ID) || (downlinkPhyID == FPORT2_FC_MSP_ID)) {
            if ((downlinkPhyID == FPORT2_FC_MSP_ID) && !mspPayload) {
                clearToSend = false;
            } else if (!sendNullFrame) {
                processSmartPortTelemetry(mspPayload, &clearToSend, NULL);
                mspPayload = NULL;
            }

        } else {
#if defined(USE_SMARTPORT_MASTER)
            int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);

            if (smartportPhyID != -1) {
                if (sendNullFrame) {
                    if (!smartportMasterPhyIDIsActive(smartportPhyID)) { // send null frame only if the sensor is active
                        clearToSend = false;
                    }
                } else {
                    smartPortPayload_t forwardPayload;
                    if (smartportMasterNextForwardResponse(smartportPhyID, &forwardPayload) || smartportMasterGetSensorPayload(smartportPhyID, &forwardPayload)) {
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
