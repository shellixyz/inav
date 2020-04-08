/*
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdbool.h>
#include <stdint.h>
/*#include <stdlib.h>*/
/*#include <string.h>*/
#include <math.h>

#include "platform.h"
/*FILE_COMPILE_FOR_SPEED*/

#if defined(USE_SMARTPORT_MASTER)

/*#include "common/axis.h"*/
/*#include "common/color.h"*/
/*#include "common/maths.h"*/
#include "build/debug.h"
#include "common/log.h"
#include "common/utils.h"
#include "common/bitarray.h"

/*#include "config/feature.h"*/
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/smartport_master.h"

#include "telemetry/smartport.h"
#include "telemetry/telemetry.h"

enum {
    PRIM_DISCARD_FRAME = 0x00,
    PRIM_DATA_FRAME = 0x10,
    PRIM_WORKING_STATE = 0x20,
    PRIM_IDLE_STATE = 0x21,
    PRIM_CONFIG_READ = 0x30,
    PRIM_CONFIG_WRITE = 0x31,
    PRIM_CONFIG_RESPONSE = 0x32,
    PRIM_DIAG_READ = 0X40,
    PRIM_DIAG_WRITE = 0X41,
    PRIM_ENABLE_APP_NODE = 0x70,
    PRIM_DISABLE_APP_NODE = 0x71,
};

enum
{
    DATAID_SPEED      = 0x0830 ,
    DATAID_VFAS       = 0x0210 ,
    DATAID_CURRENT    = 0x0200 ,
    DATAID_RPM        = 0x050F ,
    DATAID_ALTITUDE   = 0x0100 ,
    DATAID_FUEL       = 0x0600 ,
    DATAID_ADC1       = 0xF102 ,
    DATAID_ADC2       = 0xF103 ,
    DATAID_LATLONG    = 0x0800 ,
    DATAID_CAP_USED   = 0x0600 ,
    DATAID_VARIO      = 0x0110 ,
    DATAID_CELLS      = 0x0300 ,
    DATAID_CELLS_LAST = 0x030F ,
    DATAID_HEADING    = 0x0840 ,
    DATAID_FPV        = 0x0450 ,
    DATAID_PITCH      = 0x0430 ,
    DATAID_ROLL       = 0x0440 ,
    DATAID_ACCX       = 0x0700 ,
    DATAID_ACCY       = 0x0710 ,
    DATAID_ACCZ       = 0x0720 ,
    DATAID_T1         = 0x0400 ,
    DATAID_T2         = 0x0410 ,
    DATAID_HOME_DIST  = 0x0420 ,
    DATAID_GPS_ALT    = 0x0820 ,
    DATAID_ASPD       = 0x0A00 ,
    DATAID_A3         = 0x0900 ,
    DATAID_A4         = 0x0910
};

#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX

#define SMARTPORT_PHYID_MAX 0x1B
#define SMARTPORT_PHYID_COUNT (SMARTPORT_PHYID_MAX + 1)

#define SMARTPORT_POLLING_INTERVAL 12 // ms
/*#define SMARTPORT_POLLING_INTERVAL 200 // ms*/

#define SMARTPORT_FRAME_START 0x7E

static serialPort_t *smartportMasterSerialPort = NULL;
static serialPortConfig_t *portConfig;
static int8_t currentPolledPhyID = -1;
static uint8_t rxBufferLen = 0;

static BITARRAY_DECLARE(activePhyIDs, SMARTPORT_PHYID_COUNT);

typedef struct {
    int8_t cellCount;
    int16_t cellVoltage[6];
} cellsData_t;

typedef struct {
    cellsData_t cellsData;
} smartportSensorsData_t;

smartportSensorsData_t smartportSensorsData = { .cellsData.cellCount = -1 };

#pragma GCC push_options
#pragma GCC optimize ("O0")
bool smartportMasterInit(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT_MASTER);
    if (!portConfig) {
        return false;
    }

    portOptions_t portOptions = (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);
    smartportMasterSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT_MASTER, NULL, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);

    BITARRAY_CLR_ALL(activePhyIDs);

    return true;
}

static void smartportMasterSendByte(uint8_t byte)
{
    serialWrite(smartportMasterSerialPort, byte);
}

typedef enum {
    PT_ACTIVE_ID,
    PT_INACTIVE_ID
} pollType_e;

void smartportMasterPoll(void)
{
    static pollType_e nextPollType = PT_INACTIVE_ID;
    static uint8_t currentActivePhyID = 0, currentInactivePhyID = 0;
    uint8_t phyIDToPoll;

    switch (nextPollType) {

        case PT_ACTIVE_ID: {
            int8_t nextActivePhyID = BITARRAY_FIND_FIRST_SET(activePhyIDs, currentActivePhyID); // find next active PhyID
            if (nextActivePhyID == -1) {                                // no active PhyID found
                currentActivePhyID = 0;                                 // back to first valid PhyID
                nextPollType = PT_INACTIVE_ID;                          // continue by polling inactive PhyID
            } else {
                phyIDToPoll = nextActivePhyID;
                if (currentActivePhyID == SMARTPORT_PHYID_MAX) {        // last PhyID
                    currentActivePhyID = 0;                             // back to first valid PhyID
                } else {
                    currentActivePhyID += 1;                            // prepare next round
                }
                break;
            }
            FALLTHROUGH;
        }

        case PT_INACTIVE_ID: {
            phyIDToPoll = currentInactivePhyID;
            if (currentInactivePhyID == SMARTPORT_PHYID_MAX) {          // last PhyID
                currentInactivePhyID = 0;                               // back to first valid PhyID
            } else {
                currentInactivePhyID += 1;                              // prepare next round
            }
            nextPollType = PT_ACTIVE_ID;                                // next poll active PhyID
            break;
        }

    }

    /*phyIDToPoll = 1;*/

    currentPolledPhyID = phyIDToPoll;

    // construct check bits
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 1) ^ GET_BIT(phyIDToPoll, 2)) << 5;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 3) ^ GET_BIT(phyIDToPoll, 4)) << 6;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 4)) << 7;

    /*LOG_E(SYSTEM, "Polling %X (%X)", currentPolledPhyID, phyIDToPoll);*/
    /*LOG_E(SYSTEM, "Polling %X (%X) -- rxBufferLen = %d", currentPolledPhyID, phyIDToPoll, rxBufferLen);*/

    // poll
    smartportMasterSendByte(SMARTPORT_FRAME_START);
    smartportMasterSendByte(phyIDToPoll);

    rxBufferLen = 0; // discard data received during previous poll
}

typedef struct smartPortMasterFrame_s {
    uint8_t magic;
    uint8_t PhyID;
    smartPortPayload_t payload;
} PACKED smartportFrame_t;

typedef union {
    smartportFrame_t frame;
    uint8_t bytes[sizeof(smartportFrame_t)];
} smartportPayloadBuffer_u;

static uint8_t calculatePayloadCRC(smartPortPayload_t* payload)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < sizeof(*payload); ++i) {
        sum += ((uint8_t *)payload)[i];
    }
    return 0xFF - ((sum & 0xFF) + (sum >> 8));
}

static void decode_cells_data(uint32_t sdata)
{
    uint8_t voltageStartIndex = sdata & 0xF;
    uint8_t cellCount = sdata >> 4 & 0xF;
    uint16_t voltage1 = (sdata >> 8 & 0xFFF) * 2;
    uint16_t voltage2 = (sdata >> 20 & 0xFFF) * 2;
    if ((voltageStartIndex <= 4) && (cellCount <= 6)) {
        cellsData_t *cd = &smartportSensorsData.cellsData;
        cd->cellCount = cellCount;
        cd->cellVoltage[voltageStartIndex] = voltage1;
        cd->cellVoltage[voltageStartIndex+1] = voltage2;
        debug[0] = cd->cellCount;
        for (uint8_t i = 0; i < 6; ++i)
            debug[i+1] = cd->cellVoltage[i];
    }
    /*LOG_E(SYSTEM, "i: %d, S: %d, v1: %d, v2: %d", voltageStartIndex, cellCount, voltage1, voltage2);*/
}

static void processPayload(smartPortPayload_t *payload)
{
    if (payload->frameId == PRIM_DATA_FRAME) {
        switch (payload->valueId) {
            case DATAID_CELLS:
                decode_cells_data(payload->data);
                break;
        }
    }
}

static void smartportMasterReceive(void)
{
    static smartportPayloadBuffer_u buffer;
    while (serialRxBytesWaiting(smartportMasterSerialPort)) {
        if (rxBufferLen < sizeof(buffer)) {
            uint8_t c = serialRead(smartportMasterSerialPort);
            buffer.bytes[rxBufferLen] = c;
            /*LOG_E(SYSTEM, "rxBufferLen = %d, data = %x", rxBufferLen, c);*/
            if (currentPolledPhyID > -1) { // If we polled a PhyID
                rxBufferLen += 1;
            }
        } else {
            // payload complete, check crc and process payload
            uint8_t crc = serialRead(smartportMasterSerialPort);
            uint8_t calc = calculatePayloadCRC(&buffer.frame.payload);
            /*LOG_E(SYSTEM, "crc = %d, calc = %d", crc, calc);*/
            /*if (crc == calculatePayloadCRC(&buffer.frame.payload)) {*/
            if (crc == calc) {
                bitArraySet(activePhyIDs, currentPolledPhyID);
                processPayload(&buffer.frame.payload);
                rxBufferLen = 0; // reset buffer
                currentPolledPhyID = -1; // previously polled PhyID has answered, no expecting more data until next poll
            }
        }

    }
}

void smartportMasterHandle(timeUs_t currentTimeUs)
{
    static timeUs_t pollTimestamp = 0;

    if (!smartportMasterSerialPort) {
        return;
    }

    if (!pollTimestamp || (cmpTimeUs(currentTimeUs, pollTimestamp) > SMARTPORT_POLLING_INTERVAL * 1000)) {
        smartportMasterPoll();
        pollTimestamp = currentTimeUs;
    } else {
        smartportMasterReceive();
    }
}
#pragma GCC pop_options

#endif
