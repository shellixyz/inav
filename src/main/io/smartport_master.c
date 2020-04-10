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
#define SMARTPORT_BYTESTUFFING_MARKER 0x7D
#define SMARTPORT_BYTESTUFFING_XOR_VALUE 0x20

typedef struct smartPortMasterFrame_s {
    uint8_t magic;
    uint8_t PhyID;
    smartPortPayload_t payload;
} PACKED smartportFrame_t;

typedef union {
    smartportFrame_t frame;
    uint8_t bytes[sizeof(smartportFrame_t)];
} smartportPayloadBuffer_u;

typedef struct {
    int8_t cellCount;
    int16_t cellVoltage[6];
} cellsData_t;

typedef struct {
    cellsData_t cellsData;
} smartportSensorsData_t;


static serialPort_t *smartportMasterSerialPort = NULL;
static serialPortConfig_t *portConfig;
static int8_t currentPolledPhyID = -1;
static uint8_t rxBufferLen = 0;

static uint32_t activePhyIDs = 0;

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

    return true;
}

static void phyIDSetActive(uint8_t phyID, bool active)
{
    uint32_t mask = 1 << phyID;
    if (active) {
        activePhyIDs |= mask;
    } else {
        activePhyIDs &= ~mask;
    }
}

static uint32_t phyIDAllActiveMask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < SMARTPORT_PHYID_COUNT; ++i) {
        mask |= 1 << i;
    }
    return mask;
}

static int8_t phyIDNext(uint8_t start, bool active)
{
    for (uint8_t i = start; i < start + SMARTPORT_PHYID_COUNT; ++i) {
        uint8_t phyID = i % SMARTPORT_PHYID_COUNT;
        uint32_t mask = 1 << phyID;
        uint32_t phyIDMasked = activePhyIDs & mask;
        if ((active && phyIDMasked) || !(active || phyIDMasked)) {
            return phyID;
        }
    }
    return -1;
}

static bool phyIDNoneActive(void)
{
    return activePhyIDs == 0;
}

static bool phyIDAllActive(void)
{
    static uint32_t allActiveMask = 0;

    if (!allActiveMask) {
        allActiveMask = phyIDAllActiveMask();
    }

    return !!((activePhyIDs & allActiveMask) == allActiveMask);
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
    static uint8_t nextActivePhyID = 0, nextInactivePhyID = 0;
    uint8_t phyIDToPoll;

    if (currentPolledPhyID != -1) {
        // currentPolledPhyID hasn't been reset by smartportMasterReceive so we didn't get valid data for this PhyID (inactive)
        phyIDSetActive(currentPolledPhyID, false);
    }

    if (phyIDNoneActive()) {
        nextPollType = PT_INACTIVE_ID;
    }

    if (phyIDAllActive()) {
        nextPollType = PT_ACTIVE_ID;
    }

    switch (nextPollType) {

        case PT_ACTIVE_ID: {
            phyIDToPoll = phyIDNext(nextActivePhyID, true);
            nextActivePhyID = (phyIDToPoll == SMARTPORT_PHYID_MAX ? 0 : phyIDToPoll + 1);
            nextPollType = PT_INACTIVE_ID;
            break;
        }

        case PT_INACTIVE_ID: {
            phyIDToPoll = phyIDNext(nextInactivePhyID, false);
            nextInactivePhyID = (phyIDToPoll == SMARTPORT_PHYID_MAX ? 0 : phyIDToPoll + 1);
            nextPollType = PT_ACTIVE_ID;
            break;
        }

    }


    currentPolledPhyID = phyIDToPoll;

    // construct check bits
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 1) ^ GET_BIT(phyIDToPoll, 2)) << 5;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 3) ^ GET_BIT(phyIDToPoll, 4)) << 6;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 4)) << 7;

    // poll
    smartportMasterSendByte(SMARTPORT_FRAME_START);
    smartportMasterSendByte(phyIDToPoll);

    rxBufferLen = 0; // discard incomplete frames received during previous poll
}

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
    static bool processByteStuffing = false;

    if (!rxBufferLen) {
        processByteStuffing = false;
    }

    while (serialRxBytesWaiting(smartportMasterSerialPort)) {

        uint8_t c = serialRead(smartportMasterSerialPort);

        if (currentPolledPhyID == -1) { // We were did not poll a sensor or a packet has already been received and processed
            continue;
        }

        if (processByteStuffing) {
            c ^= SMARTPORT_BYTESTUFFING_XOR_VALUE;
            processByteStuffing = false;
        } else if (c == SMARTPORT_BYTESTUFFING_MARKER) {
            processByteStuffing = true;
            continue;
        }

        buffer.bytes[rxBufferLen] = c;
        rxBufferLen += 1;

        if (rxBufferLen == sizeof(buffer)) {
            // payload complete, check crc, process payload if CRC is good, reset buffer
            uint8_t rxCRC = serialRead(smartportMasterSerialPort);
            uint8_t calcCRC = calculatePayloadCRC(&buffer.frame.payload);
            if (rxCRC == calcCRC) {
                phyIDSetActive(currentPolledPhyID, true);
                processPayload(&buffer.frame.payload);
            }
            currentPolledPhyID = -1; // previously polled PhyID has answered, not expecting more data until next poll
            rxBufferLen = 0; // reset buffer
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
