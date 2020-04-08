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
FILE_COMPILE_FOR_SPEED

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_SMARTPORT_MASTER)

/*#include "common/axis.h"*/
/*#include "common/color.h"*/
/*#include "common/maths.h"*/
/*#include "common/utils.h"*/
#include "common/bitarray.h"

/*#include "config/feature.h"*/
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "telemetry/telemetry.h"
#include "telemetry/smartport_master.h"

enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_RPM        = 0x050F ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_CAP_USED   = 0x0600 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
    FSSP_DATAID_FPV        = 0x0450 ,
    FSSP_DATAID_PITCH      = 0x0430 ,
    FSSP_DATAID_ROLL       = 0x0440 ,
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_HOME_DIST  = 0x0420 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_ASPD       = 0x0A00 ,
    FSSP_DATAID_A3         = 0x0900 ,
    FSSP_DATAID_A4         = 0x0910
};

#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX

#define SMARTPORT_PHYID_MAX 0x1B
#define SMARTPORT_PHYID_COUNT (SMARTPORT_PHYID_MAX + 1)

#define SMARTPORT_POLLING_INTERVAL 12000 // ms

#define SMARTPORT_FRAME_START 0x7E

static serialPort_t *smartportMasterSerialPort = NULL;
static serialPortConfig_t *portConfig;

static BITARRAY_DECLARE(activePhyIDs, SMARTPORT_PHYID_COUNT);

bool initSmartPortMasterTelemetry(void)
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

void smartPortMasterPoll(void)
{
    static pollType_e nextPollType = PT_INACTIVE_ID;
    static uint8_t currentActivePhyID = 0, currentInactivePhyID = 0;
    uint8_t phyIDToPoll;

    switch (nextPollType) {

        case PT_ACTIVE_ID: {
            int8_t nextActivePhyID = BITARRAY_FIND_FIRST_SET(activePhyIDs, currentActivePhyID); // find next active PhyID
            if (nextActivePhyID == -1) {                             // no active PhyID found
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
            if (currentInactivePhyID == SMARTPORT_PHYID_MAX) {        // last PhyID
                currentInactivePhyID = 0;                             // back to first valid PhyID
            } else {
                currentInactivePhyID += 1;                            // prepare next round
            }
            break;
        }

    }

    // construct check bits
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 1) ^ GET_BIT(phyIDToPoll, 2)) << 5;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 3) ^ GET_BIT(phyIDToPoll, 4)) << 6;
    phyIDToPoll |= (GET_BIT(phyIDToPoll, 0) ^ GET_BIT(phyIDToPoll, 2) ^ GET_BIT(phyIDToPoll, 4)) << 7;

    // poll
    smartportMasterSendByte(SMARTPORT_FRAME_START);
    smartportMasterSendByte(phyIDToPoll);
}

void smartPortMasterReceive(void)
{
}

void handleSmartPortMasterTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t pollTimestamp = 0;

    if (!smartportMasterSerialPort) {
        return;
    }

    if (!pollTimestamp || (cmpTimeUs(currentTimeUs, pollTimestamp) > SMARTPORT_POLLING_INTERVAL)) {
        smartPortMasterPoll();
        pollTimestamp = currentTimeUs;
    } else {
        smartPortMasterReceive();
    }
}

#endif
