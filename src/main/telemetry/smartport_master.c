#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_TELEMETRY_SMARTPORT_MASTER)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "fc/runtime_config.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/vtx_vs600.h"

#include "msp/msp.h"

#include "rx/rx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "telemetry/msp_shared.h"
#include "telemetry/smartport.h"
#include "telemetry/smartport_master.h"
#include "telemetry/telemetry.h"

typedef enum {
    FSSP_DATAID_ALTITUDE   = 0x0100,
    FSSP_DATAID_VARIO      = 0x0110,
    FSSP_DATAID_CURRENT    = 0x0200,
    FSSP_DATAID_VFAS       = 0x0210,
    FSSP_DATAID_CELLS      = 0x0300,
    FSSP_DATAID_T1         = 0x0400,
    FSSP_DATAID_T2         = 0x0410,
    FSSP_DATAID_HOME_DIST  = 0x0420,
    FSSP_DATAID_RPM        = 0x0500,
    FSSP_DATAID_FUEL       = 0x0600,
    FSSP_DATAID_ACCX       = 0x0700,
    FSSP_DATAID_ACCY       = 0x0710,
    FSSP_DATAID_ACCZ       = 0x0720,
    FSSP_DATAID_LATLONG    = 0x0800,
    FSSP_DATAID_GPS_ALT    = 0x0820,
    FSSP_DATAID_SPEED      = 0x0830,
    FSSP_DATAID_HEADING    = 0x0840,
    FSSP_DATAID_A3         = 0x0900,
    FSSP_DATAID_A4         = 0x0910,
    FSSP_DATAID_ASPD       = 0x0A00,
    FSSP_DATAID_TEMP       = 0x0B70,
    FSSP_DATAID_VS600      = 0x0E10,
    FSSP_DATAID_ADC1       = 0xF102,
    FSSP_DATAID_ADC2       = 0xF103,
} applicationId_e;

static portSharing_e smartPortPortSharing;

#define SMARTPORT_BAUD 57600
#define FPORT_BAUD 115200
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_US 1000 // max allowed time to find a value to send
#define SMARTPORT_TX_RATE_US 12000
#define FPORT_TX_RATE_US 5000
#define SMARTPORT_RX_WINDOW_US 1000

enum
{
    TELEMETRY_STATE_UNINITIALIZED,
    TELEMETRY_STATE_SEND,
    TELEMETRY_STATE_RECEIVE,
};

typedef enum {
    FRSKY_CMD_READ_NODE,
    FRSKY_CMD_STATE_TRANSFER,
    FRSKY_CMD_CONFIG_WITH_WRITE,
    FRSKY_CMD_CONFIG_WITH_READ,
    FRSKY_CMD_DIAG_WITH_READ,
    FRSKY_CMD_ENABLE_APP_NODE,
    FRSKY_CMD_DISABLE_APP_NODE,
} frskyCommand_e;

typedef enum {
    FRSKY_PRIM_DISCARD_FRAME = 0x00,
    FRSKY_PRIM_DATA_FRAME = 0x10,
    FRSKY_PRIM_WORKING_STATE = 0x20,
    FRSKY_PRIM_IDLE_STATE = 0x21,
    FRSKY_PRIM_CONFIG_READ = 0x30,
    FRSKY_PRIM_CONFIG_WRITE = 0x31,
    FRSKY_PRIM_CONFIG_RESPONSE = 0x32,
    FRSKY_PRIM_DIAG_READ = 0X40,
    FRSKY_PRIM_DIAG_WRITE = 0X41,
    FRSKY_PRIM_ENABLE_APP_NODE = 0x70,
    FRSKY_PRIM_DISABLE_APP_NODE = 0x71,
} frskyPrim_e;

typedef enum {
    FRSKY_PROTOCOL_SPORT,
    FRSKY_PROTOCOL_FPORT,
} frskyProtocol_e;

frskyProtocol_e currentProtocol = FRSKY_PROTOCOL_FPORT;

#define MAX_SMART_PORT_MASTER_SENSORS 0x1C

SmartPortMasterSensorData_t smartPortMasterSensorData = {0};

static uint8_t telemetryState = TELEMETRY_STATE_UNINITIALIZED;

static serialPort_t *smartPortMasterSerialPort = NULL; // The 'SmartPort'(tm) Port.
static serialPortConfig_t *portConfig;

uint8_t activePhysicalId[MAX_SMART_PORT_MASTER_SENSORS];
uint8_t activePhysicalIdCnt = 0;
uint8_t currentPhysicalId = 0;

uint16_t activeApplicationId[MAX_SMART_PORT_MASTER_SENSORS*2] = {0x00,};
uint16_t activeApplicationIdToPid[MAX_SMART_PORT_MASTER_SENSORS*2] = {0x00,};
uint8_t activeApplicationIdCnt = 0;

uint32_t sensorRawData[MAX_SMART_PORT_MASTER_SENSORS*2] = {0,};


bool primResponseReceived = false;
uint8_t primResponse[8] = {0};

SmartPortMasterSensorData_t* getSmartPortMasterSensorData(void)
{
    if (activePhysicalIdCnt)
        return &smartPortMasterSensorData;
    else
        return NULL;
}

static void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortMasterSerialPort);
    smartPortMasterSerialPort = NULL;
}

static void configureSmartPortTelemetryPort(void)
{
    if (portConfig) {
        if(currentProtocol == FRSKY_PROTOCOL_FPORT)
        {
            portOptions_e portOptions = (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
            smartPortMasterSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT_MASTER, NULL, NULL, FPORT_BAUD, SMARTPORT_UART_MODE, portOptions);
        }
        else
        {
            portOptions_e portOptions = (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (telemetryConfig()->telemetry_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);
            smartPortMasterSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT_MASTER, NULL, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);
        }
    }
}

void checkSmartPortMasterTelemetryState(void)
{
    if (telemetryState != TELEMETRY_STATE_UNINITIALIZED) {
        bool enableSerialTelemetry = telemetryDetermineEnabledState(smartPortPortSharing);

        if (enableSerialTelemetry && !smartPortMasterSerialPort) {
            configureSmartPortTelemetryPort();
        } else if (!enableSerialTelemetry && smartPortMasterSerialPort) {
            freeSmartPortTelemetryPort();
        }
    }
}

bool initSmartPortMasterTelemetry(void)
{
    if (telemetryState == TELEMETRY_STATE_UNINITIALIZED) {
        memset(activePhysicalId, MAX_SMART_PORT_MASTER_SENSORS, sizeof(activePhysicalId));
        activePhysicalIdCnt = 0;
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT_MASTER);
        if (portConfig) {
            smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT_MASTER);
            telemetryState = TELEMETRY_STATE_SEND;
        }
        return true;
    }
    return false;
}

static void updateAltitude(uint8_t* sensorData, uint32_t currentTimeUs)
{
    //S32, 20cm, range of: -500~90000m
    smartPortMasterSensorData.vario.altitude.value = sensorData[3] + (sensorData[4] << 8) + (sensorData[5] << 16) + (sensorData[6] << 24);
    smartPortMasterSensorData.vario.altitude.updateTimeUs = currentTimeUs;
}

static void updateVario(uint8_t* sensorData, uint32_t currentTimeUs)
{
    //S32, 0.5cm/s, range of: -10.24~10.24m/s
    smartPortMasterSensorData.vario.altitudeRate.value = sensorData[3] + (sensorData[4] << 8) + (sensorData[5] << 16) + (sensorData[6] << 24);
    smartPortMasterSensorData.vario.altitudeRate.updateTimeUs = currentTimeUs;
}

static void updateCurrent(uint8_t* sensorData, uint32_t currentTimeUs)
{
    //U32, A/10 (0.1A), range of: 0~150.0A
    smartPortMasterSensorData.current.draw = sensorData[6] + (sensorData[5] << 8) + (sensorData[4] << 16) + (sensorData[3] << 24);
    smartPortMasterSensorData.current.updateTimeUs = currentTimeUs;
}

static void updateVoltage(uint8_t* sensorData, uint32_t currentTimeUs)
{
    //U16, V/100 (0.01v), range of 0~4.2v
    // V/500, 0.01v/4.2
    uint32_t voltage = (sensorData[3]) + (sensorData[5] << 8) + (sensorData[6] << 16);
    smartPortMasterSensorData.voltage.cellCount = (sensorData[3] & 0xF0) >> 4;
    smartPortMasterSensorData.voltage.updateTimeMs = currentTimeUs * 1000;
    uint8_t voltageF0 = sensorData[3] & 0x0F;
    switch (voltageF0)
    {
        case 0:
            smartPortMasterSensorData.voltage.volts[0] = ((voltage) & 0x000FFF) / 5;
            smartPortMasterSensorData.voltage.volts[1] = ((voltage & 0xFFF000) >> 12) / 5;
        break;
        case 2:
            smartPortMasterSensorData.voltage.volts[2] = ((voltage) & 0x000FFF) / 5;
            smartPortMasterSensorData.voltage.volts[3] = ((voltage & 0xFFF000) >> 12) / 5;
        break;
        case 4:
            smartPortMasterSensorData.voltage.volts[4] = ((voltage) & 0x000FFF) / 5;
            smartPortMasterSensorData.voltage.volts[5] = ((voltage & 0xFFF000) >> 12) / 5;
        break;
    }
}

static void updateVS600(uint8_t* sensorData, uint32_t currentTimeUs)
{
    smartPortMasterSensorData.vs600.vtxPresent = true;
    if(sensorData[6] > 9)
    {
        smartPortMasterSensorData.vs600.currentStatus.allFreq = true;
        smartPortMasterSensorData.vs600.currentStatus.channel = sensorData[6] - 10;
        smartPortMasterSensorData.vs600.currentStatus.power = sensorData[4];
    }
    else
    {
        smartPortMasterSensorData.vs600.currentStatus.allFreq = false;
        smartPortMasterSensorData.vs600.currentStatus.channel = sensorData[6];
        smartPortMasterSensorData.vs600.currentStatus.power = sensorData[4] - 10;
    }
    smartPortMasterSensorData.vs600.currentStatus.band = sensorData[5];
    smartPortMasterSensorData.vs600.currentStatus.updateTimeUs = currentTimeUs;
}

static uint8_t getFrskyCrc(smartPortMasterPayload_t* payload)
{
    uint16_t checksum = 0x00;
    uint8_t* dataPtr = (uint8_t*)payload;

    for(uint8_t x=0; x<sizeof(smartPortMasterPayload_t)-1; x++)
    {
        checksum += *(dataPtr++);
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    checksum = 0xFF - (checksum & 0xff);
    return checksum;
}

static uint8_t getFportCrc(smartPortMasterPayload_t* payload)
{
    uint16_t checksum = 0x00;
    uint8_t* dataPtr = (uint8_t*)payload;

    for(uint8_t x=0; x<sizeof(smartPortMasterPayload_t); x++)
    {
        checksum += *(dataPtr++);
    }
    checksum = (checksum & 0xFF) + (checksum >> 8);
    checksum = 0xFF - (checksum & 0xff);
    return checksum;
}

static uint8_t xorSportPhysicalID(uint8_t physicalID)
{
    uint8_t physicalIDCrc;
    physicalIDCrc  = ((((physicalID >> 0) & 0x01) ^ ((physicalID >> 1) & 0x01) ^ ((physicalID >> 2) & 0x01)) << 5);
    physicalIDCrc |= ((((physicalID >> 2) & 0x01) ^ ((physicalID >> 3) & 0x01) ^ ((physicalID >> 4) & 0x01)) << 6);
    physicalIDCrc |= ((((physicalID >> 0) & 0x01) ^ ((physicalID >> 2) & 0x01) ^ ((physicalID >> 4) & 0x01)) << 7);
    return physicalID | physicalIDCrc;
}

static void generateFrskyCommand(frskyCommand_e frskyCommand, uint8_t command, uint8_t* commandSize, smartPortMasterFrame_t* commandBuffer, uint8_t physicalID, uint16_t applicationID, uint8_t field, uint8_t db4, uint8_t db5, uint8_t db6)
{
    memset(commandBuffer, 0, sizeof(smartPortMasterFrame_t));
    if(currentProtocol == FRSKY_PROTOCOL_FPORT)
        commandBuffer->frameHead = 0x08;
    else
        commandBuffer->frameHead = 0x7E;

    switch (frskyCommand)
    {
        case FRSKY_CMD_READ_NODE:
            if(currentProtocol == FRSKY_PROTOCOL_FPORT)
            {
                commandBuffer->physicalID = xorSportPhysicalID(physicalID); //commandBuffer->frameHead;
                *commandSize = sizeof(smartPortMasterFrame_t);
                commandBuffer->payload.crc = getFportCrc((smartPortMasterPayload_t *)&(commandBuffer->physicalID));
            }
            else
            {
                commandBuffer->physicalID = xorSportPhysicalID(physicalID);
                *commandSize = 2;
            }
            break;
        case FRSKY_CMD_STATE_TRANSFER:
            *commandSize = sizeof(smartPortMasterFrame_t);
            commandBuffer->physicalID = xorSportPhysicalID(physicalID);
            commandBuffer->payload.data[0] = command;
            commandBuffer->payload.data[1] = applicationID & 0xFF;
            commandBuffer->payload.data[2] = (applicationID >> 8) & 0xFF;
            commandBuffer->payload.data[3] = field;
            if(currentProtocol == FRSKY_PROTOCOL_FPORT)
                commandBuffer->payload.crc = getFportCrc((smartPortMasterPayload_t *)&(commandBuffer->physicalID));
            else
                commandBuffer->payload.crc = getFrskyCrc(&(commandBuffer->payload));
            break;
        case FRSKY_CMD_CONFIG_WITH_WRITE:
            *commandSize = sizeof(smartPortMasterFrame_t);
            commandBuffer->physicalID = xorSportPhysicalID(physicalID);
            commandBuffer->payload.data[0] = command;
            commandBuffer->payload.data[1] = applicationID & 0xFF;
            commandBuffer->payload.data[2] = (applicationID >> 8) & 0xFF;
            commandBuffer->payload.data[3] = field;
            commandBuffer->payload.data[4] = db4;
            commandBuffer->payload.data[5] = db5;
            commandBuffer->payload.data[6] = db6;
            if(currentProtocol == FRSKY_PROTOCOL_FPORT)
                commandBuffer->payload.crc = getFportCrc((smartPortMasterPayload_t *)&(commandBuffer->physicalID));
            else
                commandBuffer->payload.crc = getFrskyCrc(&(commandBuffer->payload));
            break;
            break;
        case FRSKY_CMD_CONFIG_WITH_READ:
            *commandSize = sizeof(smartPortMasterFrame_t);
            commandBuffer->physicalID = xorSportPhysicalID(physicalID);
            commandBuffer->payload.data[0] = command;
            commandBuffer->payload.data[1] = applicationID & 0xFF;
            commandBuffer->payload.data[2] = (applicationID >> 8) & 0xFF;
            commandBuffer->payload.data[3] = field;
            if(currentProtocol == FRSKY_PROTOCOL_FPORT)
                commandBuffer->payload.crc = getFportCrc((smartPortMasterPayload_t *)&(commandBuffer->physicalID));
            else
                commandBuffer->payload.crc = getFrskyCrc(&(commandBuffer->payload));
            break;
            break;
        case FRSKY_CMD_DIAG_WITH_READ:
            break;
        case FRSKY_CMD_ENABLE_APP_NODE:
            break;
        case FRSKY_CMD_DISABLE_APP_NODE:
            break;
    }
}


static bool applicationAlreadyFound(uint16_t applicationId)
{
    for(uint8_t x=0; x<activeApplicationIdCnt; x++)
        if(activeApplicationId[x] == applicationId)
            return true;
    return false;
}

static bool sensorAlreadyFound(uint8_t sensorId)
{
    if(sensorId == VS600_WRITE_ID)
        return true;
    for(uint8_t x=0; x<activePhysicalIdCnt; x++)
        if(activePhysicalId[x] == sensorId)
            return true;
    return false;
}

static void sendSmartPortMasterTelemetryRequest(bool updateVS600Now)
{
    static bool lookForNewSensors = true;
    static uint8_t knownPhysicalIDIdx = 0;
    static uint8_t tempPhysicalId = 0;

    smartPortMasterFrame_t masterFrame;
    uint8_t* masterFramePtr = (uint8_t*)&masterFrame;
    uint8_t commandSize;

    if(lookForNewSensors || !activePhysicalIdCnt) {
        if(smartPortMasterSensorData.vs600.state != VS600_STEP_INIT || updateVS600Now)
        {
            switch (smartPortMasterSensorData.vs600.state)
            {
                case VS600_STEP_INIT:
                    currentPhysicalId = VS600_WRITE_ID;
                    generateFrskyCommand(FRSKY_CMD_STATE_TRANSFER, FRSKY_PRIM_IDLE_STATE, &commandSize, &masterFrame, currentPhysicalId, FSSP_DATAID_VS600, 0x80, 0, 0, 0);
                    smartPortMasterSensorData.vs600.state = VS600_STEP_SEND_REQUEST;
                    break;
                case VS600_STEP_SEND_REQUEST:
                    currentPhysicalId = VS600_WRITE_ID;
                    uint8_t pts = smartPortMasterSensorData.vs600.requestedStatus.power;
                    if(!smartPortMasterSensorData.vs600.requestedStatus.allFreq)
                    {
                        pts = pts + 10;
                    }
                    generateFrskyCommand(FRSKY_CMD_CONFIG_WITH_WRITE, FRSKY_PRIM_CONFIG_WRITE, &commandSize, &masterFrame, currentPhysicalId, FSSP_DATAID_VS600, 0x80, pts, smartPortMasterSensorData.vs600.requestedStatus.band, smartPortMasterSensorData.vs600.requestedStatus.channel);
                    smartPortMasterSensorData.vs600.state = VS600_STEP_VERIFY_REQUEST;
                    break;
                case VS600_STEP_VERIFY_REQUEST:
                    currentPhysicalId = VS600_WRITE_ID;
                    generateFrskyCommand(FRSKY_CMD_CONFIG_WITH_READ, FRSKY_PRIM_CONFIG_READ, &commandSize, &masterFrame, currentPhysicalId, FSSP_DATAID_VS600, 0x80, 0, 0, 0);
                    primResponseReceived = false;
                    smartPortMasterSensorData.vs600.state = currentProtocol == FRSKY_PROTOCOL_FPORT ? VS600_STEP_CHECK_REQUEST : VS600_STEP_READ_REQUEST;
                    break;
                case VS600_STEP_READ_REQUEST:
                    currentPhysicalId = VS600_READ_ID;
                    primResponseReceived = false;
                    generateFrskyCommand(FRSKY_CMD_READ_NODE, 0, &commandSize, &masterFrame, currentPhysicalId, 0, 0, 0, 0, 0);
                    smartPortMasterSensorData.vs600.state = VS600_STEP_CHECK_REQUEST;
                    break;
                case VS600_STEP_CHECK_REQUEST:
                    currentPhysicalId = VS600_READ_ID;
                    if (primResponseReceived) {
                        primResponseReceived = false;
                        updateVS600(primResponse, micros());
                        if(vs600MatchesRequest())
                        {
                            smartPortMasterSensorData.vs600.state = VS600_STEP_REINIT;
                        } else {
                            smartPortMasterSensorData.vs600.state = VS600_STEP_SEND_REQUEST;
                        }
                    } else {
                        smartPortMasterSensorData.vs600.state = VS600_STEP_VERIFY_REQUEST;
                    }
                    break;
                case VS600_STEP_REINIT:
                    currentPhysicalId = VS600_WRITE_ID;
                    delay(15);
                    generateFrskyCommand(FRSKY_CMD_STATE_TRANSFER, FRSKY_PRIM_WORKING_STATE, &commandSize, &masterFrame, currentPhysicalId, FSSP_DATAID_VS600, 0x80, 0, 0, 0);
                    smartPortMasterSensorData.vs600.state = VS600_STEP_INIT;
                    break;
            }
        } else {
            currentPhysicalId = tempPhysicalId++;
            if(tempPhysicalId >= MAX_SMART_PORT_MASTER_SENSORS)
                tempPhysicalId = 0;
            lookForNewSensors = false;
            generateFrskyCommand(FRSKY_CMD_READ_NODE, 0, &commandSize, &masterFrame, currentPhysicalId, 0, 0, 0, 0, 0);
        }
    } else {
        currentPhysicalId = activePhysicalId[knownPhysicalIDIdx++];
        if(knownPhysicalIDIdx >= activePhysicalIdCnt)
            knownPhysicalIDIdx = 0;
        lookForNewSensors = true;
        generateFrskyCommand(FRSKY_CMD_READ_NODE, 0, &commandSize, &masterFrame, currentPhysicalId, 0, 0, 0, 0, 0);
    }

    while(commandSize--)
        serialWrite(smartPortMasterSerialPort, *(masterFramePtr++));
}

static uint16_t getApplicationIDFromPid(uint8_t pid)
{
    for (int x=0; x<activeApplicationIdCnt; x++)
        if (pid == activeApplicationIdToPid[x])
            return activeApplicationIdToPid[x];

    return 0;
}

static uint8_t getApplicationCounterFromApplicationId(uint16_t applicationId)
{
    for (int x=0; x<activeApplicationIdCnt; x++)
        if (applicationId == activeApplicationId[x])
            return x;

    return 0;
}

static void processSensorData(uint8_t* sensorData, uint32_t currentTimeUs)
{
    uint16_t applicationId = (sensorData[2] << 8) + (sensorData[1]);

    if(applicationId && !applicationAlreadyFound(applicationId))
    {
        activeApplicationIdToPid[activeApplicationIdCnt] = currentPhysicalId;
        activeApplicationId[activeApplicationIdCnt++] = applicationId;
    }

    sensorRawData[getApplicationCounterFromApplicationId(applicationId)] = (sensorData[6] << 24) + (sensorData[5] << 16) + (sensorData[4] << 8) + (sensorData[3]);

    applicationId &= 0xFF0;
    switch (applicationId)
    {
        case FSSP_DATAID_ALTITUDE:
            updateAltitude(sensorData, currentTimeUs);
            break;
        case FSSP_DATAID_VARIO:
            updateVario(sensorData, currentTimeUs);
            break;
        case FSSP_DATAID_CURRENT:
            updateCurrent(sensorData, currentTimeUs);
            break;
        case FSSP_DATAID_CELLS:
            updateVoltage(sensorData, currentTimeUs);
            break;
        case FSSP_DATAID_VS600:
            updateVS600(sensorData, currentTimeUs);
            break;
        default:
        break;
    }
}

static bool processSmartPortMasterPacket(smartPortMasterPayload_t* frame, uint32_t currentTimeUs, uint8_t physId)
{
    if(physId != MAX_SMART_PORT_MASTER_SENSORS)
    {
        if(!sensorAlreadyFound(currentPhysicalId))
            activePhysicalId[activePhysicalIdCnt++] = currentPhysicalId;

        switch (frame->data[0])
        {
            case FRSKY_PRIM_DATA_FRAME:
                processSensorData(frame->data, currentTimeUs);
                memset(frame->data, 0, 8);
                break;
            case FRSKY_PRIM_CONFIG_RESPONSE:
                primResponseReceived = true;
                memcpy(primResponse, frame->data, 8);
                memset(frame->data, 0, 8);
                break;
        }
    }
    return true;
}

static bool handleSmartPortMasterTelemetryReceive(uint8_t* smartPortRxBytes, uint32_t currentTimeUs)
{
    static uint8_t rxBuffer[sizeof(smartPortMasterFrame_t)*2];
    static bool xorNext = false;

    int waitSize = currentProtocol == FRSKY_PROTOCOL_FPORT ? sizeof(smartPortMasterFrame_t)*2 : sizeof(smartPortMasterFrame_t);

    while (serialRxBytesWaiting(smartPortMasterSerialPort) > 0 && ((*smartPortRxBytes) < waitSize ))
    {
        uint8_t c = serialRead(smartPortMasterSerialPort);
        if (c == 0x7D) {
            xorNext = true;
            return false;
        } else if (xorNext) {
            c ^= 0x20;
            xorNext = false;
        }

        rxBuffer[(*smartPortRxBytes)++] = (uint8_t)c;

        if((*smartPortRxBytes) >= waitSize )
        {
            uint8_t* buffPtrLoc = currentProtocol == FRSKY_PROTOCOL_FPORT ? rxBuffer+12 : rxBuffer+2;
            uint8_t* pidPtrLoc = currentProtocol == FRSKY_PROTOCOL_FPORT ? rxBuffer+11 : rxBuffer+1;
            uint8_t calculatedCRC = currentProtocol == FRSKY_PROTOCOL_FPORT ? getFportCrc((smartPortMasterPayload_t*)(buffPtrLoc-1)) : getFrskyCrc((smartPortMasterPayload_t*)buffPtrLoc);
            if(calculatedCRC == ((smartPortMasterPayload_t*)buffPtrLoc)->crc)
                return processSmartPortMasterPacket((smartPortMasterPayload_t*)(buffPtrLoc), currentTimeUs, *pidPtrLoc);
        }
    }

    return false;
}

void handleSmartPortMasterTelemetry(uint32_t currentTime)
{
    static uint32_t lastTxTime = 0;
    static uint8_t smartPortRxBytes;
    bool updateVS600Now = false;
    static bool skipFirstSwitch = true;

    if ( telemetryState == TELEMETRY_STATE_UNINITIALIZED ) {
        return;
    }

    if ( telemetryState == TELEMETRY_STATE_RECEIVE && (currentTime - lastTxTime > SMARTPORT_RX_WINDOW_US) ) {
        handleSmartPortMasterTelemetryReceive(&smartPortRxBytes, currentTime);
    }

    if(currentTime - lastTxTime >= ( (currentProtocol == FRSKY_PROTOCOL_FPORT) ? FPORT_TX_RATE_US : SMARTPORT_TX_RATE_US) ) {
        if ( !activePhysicalIdCnt && !ARMING_FLAG(ARMED) && currentPhysicalId == 0) { //check both s.port and fport
            if (currentProtocol == FRSKY_PROTOCOL_FPORT){
                currentProtocol = FRSKY_PROTOCOL_SPORT;
            } else if (currentProtocol == FRSKY_PROTOCOL_SPORT) {
                currentProtocol = FRSKY_PROTOCOL_FPORT;
            }
            if(!skipFirstSwitch) {
                freeSmartPortTelemetryPort();
                checkSmartPortMasterTelemetryState();
                delayMicroseconds(200);
            } else {
                skipFirstSwitch = false;
            }

        }

        lastTxTime = currentTime;
        handleSmartPortMasterTelemetryReceive(&smartPortRxBytes, currentTime);
        while (serialRxBytesWaiting(smartPortMasterSerialPort)) {
            serialRead(smartPortMasterSerialPort);
        }
        smartPortRxBytes = 0;

        if(sensorAlreadyFound(VS600_READ_ID)) {
            vs600Init();
            if(!vs600MatchesRequest()) {
                updateVS600Now = true;
            }
        }
        sendSmartPortMasterTelemetryRequest(updateVS600Now);
        telemetryState = TELEMETRY_STATE_RECEIVE;
    }
}

//passthru
void returnRawSensorData(serialPort_t *instance, smartPortMasterFrame_t* frame)
{
    smartPortMasterFrame_t replyFrame;
    uint8_t* replyFramePtr = (uint8_t*)&replyFrame;
    uint8_t commandSize = 10;

    if (sensorAlreadyFound(currentPhysicalId))
    {
        replyFrame.frameHead = 0x08;
        replyFrame.physicalID = frame->physicalID;
        replyFrame.payload.data[0] = 0x10;
        uint16_t appId = getApplicationIDFromPid(replyFrame.physicalID & 0x1F);
        replyFrame.payload.data[1] = appId & 0xFF;
        replyFrame.payload.data[2] = (appId >> 8) & 0xFF;
        uint32_t rawData = sensorRawData[getApplicationCounterFromApplicationId(appId)];
        replyFrame.payload.data[3] = rawData & 0xFF;
        replyFrame.payload.data[4] = (rawData >> 8) & 0xFF;
        replyFrame.payload.data[5] = (rawData >> 16) & 0xFF;
        replyFrame.payload.data[6] = (rawData >> 24) & 0xFF;
        replyFrame.payload.crc     =  getFportCrc((smartPortMasterPayload_t *)&(replyFrame.physicalID));

        delayMicroseconds(500);
        while(commandSize--)
            serialWrite(instance, *(replyFramePtr++));
    }

}

void writeFportToSerial(serialPort_t *instance, const smartPortPayload_t *payload)
{

    uint8_t commandSize = 10;
    smartPortMasterFrame_t replyFrame;
    uint8_t* replyFramePtr = (uint8_t*)&replyFrame;

    if(!payload)
    {
        while(commandSize--)
            serialWrite(instance, 0);
    }
    else
    {
        replyFrame.frameHead = 0x08;
        replyFrame.physicalID = 0xB7;
        replyFrame.payload.data[0] = 0x10;
        replyFrame.payload.data[1] = payload->valueId & 0xFF;
        replyFrame.payload.data[2] = (payload->valueId >> 8) & 0xFF;
        replyFrame.payload.data[3] = payload->data & 0xFF;
        replyFrame.payload.data[4] = (payload->data >> 8) & 0xFF;
        replyFrame.payload.data[5] = (payload->data >> 16) & 0xFF;
        replyFrame.payload.data[6] = (payload->data >> 24) & 0xFF;
        replyFrame.payload.crc     =  getFportCrc((smartPortMasterPayload_t *)&(replyFrame.physicalID));

        while(commandSize--)
            serialWrite(instance, *(replyFramePtr++));
    }
}

#endif
