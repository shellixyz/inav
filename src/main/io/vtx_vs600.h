#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t channel;
    uint8_t band;
    uint8_t power;
    bool pitmode;
    bool allFreq;
    uint16_t frequency;
    uint32_t updateTimeUs;
}  VS600State;

typedef enum {
    VS600_STEP_INIT = 0,
    VS600_STEP_SEND_REQUEST,
    VS600_STEP_VERIFY_REQUEST,
    VS600_STEP_READ_REQUEST,
    VS600_STEP_CHECK_REQUEST,
    VS600_STEP_REINIT,
} vs600_step_e;

typedef struct
{
    bool vtxPresent;
    VS600State currentStatus;
    VS600State requestedStatus;
    vs600_step_e state;
}  SmartPortMasterVS600;

#define VS600_READ_ID 0x12
#define VS600_WRITE_ID 0x13
#define VS600_TIMEOUT_US 3000000

void vs600Init(void);
bool vs600MatchesRequest(void);