#include "platform.h"

#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_string.h"
#include "io/vtx_vs600.h"

#include "telemetry/smartport_master.h"

#include "drivers/time.h"


#if defined(USE_VTX_COMMON) && defined(USE_TELEMETRY_SMARTPORT_MASTER)

#define VTX_VS600_POWER_COUNT 3
#define VTX_VS600_STRING_BAND_COUNT 6
#define VTX_VS600_STRING_CHAN_COUNT 8

extern SmartPortMasterSensorData_t smartPortMasterSensorData;

static const uint16_t vs600frequencyTable[VTX_VS600_STRING_BAND_COUNT][VTX_VS600_STRING_CHAN_COUNT] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
    { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5612 }, // Race Low
};

static const char * vs600BandNames[] = {
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
    "RACE LOW",
};

static char const vs600BandLetter[] = "-ABEFRL";

static char const * vs600ChannelNames[] = {
    "-", "1", "2", "3", "4", "5", "6", "7", "8",
};

const char * vs600PowerNames[VTX_VS600_POWER_COUNT+1] = {
    "PIT", "25 ", "200", "600",
};

uint16_t vs600PowerValues[VTX_VS600_POWER_COUNT] = {
    25, 200, 600
};

const uint16_t *vs600StringFrequencyTable(void)
{
    return &vs600frequencyTable[0][0];
}

const char **vs600StringBandNames(void)
{
    return vs600BandNames;
}

const char **vs600StringChannelNames(void)
{
    return vs600ChannelNames;
}

const char *vs600StringBandLetters(void)
{
    return vs600BandLetter;
}

static vtxVTable_t vs600VTable;
static vtxDevice_t vs600 = {
    .vTable = &vs600VTable,
};

void vs600Init(void)
{
    static bool init = false;

    if(init)
        return;

    vs600.capability.bandCount = VTX_VS600_STRING_BAND_COUNT,
    vs600.capability.channelCount = VTX_VS600_STRING_CHAN_COUNT,
    vs600.capability.powerCount = VTX_VS600_POWER_COUNT,
    vs600.frequencyTable = vs600StringFrequencyTable();
    vs600.bandNames = vs600StringBandNames();
    vs600.bandLetters = vs600StringBandLetters();
    vs600.channelNames = vs600StringChannelNames();
    vs600.powerNames = vs600PowerNames,
    vs600.powerValues = vs600PowerValues,

    vtxCommonSetDevice(&vs600);

    vtxInit();
    init = true;

}

static uint16_t getVS600Frequency(uint8_t band, uint8_t channel)
{
    if( (band < VTX_VS600_STRING_BAND_COUNT) && (channel < VTX_VS600_STRING_CHAN_COUNT) )
        return vs600frequencyTable[band][channel];
    else
        return 0;
}

static void vs600Process(vtxDevice_t *vtxDevice, timeUs_t now)
{
    UNUSED(vtxDevice);
    UNUSED(now);
}

static bool vs600IsReady(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return smartPortMasterSensorData.vs600.vtxPresent;
}

vtxDevType_e vs600GetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_VS600;
}

static void vs600SetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    smartPortMasterSensorData.vs600.requestedStatus.band = band-1;
    smartPortMasterSensorData.vs600.requestedStatus.channel = channel-1;
    smartPortMasterSensorData.vs600.requestedStatus.updateTimeUs = micros();
}

static void vs600SetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    UNUSED(vtxDevice);
    smartPortMasterSensorData.vs600.requestedStatus.power = index;
    smartPortMasterSensorData.vs600.requestedStatus.updateTimeUs = micros();
}

static void vs600SetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    static uint8_t onPowerMode = 255;
    UNUSED(vtxDevice);

    if(onoff)
    {
        smartPortMasterSensorData.vs600.requestedStatus.pitmode = true;
        if(onPowerMode == 255)
        {
            onPowerMode = smartPortMasterSensorData.vs600.requestedStatus.power;
        }
        smartPortMasterSensorData.vs600.requestedStatus.power = 0;
    }
    else
    {
        smartPortMasterSensorData.vs600.requestedStatus.pitmode = false;
        if(onPowerMode == 255)
        {
            onPowerMode = 1;
        }
        smartPortMasterSensorData.vs600.requestedStatus.power = onPowerMode;
    }
    smartPortMasterSensorData.vs600.requestedStatus.updateTimeUs = micros();
}

static void vs600SetFrequency(vtxDevice_t *vtxDevice, uint16_t freq)
{
    uint8_t band;
    uint8_t channel;

    if(vtxCommonLookupBandChan(vtxDevice, freq, &band, &channel))
    {
        vs600SetBandAndChannel(vtxDevice, band, channel);
    }
}

static bool vs600GetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    UNUSED(vtxDevice);
    *pBand = smartPortMasterSensorData.vs600.currentStatus.band+1;
    *pChannel = smartPortMasterSensorData.vs600.currentStatus.channel+1;
    return true;
}

static bool vs600GetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    UNUSED(vtxDevice);
    if(smartPortMasterSensorData.vs600.requestedStatus.pitmode)
    {
        *pIndex = smartPortMasterSensorData.vs600.currentStatus.power + 1;
    }
    else
    {
        *pIndex = smartPortMasterSensorData.vs600.currentStatus.power;
    }

    return true;
}

static bool vs600GetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    UNUSED(vtxDevice);
    UNUSED(pOnOff);

    return smartPortMasterSensorData.vs600.currentStatus.power == 0;
}

static bool vs600GetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    UNUSED(vtxDevice);
    *pFrequency = getVS600Frequency(smartPortMasterSensorData.vs600.currentStatus.band, smartPortMasterSensorData.vs600.currentStatus.channel);
    return true;
}

static vtxVTable_t vs600VTable = {
    .process = vs600Process,
    .getDeviceType = vs600GetDeviceType,
    .isReady = vs600IsReady,
    .setBandAndChannel = vs600SetBandAndChannel,
    .setPowerByIndex = vs600SetPowerByIndex,
    .setPitMode = vs600SetPitMode,
    .setFrequency = vs600SetFrequency,
    .getBandAndChannel = vs600GetBandAndChannel,
    .getPowerIndex = vs600GetPowerIndex,
    .getPitMode = vs600GetPitMode,
    .getFrequency = vs600GetFreq,
};

bool vs600MatchesRequest(void)
{
    if( micros() - smartPortMasterSensorData.vs600.requestedStatus.updateTimeUs > VS600_TIMEOUT_US )
    {
        smartPortMasterSensorData.vs600.requestedStatus.allFreq = smartPortMasterSensorData.vs600.currentStatus.allFreq;
        smartPortMasterSensorData.vs600.requestedStatus.band = smartPortMasterSensorData.vs600.currentStatus.allFreq;
        smartPortMasterSensorData.vs600.requestedStatus.power = smartPortMasterSensorData.vs600.currentStatus.allFreq;
        smartPortMasterSensorData.vs600.requestedStatus.channel = smartPortMasterSensorData.vs600.currentStatus.allFreq;
        smartPortMasterSensorData.vs600.requestedStatus.updateTimeUs = smartPortMasterSensorData.vs600.currentStatus.allFreq;
        return true;
    }
    if (smartPortMasterSensorData.vs600.currentStatus.allFreq != smartPortMasterSensorData.vs600.requestedStatus.allFreq)
        return false;
    if (smartPortMasterSensorData.vs600.currentStatus.channel != smartPortMasterSensorData.vs600.requestedStatus.channel)
        return false;
    if (smartPortMasterSensorData.vs600.currentStatus.band    != smartPortMasterSensorData.vs600.requestedStatus.band)
        return false;
    if (smartPortMasterSensorData.vs600.currentStatus.power   != smartPortMasterSensorData.vs600.requestedStatus.power)
        return false;

    return true;
}
#endif
