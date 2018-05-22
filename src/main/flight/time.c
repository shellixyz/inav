
#include "common/time.h"
#include "fc/runtime_config.h"

timeUs_t flyTime = 0;

void taskUpdateFlyTime(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;

    if (ARMING_FLAG(ARMED)) {
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
        flyTime += deltaT;
    }

    lastTimeUs = currentTimeUs;
}
