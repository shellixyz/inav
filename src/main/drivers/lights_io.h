
#pragma once

#ifdef USE_LIGHTS


void lightsHardwareInit();

#ifdef WING_LIGHTS_PIN
void wingLightsHardwareSetStatus(bool status);
#endif

#ifdef FRONT_LIGHTS_PIN
void frontLightsHardwareSetStatus(bool status);
#endif

#endif /* USE_LIGHTS */
