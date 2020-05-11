/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"

BUSDEV_REGISTER_SPI_TAG(busdev_imu0,    DEVHW_MPU6500,      MPU6500_SPI_BUS,    MPU6500_CS_PIN,     GYRO_INT_EXTI,       0,  DEVFLAGS_NONE);

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR, 0, 0),
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR, 0, 0),
    DEF_TIM(TIM1,  CH1, PE9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM1,  CH2, PE11, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM1,  CH3, PE13, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // new
    DEF_TIM(TIM1,  CH4, PE14, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // new
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // new
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // new

    DEF_TIM(TIM5, CH1,  PA0,  TIM_USE_BEEPER, 0, 0),    // beeper
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
