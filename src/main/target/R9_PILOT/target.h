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


#pragma once

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS
#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "R9P"
#define USBD_PRODUCT_STRING     "R9PILOT"

#define LED0                    PE0
#define LED1                    PD12

//#define BEEPER                  PD15
//#define BEEPER_INVERTED

#define USE_SPI

// *************** SPI1 BARO **********************

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_BARO
#define USE_BARO_BMP388
#define BMP388_SPI_BUS          BUS_SPI1
#define BMP388_CS_PIN           PA1

// *************** SPI2 receiver *******************

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// *************** SPI3 Gyro & ACC *******************
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_ACC
#define USE_GYRO

#define USE_ACC_MPU6500
#define USE_GYRO_MPU6500
#define MPU6500_SPI_BUS         BUS_SPI3
#define MPU6500_CS_PIN          SPI3_NSS_PIN
#define MPU6500_EXTI_PIN        PE8

#define GYRO_MPU6500_ALIGN      CW270_DEG_FLIP // XXX check
#define ACC_MPU6500_ALIGN       CW270_DEG_FLIP // XXX check

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

// *************** SPI4: SDCARD *******************

#define USE_SPI_DEVICE_4
#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI4
#define SDCARD_CS_PIN           SPI4_NSS_PIN

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// *************** I2C *********************
#define USE_I2C

#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C2

#define PITOT_I2C_BUS           BUS_I2C2

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C2

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC4
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_UART5
#define UART5_TX_PIN            PB6
#define UART5_RX_PIN            PB5
#define UART5_AF                1

// OSD
#define USE_OSD
#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

// RX connector
#define USE_UART7
#define UART7_TX_PIN            PB4     // S_OUT
#define UART7_RX_PIN            PB3     // S_IN
#define UART7_AF                12

#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            NONE

#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PB4     // S.Bus out pad (UART7 TX)
#define SOFTSERIAL_1_TX_PIN     PB4

#define SERIAL_PORT_COUNT       9

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART8

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1

#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC3
#define ADC_CHANNEL_3_PIN           PC5

#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_1
#define VBAT_ADC_CHANNEL            ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES            (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)
#define VBAT_SCALE_DEFAULT          1545
#define CURRENT_METER_SCALE         168
#define CURRENT_METER_OFFSET        3277

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define MAX_PWM_OUTPUT_PORTS 9
