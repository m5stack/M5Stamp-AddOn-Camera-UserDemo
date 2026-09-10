#pragma once

#include "driver/gpio.h"

/*
 * OV3660 wiring matched by connector pin number:
 *  1 MCLK  -> GPIO46   9 VS    -> GPIO42   19 SCL -> GPIO47
 *  2 D7    -> GPIO40  10 D4    -> GPIO14   23 SDA -> GPIO48
 *  4 D6    -> GPIO38  11 PWDN  -> GPIO39
 *  5 RESET -> GPIO41  12 D3    -> GPIO15
 *  6 D5    -> GPIO12  14 D2    -> GPIO16
 *  8 PCLK  -> GPIO13  15 HS    -> GPIO17
 * 16 D1    -> GPIO18  18 D0    -> GPIO21
 */

#define OV3660_I2C_SCL_PIN      GPIO_NUM_47
#define OV3660_I2C_SDA_PIN      GPIO_NUM_48

#define OV3660_XCLK_PIN         GPIO_NUM_46

#define OV3660_PWDN_PIN         GPIO_NUM_39
#define OV3660_RESET_PIN        GPIO_NUM_41

#define OV3660_D0_PIN           GPIO_NUM_21
#define OV3660_D1_PIN           GPIO_NUM_18
#define OV3660_D2_PIN           GPIO_NUM_16
#define OV3660_D3_PIN           GPIO_NUM_15
#define OV3660_D4_PIN           GPIO_NUM_14
#define OV3660_D5_PIN           GPIO_NUM_12
#define OV3660_D6_PIN           GPIO_NUM_38
#define OV3660_D7_PIN           GPIO_NUM_40

#define OV3660_VSYNC_PIN        GPIO_NUM_42
#define OV3660_HREF_PIN         GPIO_NUM_17
#define OV3660_PCLK_PIN         GPIO_NUM_13
#define OV3660_DE_PIN           OV3660_HREF_PIN
