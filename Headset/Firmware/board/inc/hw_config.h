/************************************************************************************

Filename    :   hw_config.h
Content     :   Hardware configuration for the main board.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_

// 12 KB is reserved for the bootloader, and the application firmware 
// starts immediately after
#define APPLICATION_OFFSET (0x4000)
#define APPLICATION_ADDRESS (0x08000000 + APPLICATION_OFFSET)
#define PWM_FREQ PANEL_PWM
#define BL_SCALER (32 - 1)
#define BL_PERIOD(pwm) (((8000000 / BL_SCALER) / pwm))

/******************************************************************************/
#ifdef DK2_REV3
/******************************************************************************/

#define MAG_DRDY_PORT           GPIOC
#define MAG_DRDY_PIN            GPIO_Pin_5
#define MAG_DRDY_SOURCE_PORT    EXTI_PortSourceGPIOC
#define MAG_DRDY_SOURCE         EXTI_PinSource5
#define MAG_DRDY_LINE           EXTI_Line5
#define MAG_DRDY_CHANNEL        EXTI9_5_IRQn

#define MAG_CS_PORT             GPIOC
#define MAG_CS_PIN              GPIO_Pin_6

#define GYRO_FSYNC_PORT         GPIOA
#define GYRO_FSYNC_PIN          GPIO_Pin_4

#define GYRO_CS_PORT            GPIOB
#define GYRO_CS_PIN             GPIO_Pin_12

#define GYRO_IRQ_PORT           GPIOA
#define GYRO_IRQ_PIN            GPIO_Pin_2
#define GYRO_IRQ_SOURCE         GPIO_PinSource2
#define GYRO_IRQ_CHANNEL        TIM_Channel_3
#define GYRO_IRQ_FLAG           TIM_IT_CC3
#define GYRO_IRQ_TIM            TIM2
#define GYRO_IRQ_IRQ            TIM2_IRQn
#define GYRO_IRQ_AF             GPIO_AF_TIM2

#define MEMS_SPI_NUM            SPI2
#define MEMS_SPI_RCC            RCC_APB1Periph_SPI2
#define MEMS_SPI_AF             GPIO_AF_SPI2
#define MEMS_SPI_DMA_TX         DMA1_Channel5
#define MEMS_SPI_DMA_RX         DMA1_Channel4
#define MEMS_SPI_DMA_RCC        RCC_AHBPeriph_DMA1
#define MEMS_SPI_DMA_TX_FLAG    (DMA1_FLAG_TE5 | DMA1_FLAG_TC5)
#define MEMS_SPI_DMA_RX_FLAG    (DMA1_FLAG_TE4 | DMA1_FLAG_TC4)
#define MEMS_SPI_CONFIG_SPEED   SPI_BaudRatePrescaler_32
#define MEMS_SPI_READ_SPEED     SPI_BaudRatePrescaler_4
#define MEMS_PORT               GPIOB
#define MEMS_SCK_PIN            GPIO_Pin_13
#define MEMS_SCK_SOURCE         GPIO_PinSource13
#define MEMS_MISO_PIN           GPIO_Pin_14
#define MEMS_MISO_SOURCE        GPIO_PinSource14
#define MEMS_MOSI_PIN           GPIO_Pin_15
#define MEMS_MOSI_SOURCE        GPIO_PinSource15

#define MCO_PORT                GPIOA
#define MCO_PIN                 GPIO_Pin_8

#define USB_DM_PORT             GPIOA
#define USB_DM_PIN              GPIO_Pin_11

#define USB_DP_PORT             GPIOA
#define USB_DP_PIN              GPIO_Pin_12

#define ACC_PWR_EN_PORT         GPIOB
#define ACC_PWR_EN_PIN          GPIO_Pin_5

#define ACC_PWR_FAULT_PORT      GPIOC
#define ACC_PWR_FAULT_PIN       GPIO_Pin_10
#define ACC_PWR_FAULT_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOC
#define ACC_PWR_FAULT_IRQ_SOURCE        EXTI_PinSource10
#define ACC_PWR_FAULT_IRQ_LINE          EXTI_Line10

#define GPIO0_PORT              GPIOC
#define GPIO0_PIN               GPIO_Pin_7
#define GPIO0_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOC
#define GPIO0_IRQ_SOURCE        EXTI_PinSource7
#define GPIO0_IRQ_LINE          EXTI_Line7

#define CAM_SYNC_IN_PORT              GPIO0_PORT
#define CAM_SYNC_IN_PIN               GPIO0_PIN
#define CAM_SYNC_IN_IRQ_SOURCE_PORT   GPIO0_IRQ_SOURCE_PORT
#define CAM_SYNC_IN_IRQ_SOURCE        GPIO0_IRQ_SOURCE
#define CAM_SYNC_IN_IRQ_LINE          GPIO0_IRQ_LINE

#define GPIO1_PORT              GPIOC
#define GPIO1_PIN               GPIO_Pin_8

// Use GPIO1 since there isn't a dedicated pin
#define CAL_EN_PORT             GPIO1_PORT
#define CAL_EN_PIN              GPIO1_PIN

#define PANEL_PWM_TIM           TIM2
#define PANEL_PWM_TIM_RCC       RCC_APB1Periph_TIM2
#define PANEL_PWM_PORT          GPIOB
#define PANEL_PWM_PIN           GPIO_Pin_3
#define PANEL_PWM_SOURCE        GPIO_PinSource3
#define PANEL_PWM_AF            GPIO_AF_TIM2

#define PANEL_RESET_PORT        GPIOC
#define PANEL_RESET_PIN         GPIO_Pin_3

#define PANEL_MIPI_ERR_PORT     GPIOA
#define PANEL_MIPI_ERR_PIN      GPIO_Pin_0

#define PANEL_VSYNC_PORT        GPIOA
#define PANEL_VSYNC_PIN         GPIO_Pin_1
#define PANEL_VSYNC_SOURCE      GPIO_PinSource1
#define PANEL_VSYNC_CHANNEL     TIM_Channel_2
#define PANEL_VSYNC_FLAG        TIM_IT_CC2
#define PANEL_VSYNC_TIM         TIM2
#define PANEL_VSYNC_IRQ         TIM2_IRQn
#define PANEL_VSYNC_AF          GPIO_AF_TIM2

#define CAM_SYNC_OUT_PORT       GPIOA
#define CAM_SYNC_OUT_PIN        GPIO_Pin_15

#define POWER_BTN_PORT          GPIOB
#define POWER_BTN_PIN           GPIO_Pin_4

#define IR_LED_PWR_EN_PORT      GPIOB
#define IR_LED_PWR_EN_PIN       GPIO_Pin_2

#define USB_SUSPEND_PORT        GPIOC
#define USB_SUSPEND_PIN         GPIO_Pin_11
#define USB_SUSPEND_IRQ_SOURCE_PORT EXTI_PortSourceGPIOC
#define USB_SUSPEND_IRQ_SOURCE      EXTI_PinSource11
#define USB_SUSPEND_IRQ_LINE        EXTI_Line11

#define WW_DETECT_PORT              GPIOB
#define WW_DETECT_PIN               GPIO_Pin_8
#define WW_DETECT_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOB
#define WW_DETECT_IRQ_SOURCE        EXTI_PinSource8
#define WW_DETECT_IRQ_LINE          EXTI_Line8

#define USART_NUM               USART1
#define USART_RCC               RCC_APB2Periph_USART1
#define USART_PORT              GPIOA
#define USART_GPIO_RCC          RCC_AHBPeriph_GPIOA
#define USART_TX_PIN            GPIO_Pin_9
#define USART_TX_SOURCE         GPIO_PinSource9
#define USART_RX_PIN            GPIO_Pin_10
#define USART_RX_SOURCE         GPIO_PinSource10
#define USART_AF                GPIO_AF_USART1

#define BRIDGE_STBY_PORT        GPIOC
#define BRIDGE_STBY_PIN         GPIO_Pin_15

#define BRIDGE_RESET_PORT       GPIOC
#define BRIDGE_RESET_PIN        GPIO_Pin_9

#define BRIDGE_PWR_EN_PORT      GPIOB
#define BRIDGE_PWR_EN_PIN       GPIO_Pin_0

#define BRIDGE_1_2V_EN_PORT     GPIOC
#define BRIDGE_1_2V_EN_PIN      GPIO_Pin_14

#define BRIDGE_IRQ_LINE         EXTI_Line13
#define BRIDGE_IRQ_PORT         GPIOC
#define BRIDGE_IRQ_PIN          GPIO_Pin_13
#define BRIDGE_IRQ_SOURCE       EXTI_PinSource13
#define BRIDGE_IRQ_SOURCE_PORT  EXTI_PortSourceGPIOC
#define BRIDGE_IRQ_CHANNEL      EXTI15_10_IRQn

#define HDMI_5V_PORT            GPIOB
#define HDMI_5V_PIN             GPIO_Pin_9

#define HUB_RESET_PORT          GPIOD
#define HUB_RESET_PIN           GPIO_Pin_2

#define HUB_SELFPWR_PORT        GPIOC
#define HUB_SELFPWR_PIN         GPIO_Pin_12

#define LED_BLUE_PORT           GPIOC
#define LED_BLUE_PIN            GPIO_Pin_0

#define LED_AMBER_PORT          GPIOC
#define LED_AMBER_PIN           GPIO_Pin_1

#define SR_SPI                  SPI1
#define SR_PORT                 GPIOA
#define SR_RCLK_PORT            GPIOA
#define SR_SPI_SCK_PIN          GPIO_Pin_5
#define SR_SPI_SDI_PIN          GPIO_Pin_7
#define SR_RCLK_PIN             GPIO_Pin_6
#define SR_SPI_SCK_SOURCE       GPIO_PinSource5
#define SR_SPI_SDI_SOURCE       GPIO_PinSource7
#define SR_SPI_RCC              RCC_APB2Periph_SPI1
#define SR_SPI_AF               GPIO_AF_SPI1
#define SR_SPI_DMA              DMA1_Channel3
#define SR_SPI_DMA_RCC          RCC_AHBPeriph_DMA1
#define SR_SPI_DMA_FLAG         (DMA1_FLAG_TE3 | DMA1_FLAG_TC3)
#define SR_OE_PORT              GPIOA
#define SR_OE_PIN               GPIO_Pin_3
#define SR_OE_TIM               TIM9
#define SR_OE_TIM_RCC           RCC_APB2Periph_TIM9
#define SR_OE_SOURCE            GPIO_PinSource3
#define SR_OE_AF                GPIO_AF_TIM9
#define SR_OE_IRQ               TIM9_IRQn
#define SR_OE_IRQHandler        TIM9_IRQHandler
#define SR_OE_CC_FLAG           TIM_IT_CC2

#define SYNC_OUTPUT_TIM         TIM3

#define EXPOSURE_TIM            TIM4
#define EXPOSURE_TIM_RCC        RCC_APB1Periph_TIM4
#define EXPOSURE_TIM_IRQ        TIM4_IRQn
#define EXPOSURE_TIM_IRQHandler TIM4_IRQHandler

#define I2C_SCL_PU_EN_PORT      GPIOB
#define I2C_SCL_PU_EN_PIN       GPIO_Pin_1

#define HUB_I2C_EN_PORT         GPIOC
#define HUB_I2C_EN_PIN          GPIO_Pin_2

#define ACC_LED_EN_PORT         GPIOC
#define ACC_LED_EN_PIN          GPIO_Pin_4

#define I2C_SCL_PU_EN_PORT      GPIOB
#define I2C_SCL_PU_EN_PIN       GPIO_Pin_1

#define HUB_I2C_DEVICE          CPAL_I2C1
#define HUB_I2C_CPAL            (&I2C1_DevStructure)
#define HUB_I2C_SLAVE_ADDR      (0xA0)

#define CONFIG_I2C_DEVICE       CPAL_I2C1
#define CONFIG_I2C_CPAL         (&I2C1_DevStructure)
#define CONFIG_SLAVE_ADDR       (0xA2)

/******************************************************************************/
#elif defined(DK2_REV3_3)
/******************************************************************************/

#define MAG_DRDY_PORT           GPIOC
#define MAG_DRDY_PIN            GPIO_Pin_5
#define MAG_DRDY_SOURCE_PORT    EXTI_PortSourceGPIOC
#define MAG_DRDY_SOURCE         EXTI_PinSource5
#define MAG_DRDY_LINE           EXTI_Line5
#define MAG_DRDY_CHANNEL        EXTI9_5_IRQn

#define MAG_CS_PORT             GPIOC
#define MAG_CS_PIN              GPIO_Pin_6

#define GYRO_CS_PORT            GPIOB
#define GYRO_CS_PIN             GPIO_Pin_12

#define GYRO_IRQ_PORT           GPIOA
#define GYRO_IRQ_PIN            GPIO_Pin_2
#define GYRO_IRQ_SOURCE         GPIO_PinSource2
#define GYRO_IRQ_CHANNEL        TIM_Channel_3
#define GYRO_IRQ_FLAG           TIM_IT_CC3
#define GYRO_IRQ_TIM            TIM2
#define GYRO_IRQ_IRQ            TIM2_IRQn
#define GYRO_IRQ_AF             GPIO_AF_TIM2

#define MEMS_SPI_NUM            SPI2
#define MEMS_SPI_RCC            RCC_APB1Periph_SPI2
#define MEMS_SPI_AF             GPIO_AF_SPI2
#define MEMS_SPI_DMA_TX         DMA1_Channel5
#define MEMS_SPI_DMA_RX         DMA1_Channel4
#define MEMS_SPI_DMA_RCC        RCC_AHBPeriph_DMA1
#define MEMS_SPI_DMA_TX_FLAG    (DMA1_FLAG_TE5 | DMA1_FLAG_TC5)
#define MEMS_SPI_DMA_RX_FLAG    (DMA1_FLAG_TE4 | DMA1_FLAG_TC4)
#define MEMS_SPI_CONFIG_SPEED   SPI_BaudRatePrescaler_32
#define MEMS_SPI_READ_SPEED     SPI_BaudRatePrescaler_4
#define MEMS_PORT               GPIOB
#define MEMS_SCK_PIN            GPIO_Pin_13
#define MEMS_SCK_SOURCE         GPIO_PinSource13
#define MEMS_MISO_PIN           GPIO_Pin_14
#define MEMS_MISO_SOURCE        GPIO_PinSource14
#define MEMS_MOSI_PIN           GPIO_Pin_15
#define MEMS_MOSI_SOURCE        GPIO_PinSource15

#define MCO_PORT                GPIOA
#define MCO_PIN                 GPIO_Pin_8

#define USB_DM_PORT             GPIOA
#define USB_DM_PIN              GPIO_Pin_11

#define USB_DP_PORT             GPIOA
#define USB_DP_PIN              GPIO_Pin_12

#define ACC_PWR_EN_PORT         GPIOB
#define ACC_PWR_EN_PIN          GPIO_Pin_5

#define ACC_PWR_FAULT_PORT      GPIOC
#define ACC_PWR_FAULT_PIN       GPIO_Pin_10
#define ACC_PWR_FAULT_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOC
#define ACC_PWR_FAULT_IRQ_SOURCE        EXTI_PinSource10
#define ACC_PWR_FAULT_IRQ_LINE          EXTI_Line10

#define CAM_SYNC_IN_PORT              GPIOC
#define CAM_SYNC_IN_PIN               GPIO_Pin_7
#define CAM_SYNC_IN_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOC
#define CAM_SYNC_IN_IRQ_SOURCE        EXTI_PinSource7
#define CAM_SYNC_IN_IRQ_LINE          EXTI_Line7

#define GPIO0_PORT              GPIOA
#define GPIO0_PIN               GPIO_Pin_4

#define GPIO1_PORT              GPIOC
#define GPIO1_PIN               GPIO_Pin_8

// Use GPIO1 since there isn't a dedicated pin
#define CAL_EN_PORT             GPIO1_PORT
#define CAL_EN_PIN              GPIO1_PIN

#define PANEL_PWM_TIM           TIM2
#define PANEL_PWM_TIM_RCC       RCC_APB1Periph_TIM2
#define PANEL_PWM_PORT          GPIOB
#define PANEL_PWM_PIN           GPIO_Pin_3
#define PANEL_PWM_SOURCE        GPIO_PinSource3
#define PANEL_PWM_AF            GPIO_AF_TIM2

#define PANEL_RESET_PORT        GPIOC
#define PANEL_RESET_PIN         GPIO_Pin_3

#define PANEL_MIPI_ERR_PORT     GPIOA
#define PANEL_MIPI_ERR_PIN      GPIO_Pin_0

#define PANEL_VSYNC_PORT        GPIOA
#define PANEL_VSYNC_PIN         GPIO_Pin_1
#define PANEL_VSYNC_SOURCE      GPIO_PinSource1
#define PANEL_VSYNC_CHANNEL     TIM_Channel_2
#define PANEL_VSYNC_FLAG        TIM_IT_CC2
#define PANEL_VSYNC_TIM         TIM2
#define PANEL_VSYNC_IRQ         TIM2_IRQn
#define PANEL_VSYNC_AF          GPIO_AF_TIM2

#define CAM_SYNC_OUT_PORT       GPIOA
#define CAM_SYNC_OUT_PIN        GPIO_Pin_15

#define POWER_BTN_PORT          GPIOB
#define POWER_BTN_PIN           GPIO_Pin_4

#define IR_LED_PWR_EN_PORT      GPIOB
#define IR_LED_PWR_EN_PIN       GPIO_Pin_2

#define USB_SUSPEND_PORT        GPIOC
#define USB_SUSPEND_PIN         GPIO_Pin_11
#define USB_SUSPEND_IRQ_SOURCE_PORT EXTI_PortSourceGPIOC
#define USB_SUSPEND_IRQ_SOURCE      EXTI_PinSource11
#define USB_SUSPEND_IRQ_LINE        EXTI_Line11

#define WW_DETECT_PORT              GPIOB
#define WW_DETECT_PIN               GPIO_Pin_8
#define WW_DETECT_IRQ_SOURCE_PORT   EXTI_PortSourceGPIOB
#define WW_DETECT_IRQ_SOURCE        EXTI_PinSource8
#define WW_DETECT_IRQ_LINE          EXTI_Line8

#define USART_NUM               USART1
#define USART_RCC               RCC_APB2Periph_USART1
#define USART_PORT              GPIOA
#define USART_GPIO_RCC          RCC_AHBPeriph_GPIOA
#define USART_TX_PIN            GPIO_Pin_9
#define USART_TX_SOURCE         GPIO_PinSource9
#define USART_RX_PIN            GPIO_Pin_10
#define USART_RX_SOURCE         GPIO_PinSource10
#define USART_AF                GPIO_AF_USART1

#define BRIDGE_STBY_PORT        GPIOC
#define BRIDGE_STBY_PIN         GPIO_Pin_15

#define BRIDGE_RESET_PORT       GPIOC
#define BRIDGE_RESET_PIN        GPIO_Pin_9

#define BRIDGE_PWR_EN_PORT      GPIOB
#define BRIDGE_PWR_EN_PIN       GPIO_Pin_0

#define BRIDGE_1_2V_EN_PORT     GPIOC
#define BRIDGE_1_2V_EN_PIN      GPIO_Pin_14

#define BRIDGE_IRQ_LINE         EXTI_Line13
#define BRIDGE_IRQ_PORT         GPIOC
#define BRIDGE_IRQ_PIN          GPIO_Pin_13
#define BRIDGE_IRQ_SOURCE       EXTI_PinSource13
#define BRIDGE_IRQ_SOURCE_PORT  EXTI_PortSourceGPIOC
#define BRIDGE_IRQ_CHANNEL      EXTI15_10_IRQn

#define HDMI_5V_PORT            GPIOB
#define HDMI_5V_PIN             GPIO_Pin_9

#define HUB_RESET_PORT          GPIOD
#define HUB_RESET_PIN           GPIO_Pin_2

#define HUB_SELFPWR_PORT        GPIOC
#define HUB_SELFPWR_PIN         GPIO_Pin_12

#define LED_BLUE_PORT           GPIOC
#define LED_BLUE_PIN            GPIO_Pin_0

#define LED_AMBER_PORT          GPIOC
#define LED_AMBER_PIN           GPIO_Pin_1

#define SR_SPI                  SPI1
#define SR_PORT                 GPIOA
#define SR_RCLK_PORT            GPIOA
#define SR_SPI_SCK_PIN          GPIO_Pin_5
#define SR_SPI_SDI_PIN          GPIO_Pin_7
#define SR_RCLK_PIN             GPIO_Pin_6
#define SR_SPI_SCK_SOURCE       GPIO_PinSource5
#define SR_SPI_SDI_SOURCE       GPIO_PinSource7
#define SR_SPI_RCC              RCC_APB2Periph_SPI1
#define SR_SPI_AF               GPIO_AF_SPI1
#define SR_SPI_DMA              DMA1_Channel3
#define SR_SPI_DMA_RCC          RCC_AHBPeriph_DMA1
#define SR_SPI_DMA_FLAG         (DMA1_FLAG_TE3 | DMA1_FLAG_TC3)
#define SR_OE_PORT              GPIOA
#define SR_OE_PIN               GPIO_Pin_3
#define SR_OE_TIM               TIM9
#define SR_OE_TIM_RCC           RCC_APB2Periph_TIM9
#define SR_OE_SOURCE            GPIO_PinSource3
#define SR_OE_AF                GPIO_AF_TIM9
#define SR_OE_IRQ               TIM9_IRQn
#define SR_OE_IRQHandler        TIM9_IRQHandler
#define SR_OE_CC_FLAG           TIM_IT_CC2

#define SYNC_OUTPUT_TIM         TIM3

#define EXPOSURE_TIM            TIM4
#define EXPOSURE_TIM_RCC        RCC_APB1Periph_TIM4
#define EXPOSURE_TIM_IRQ        TIM4_IRQn
#define EXPOSURE_TIM_IRQHandler TIM4_IRQHandler

#define HUB_I2C_EN_PORT         GPIOC
#define HUB_I2C_EN_PIN          GPIO_Pin_2

#define ACC_LED_EN_PORT         GPIOC
#define ACC_LED_EN_PIN          GPIO_Pin_4

#define I2C_SCL_PU_EN_PORT      GPIOB
#define I2C_SCL_PU_EN_PIN       GPIO_Pin_1

#define HUB_I2C_DEVICE          CPAL_I2C1
#define HUB_I2C_CPAL            (&I2C1_DevStructure)
#define HUB_I2C_SLAVE_ADDR      (0xA0)

#define CONFIG_I2C_DEVICE       CPAL_I2C1
#define CONFIG_I2C_CPAL         (&I2C1_DevStructure)
#define CONFIG_SLAVE_ADDR       (0xA2)

#endif /* DK2_REV3 */

#endif /* _HW_CONFIG_H_ */
