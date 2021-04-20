/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 20/04/2021 20:48:49
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLCLKFreq_Value                     84000000
#define MX_PLLQCLKFreq_Value                    84000000
#define MX_PLLRCLKFreq_Value                    84000000
#define MX_PLLSAIPCLKFreq_Value                 96000000
#define MX_PLLSAIQCLKFreq_Value                 96000000
#define MX_PLLI2SPCLKFreq_Value                 96000000
#define MX_PLLI2SQCLKFreq_Value                 96000000
#define MX_PLLI2SRCLKFreq_Value                 96000000
#define MX_SYSCLKFreq_VALUE                     84000000
#define MX_HCLKFreq_Value                       84000000
#define MX_FCLKCortexFreq_Value                 84000000
#define MX_CortexFreq_Value                     84000000
#define MX_AHBFreq_Value                        84000000
#define MX_APB1Freq_Value                       42000000
#define MX_APB2Freq_Value                       84000000
#define MX_APB1TimFreq_Value                    84000000
#define MX_APB2TimFreq_Value                    84000000
#define MX_CECFreq_Value                        32786
#define MX_I2S1Freq_Value                       96000000
#define MX_I2S2Freq_Value                       96000000
#define MX_SAIAFreq_Value                       96000000
#define MX_SAIBFreq_Value                       96000000
#define MX_SDIOFreq_Value                       84000000
#define MX_PWRFreq_Value                        84000000
#define MX_RTCFreq_Value                        32000
#define MX_USBFreq_Value                        84000000
#define MX_WatchDogFreq_Value                   32000
#define MX_FMPI2C1Freq_Value                    42000000
#define MX_SPDIFRXFreq_Value                    84000000
#define MX_MCO1PinFreq_Value                    16000000
#define MX_MCO2PinFreq_Value                    84000000

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- TIM4       --------------------------------*/

#define MX_TIM4                                 1

/* GPIO Configuration */

/* Pin PB6 */
#define MX_S_TIM4_CH1_GPIO_ModeDefaultPP        GPIO_MODE_AF_PP
#define MX_S_TIM4_CH1_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_S_TIM4_CH1_Pin                       PB6
#define MX_S_TIM4_CH1_GPIOx                     GPIOB
#define MX_S_TIM4_CH1_GPIO_PuPd                 GPIO_NOPULL
#define MX_S_TIM4_CH1_GPIO_Pin                  GPIO_PIN_6
#define MX_S_TIM4_CH1_GPIO_AF                   GPIO_AF2_TIM4

/* Pin PB7 */
#define MX_S_TIM4_CH2_GPIO_ModeDefaultPP        GPIO_MODE_AF_PP
#define MX_S_TIM4_CH2_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_S_TIM4_CH2_Pin                       PB7
#define MX_S_TIM4_CH2_GPIOx                     GPIOB
#define MX_S_TIM4_CH2_GPIO_PuPd                 GPIO_NOPULL
#define MX_S_TIM4_CH2_GPIO_Pin                  GPIO_PIN_7
#define MX_S_TIM4_CH2_GPIO_AF                   GPIO_AF2_TIM4

/*-------------------------------- USART2     --------------------------------*/

#define MX_USART2                               1

#define MX_USART2_VM                            VM_SYNC

/* GPIO Configuration */

/* Pin PA2 */
#define MX_USART2_TX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART2_TX_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_USART2_TX_Pin                        PA2
#define MX_USART2_TX_GPIOx                      GPIOA
#define MX_USART2_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_TX_GPIO_Pin                   GPIO_PIN_2
#define MX_USART2_TX_GPIO_AF                    GPIO_AF7_USART2

/* Pin PA3 */
#define MX_USART2_RX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART2_RX_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_USART2_RX_Pin                        PA3
#define MX_USART2_RX_GPIOx                      GPIOA
#define MX_USART2_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_RX_GPIO_Pin                   GPIO_PIN_3
#define MX_USART2_RX_GPIO_AF                    GPIO_AF7_USART2

/* Pin PA4 */
#define MX_USART2_CK_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_USART2_CK_Pin                        PA4
#define MX_USART2_CK_GPIOx                      GPIOA
#define MX_USART2_CK_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART2_CK_GPIO_Pin                   GPIO_PIN_4
#define MX_USART2_CK_GPIO_AF                    GPIO_AF7_USART2
#define MX_USART2_CK_GPIO_Mode                  GPIO_MODE_AF_PP

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

#endif  /* __MX_DEVICE_H */

