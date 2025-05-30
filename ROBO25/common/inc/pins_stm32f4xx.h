#ifdef STM32F4XX_HYREL
#ifndef pins_stm32f4xx_HEADER // prevent double dipping
#define pins_stm32f4xx_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    pins_stm32f4xx.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains common pin/gpio specific defines, global references, and method prototypes
//          for 407/429 designs
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////
//
// the next set of defines build up in a single constant the information defining each pin on
// the device in the following format:
//
// this packs in the the 32-bit constant as follows:
//      [3:0] - PIN_NUM  (which bit in the port, values 0 to 15
//
//      [7:4] - PORT_NUM  (which port the signal is attached to, 0=GPIOA, 1=GPIOB, etc
//
//      [9:8] - MODE  (set whether, input, output, analog or whether use to a defined alternate function (UART, CAN, SPI, etc)
//                              0 : Input
//                              1 : Output
//                              2 : Alternate Function
//                              3 : Analog//      [11:8] - AFR  (which Alternate Function to use if the pin is set to MODE=AF)
//                              (see AF mapping table in part datasheet
//
//      [11:10] - OSPEED (needed speep for output)
//                              0 : 2MHz
//                              1 : 25MHz
//                              2 : 50MHz
//                              3 : 100MHz
//
//      [12] - INIT_VAL  (initialization value, if initialization enabled)
//                              0 : Init Low
//                              1 : Init High
//
//      [13] - INIT ENABLE
//                              0 : no initialization
//                              1 : Init to INIT_VAL
//
//      [15:14] - PUPD (whether to add a pullup or pulldown or leaving input floating
//                              0 : no pullup or pulldown -- floating input
//                              1 : add weak pullup to input
//                              2 : add weak pulldown to input
//
//      [19:16] - AFR  (which Alternate Function to use if the pin is set to MODE=AF)
//                              (see AF mapping table in part datasheet
//
//      [20] - OTYPE (output type for pins with MODE=OUTPUT)
//                              0    : Push Pull output
//                              1    : Open drain output
//
//      [21] - LOCKR (whether to lock the pin/port definition for this I/O after initialization)
//             XXX CURRENTLY NOT IMPLEMENTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                              0 = UNLOCKED (power on default)
//                              1 = LOCKED
//
//
// using the resulting #define for each pin, initialization can be easily done as well as easily construct
// which physical registers to read or write in order get or set the value of the pin.   In the event a
// definition needs to change (moving a pin for example), only the definition needs to change in this
// and everything else should be self correcting.
//
////////////////////////////////////////////////////////////////////////////////

typedef union {
	uint32_t u32;
	struct {
		unsigned pinNum     : 4;    // shift 0
		unsigned portNum    : 4;    // shift 4
		unsigned mode       : 2;    // shift 8
		unsigned speed      : 2;    // shift 10
		unsigned initVal    : 1;    // shift 11
		unsigned initEn     : 1;    // shift 12
		unsigned pupd       : 2;    // shift 14
		unsigned af         : 4;    // shift 16
		unsigned otype      : 1;    // shift 20
		unsigned lock       : 1;    // shift 21
	} b;
} pinUnion_t;

#define PIN_NUM_SHIFT          0        // pin[3:0]
#define PIN_PORT_NUM_SHIFT     4        // pin[7:4]
#define PIN_MODE_SHIFT         8        // pin[9:8]
#define PIN_OSPEED_SHIFT       10       // pin[11:10]
#define PIN_INIT_VAL_SHIFT     12       // pin[12]
#define PIN_INIT_EN_SHIFT      13       // pin[13]
#define PIN_PUPPD_SHIFT        14       // pin[15:14]
#define PIN_AF_SHIFT           16       // pin[19:16]
#define PIN_OTYPE_SHIFT        20       // pin[20]
#define PIN_LOCK_SHIFT         21       // pin[21]

////////////////////////////////////////////////////////////////////////////////
//
// define a simple pin number mapping
//

#define PIN_NUM_00             (0x0 << PIN_NUM_SHIFT)
#define PIN_NUM_01             (0x1 << PIN_NUM_SHIFT)
#define PIN_NUM_02             (0x2 << PIN_NUM_SHIFT)
#define PIN_NUM_03             (0x3 << PIN_NUM_SHIFT)
#define PIN_NUM_04             (0x4 << PIN_NUM_SHIFT)
#define PIN_NUM_05             (0x5 << PIN_NUM_SHIFT)
#define PIN_NUM_06             (0x6 << PIN_NUM_SHIFT)
#define PIN_NUM_07             (0x7 << PIN_NUM_SHIFT)
#define PIN_NUM_08             (0x8 << PIN_NUM_SHIFT)
#define PIN_NUM_09             (0x9 << PIN_NUM_SHIFT)
#define PIN_NUM_10             (0xa << PIN_NUM_SHIFT)
#define PIN_NUM_11             (0xb << PIN_NUM_SHIFT)
#define PIN_NUM_12             (0xc << PIN_NUM_SHIFT)
#define PIN_NUM_13             (0xd << PIN_NUM_SHIFT)
#define PIN_NUM_14             (0xe << PIN_NUM_SHIFT)
#define PIN_NUM_15             (0xf << PIN_NUM_SHIFT)


#define PIN_UNDEFINED   (0xffffffff)

#define PIN_MASK_00  0x0001
#define PIN_MASK_01  0x0002
#define PIN_MASK_02  0x0004
#define PIN_MASK_03  0x0008
#define PIN_MASK_04  0x0010
#define PIN_MASK_05  0x0020
#define PIN_MASK_06  0x0040
#define PIN_MASK_07  0x0080
#define PIN_MASK_08  0x0100
#define PIN_MASK_09  0x0200
#define PIN_MASK_10  0x0400
#define PIN_MASK_11  0x0800
#define PIN_MASK_12  0x1000
#define PIN_MASK_13  0x2000
#define PIN_MASK_14  0x4000
#define PIN_MASK_15  0x8000

#define SET_BIT_00                       ((uint32_t)0x00000001)
#define SET_BIT_01                       ((uint32_t)0x00000002)
#define SET_BIT_02                       ((uint32_t)0x00000004)
#define SET_BIT_03                       ((uint32_t)0x00000008)
#define SET_BIT_04                       ((uint32_t)0x00000010)
#define SET_BIT_05                       ((uint32_t)0x00000020)
#define SET_BIT_06                       ((uint32_t)0x00000040)
#define SET_BIT_07                       ((uint32_t)0x00000080)
#define SET_BIT_08                       ((uint32_t)0x00000100)
#define SET_BIT_09                       ((uint32_t)0x00000200)
#define SET_BIT_10                      ((uint32_t)0x00000400)
#define SET_BIT_11                      ((uint32_t)0x00000800)
#define SET_BIT_12                      ((uint32_t)0x00001000)
#define SET_BIT_13                      ((uint32_t)0x00002000)
#define SET_BIT_14                      ((uint32_t)0x00004000)
#define SET_BIT_15                      ((uint32_t)0x00008000)
#define CLR_BIT_00                       ((uint32_t)0x00010000)
#define CLR_BIT_01                       ((uint32_t)0x00020000)
#define CLR_BIT_02                       ((uint32_t)0x00040000)
#define CLR_BIT_03                       ((uint32_t)0x00080000)
#define CLR_BIT_04                       ((uint32_t)0x00100000)
#define CLR_BIT_05                       ((uint32_t)0x00200000)
#define CLR_BIT_06                       ((uint32_t)0x00400000)
#define CLR_BIT_07                       ((uint32_t)0x00800000)
#define CLR_BIT_08                       ((uint32_t)0x01000000)
#define CLR_BIT_09                       ((uint32_t)0x02000000)
#define CLR_BIT_10                      ((uint32_t)0x04000000)
#define CLR_BIT_11                      ((uint32_t)0x08000000)
#define CLR_BIT_12                      ((uint32_t)0x10000000)
#define CLR_BIT_13                      ((uint32_t)0x20000000)
#define CLR_BIT_14                      ((uint32_t)0x40000000)
#define CLR_BIT_15                      ((uint32_t)0x80000000)
////////////////////////////////////////////////////////////////////////////////
//
// these define the values for the various fields (and most just match the spec defined register values
//

#define PIN_INIT_NONE          (0 << PIN_INIT_EN_SHIFT)
#define PIN_INIT_LOW           ((1 << PIN_INIT_EN_SHIFT) | (0 << PIN_INIT_VAL_SHIFT))
#define PIN_INIT_HIGH          ((1 << PIN_INIT_EN_SHIFT) | (1 << PIN_INIT_VAL_SHIFT))

#define PIN_MODE_IN            (GPIO_Mode_IN  << PIN_MODE_SHIFT)
#define PIN_MODE_OUT           (GPIO_Mode_OUT << PIN_MODE_SHIFT)
#define PIN_MODE_AF            (GPIO_Mode_AF  << PIN_MODE_SHIFT)
#define PIN_MODE_ANALOG        (GPIO_Mode_AN  << PIN_MODE_SHIFT)

#define PIN_OTYPE_PUSHPULL     (GPIO_OType_PP << PIN_OTYPE_SHIFT)
#define PIN_OTYPE_OPENDRAIN    (GPIO_OType_OD << PIN_OTYPE_SHIFT)

#define PIN_OSPEED_2MHZ        (GPIO_Speed_2MHz   << PIN_OSPEED_SHIFT)
#define PIN_OSPEED_25MHZ       (GPIO_Speed_25MHz  << PIN_OSPEED_SHIFT)
#define PIN_OSPEED_50MHZ       (GPIO_Speed_50MHz  << PIN_OSPEED_SHIFT)
#define PIN_OSPEED_100MHZ      (GPIO_Speed_100MHz << PIN_OSPEED_SHIFT)

#define PIN_PUPPD_NONE         (GPIO_PuPd_NOPULL << PIN_PUPPD_SHIFT)
#define PIN_PUPPD_PULLUP       (GPIO_PuPd_UP     << PIN_PUPPD_SHIFT)
#define PIN_PUPPD_PULLDOWN     (GPIO_PuPd_DOWN   << PIN_PUPPD_SHIFT)

#define PIN_LOCK_UNLOCKED      (0 << PIN_LOCK_SHIFT)
#define PIN_LOCK_LOCKED        (1 << PIN_LOCK_SHIFT)

////////////////////////////////////////////////////////////////////////////////
//
// these define all of the legal Alternate Function mappings
// it's up to the coder to made sure a given pin supports the requested mapping
//

#define PIN_AF_CAN1            (PIN_MODE_AF | (GPIO_AF_CAN1      << PIN_AF_SHIFT))    // CAN1 Alternate Function
#define PIN_AF_CAN2            (PIN_MODE_AF | (GPIO_AF_CAN2      << PIN_AF_SHIFT))    // CAN2 Alternate Function
#define PIN_AF_DCMI            (PIN_MODE_AF | (GPIO_AF_DCMI      << PIN_AF_SHIFT))    // DCMI Alternate Function
#define PIN_AF_ETH             (PIN_MODE_AF | (GPIO_AF_ETH       << PIN_AF_SHIFT))    // ETHERNET Alternate Function
#define PIN_AF_EVENTOUT        (PIN_MODE_AF | (GPIO_AF_EVENTOUT  << PIN_AF_SHIFT))    // EVENTOUT Alternate Function
#define PIN_AF_FSMC            (PIN_MODE_AF | (GPIO_AF_FSMC      << PIN_AF_SHIFT))    // FSMC Alternate Function
#define PIN_AF_I2C1            (PIN_MODE_AF | (GPIO_AF_I2C1      << PIN_AF_SHIFT))    // I2C1 Alternate Function
#define PIN_AF_I2C2            (PIN_MODE_AF | (GPIO_AF_I2C2      << PIN_AF_SHIFT))    // I2C2 Alternate Function
#define PIN_AF_I2C3            (PIN_MODE_AF | (GPIO_AF_I2C3      << PIN_AF_SHIFT))    // I2C3 Alternate Function
#define PIN_AF_I2S3ext         (PIN_MODE_AF | (GPIO_AF_I2S3ext   << PIN_AF_SHIFT))    // I2S3ext Alternate Function
#define PIN_AF_MCO             (PIN_MODE_AF | (GPIO_AF_MCO       << PIN_AF_SHIFT))    // MCO (MCO1 and MCO2) Alternate Function
//#define PIN_AF_OTG_FS          (PIN_MODE_AF | (GPIO_AF_OTG_FS    << PIN_AF_SHIFT))    // OTG_FS Alternate Function
#define PIN_AF_OTG_FS          (PIN_MODE_AF | PIN_OSPEED_100MHZ | (GPIO_AF_OTG_FS    << PIN_AF_SHIFT))    // OTG_FS Alternate Function
//#define PIN_AF_OTG_HS          (PIN_MODE_AF | PIN_OSPEED_100MHZ | (GPIO_AF_OTG_HS    << PIN_AF_SHIFT))    // OTG_HS Alternate Function
//#define PIN_AF_OTG_HS_FS       (PIN_MODE_AF | PIN_OSPEED_100MHZ | (GPIO_AF_OTG_HS_FS << PIN_AF_SHIFT))    // OTG HS configured in FS, Alternate Function
#define PIN_AF_RTC_50Hz        (PIN_MODE_AF | (GPIO_AF_RTC_50Hz  << PIN_AF_SHIFT))    // RTC_50Hz Alternate Function
#define PIN_AF_SDIO            (PIN_MODE_AF | (GPIO_AF_SDIO      << PIN_AF_SHIFT))    // SDIO Alternate Function
#define PIN_AF_SPI1            (PIN_MODE_AF | (GPIO_AF_SPI1      << PIN_AF_SHIFT))    // SPI1 Alternate Function
#define PIN_AF_SPI2            (PIN_MODE_AF | (GPIO_AF_SPI2      << PIN_AF_SHIFT))    // SPI2/I2S2 Alternate Function
#define PIN_AF_SPI3            (PIN_MODE_AF | (GPIO_AF_SPI3      << PIN_AF_SHIFT))    // SPI3/I2S3 Alternate Function
#define PIN_AF_SWJ             (PIN_MODE_AF | (GPIO_AF_SWJ       << PIN_AF_SHIFT))    // SWJ (SWD and JTAG) Alternate Function
#define PIN_AF_TAMPER          (PIN_MODE_AF | (GPIO_AF_TAMPER    << PIN_AF_SHIFT))    // TAMPER (TAMPER_1 and TAMPER_2) Alternate Function
#define PIN_AF_TIM1            (PIN_MODE_AF | (GPIO_AF_TIM1      << PIN_AF_SHIFT))    // TIM1 Alternate Function
#define PIN_AF_TIM10           (PIN_MODE_AF | (GPIO_AF_TIM10     << PIN_AF_SHIFT))    // TIM10 Alternate Function
#define PIN_AF_TIM11           (PIN_MODE_AF | (GPIO_AF_TIM11     << PIN_AF_SHIFT))    // TIM11 Alternate Function
#define PIN_AF_TIM12           (PIN_MODE_AF | (GPIO_AF_TIM12     << PIN_AF_SHIFT))    // TIM12 Alternate Function
#define PIN_AF_TIM13           (PIN_MODE_AF | (GPIO_AF_TIM13     << PIN_AF_SHIFT))    // TIM13 Alternate Function
#define PIN_AF_TIM14           (PIN_MODE_AF | (GPIO_AF_TIM14     << PIN_AF_SHIFT))    // TIM14 Alternate Function
#define PIN_AF_TIM2            (PIN_MODE_AF | (GPIO_AF_TIM2      << PIN_AF_SHIFT))    // TIM2 Alternate Function
#define PIN_AF_TIM3            (PIN_MODE_AF | (GPIO_AF_TIM3      << PIN_AF_SHIFT))    // TIM3 Alternate Function
#define PIN_AF_TIM4            (PIN_MODE_AF | (GPIO_AF_TIM4      << PIN_AF_SHIFT))    // TIM4 Alternate Function
#define PIN_AF_TIM5            (PIN_MODE_AF | (GPIO_AF_TIM5      << PIN_AF_SHIFT))    // TIM5 Alternate Function
#define PIN_AF_TIM8            (PIN_MODE_AF | (GPIO_AF_TIM8      << PIN_AF_SHIFT))    // TIM8 Alternate Function
#define PIN_AF_TIM9            (PIN_MODE_AF | (GPIO_AF_TIM9      << PIN_AF_SHIFT))    // TIM9 Alternate Function
#define PIN_AF_TRACE           (PIN_MODE_AF | (GPIO_AF_TRACE     << PIN_AF_SHIFT))    // TRACE Alternate Function
#define PIN_AF_UART4           (PIN_MODE_AF | (GPIO_AF_UART4     << PIN_AF_SHIFT))    // UART4 Alternate Function
#define PIN_AF_UART5           (PIN_MODE_AF | (GPIO_AF_UART5     << PIN_AF_SHIFT))    // UART5 Alternate Function
#define PIN_AF_USART1          (PIN_MODE_AF | (GPIO_AF_USART1    << PIN_AF_SHIFT))    // USART1 Alternate Function
#define PIN_AF_USART2          (PIN_MODE_AF | (GPIO_AF_USART2    << PIN_AF_SHIFT))    // USART2 Alternate Function
#define PIN_AF_USART3          (PIN_MODE_AF | (GPIO_AF_USART3    << PIN_AF_SHIFT))    // USART3 Alternate Function
#define PIN_AF_USART6          (PIN_MODE_AF | (GPIO_AF_USART6    << PIN_AF_SHIFT))    // USART6 Alternate Function

////////////////////////////////////////////////////////////////////////////////
//
// common definitions for basic IO
//

#define AF_OUT_PP_2MHZ    (PIN_MODE_AF | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_2MHZ)
#define AF_OUT_PP_25MHZ   (PIN_MODE_AF | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_25MHZ)
#define AF_OUT_PP_50MHZ   (PIN_MODE_AF | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_50MHZ)
#define AF_OUT_PP_100MHZ  (PIN_MODE_AF | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_100MHZ)

#define AF_OUT_OD_2MHZ    (PIN_MODE_AF | PIN_OTYPE_OPENDRAIN  | PIN_OSPEED_2MHZ)
#define AF_OUT_OD_25MHZ   (PIN_MODE_AF | PIN_OTYPE_OPENDRAIN  | PIN_OSPEED_25MHZ)
#define AF_OUT_OD_50MHZ   (PIN_MODE_AF | PIN_OTYPE_OPENDRAIN  | PIN_OSPEED_50MHZ)
#define AF_OUT_OD_100MHZ  (PIN_MODE_AF | PIN_OTYPE_OPENDRAIN  | PIN_OSPEED_100MHZ)

#define OUTPUT_PP_2MHZ    (PIN_MODE_OUT | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_2MHZ)
#define OUTPUT_PP_25MHZ   (PIN_MODE_OUT | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_25MHZ)
#define OUTPUT_PP_50MHZ   (PIN_MODE_OUT | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_50MHZ)
#define OUTPUT_PP_100MHZ  (PIN_MODE_OUT | PIN_OTYPE_PUSHPULL  | PIN_OSPEED_100MHZ)

#define OUTPUT_OD_2MHZ    (PIN_MODE_OUT | PIN_OTYPE_OPENDRAIN | PIN_OSPEED_2MHZ)
#define OUTPUT_OD_25MHZ   (PIN_MODE_OUT | PIN_OTYPE_OPENDRAIN | PIN_OSPEED_25MHZ)
#define OUTPUT_OD_50MHZ   (PIN_MODE_OUT | PIN_OTYPE_OPENDRAIN | PIN_OSPEED_50MHZ)
#define OUTPUT_OD_100MHZ  (PIN_MODE_OUT | PIN_OTYPE_OPENDRAIN | PIN_OSPEED_100MHZ)

#define AF_IN_FLOATING    (PIN_MODE_IN | PIN_PUPPD_NONE)
#define AF_IN_PULLUP      (PIN_MODE_IN | PIN_PUPPD_PULLUP)
#define AF_IN_PULLDOWN    (PIN_MODE_IN | PIN_PUPPD_PULLDOWN)

#define INPUT_FLOATING    (PIN_MODE_IN | PIN_PUPPD_NONE)
#define INPUT_PULLUP      (PIN_MODE_IN | PIN_PUPPD_PULLUP)
#define INPUT_PULLDOWN    (PIN_MODE_IN | PIN_PUPPD_PULLDOWN)

#define ANALOG_FLOATING   (PIN_MODE_ANALOG | PIN_PUPPD_NONE)
#define ANALOG_PULLUP     (PIN_MODE_ANALOG | PIN_PUPPD_PULLUP)
#define ANALOG_PULLDOWN   (PIN_MODE_ANALOG | PIN_PUPPD_PULLDOWN)

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef pins_stm32f4xx_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
#endif //STM32F4XX_HYREL
