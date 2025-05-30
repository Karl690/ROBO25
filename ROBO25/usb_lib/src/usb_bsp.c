/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file is responsible to offer board support package and is 
  *          configurable by user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
//NUKE #include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_misc.h"
#include "usb_bsp.h"
#include "usbd_conf.h"

//NUKE #include "stm32f4xx_usb_vcp.h"

//GBXXX #include "defines.h"

#ifndef USB_VCP_NVIC_PRIORITY
#define USB_VCP_NVIC_PRIORITY			0x01
#endif

#ifndef USB_VCP_NVIC_SUBPRIORITY
#define USB_VCP_NVIC_SUBPRIORITY		0x01
#endif

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev);

/**
* @brief  USB_OTG_BSP_Init
*         Initilizes BSP configurations
* @param  None
* @retval None
*/
#define USB_VCP_DISABLE_ID
#define USB_VCP_DISABLE_VBUS

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev) {
//GBXXX  GPIO_InitTypeDef GPIO_InitStructure;
#ifdef USE_USB_OTG_FS
#if 0// GBXXX IO init is elsewhere
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_11 | 	// OTG FS Data -
									GPIO_Pin_12;	// OTG FS Data +

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
#endif
#ifndef USB_VCP_DISABLE_VBUS
	// Configure  VBUS Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
#endif

#ifndef USB_VCP_DISABLE_ID
	// Configure ID pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_OTG1_FS); 
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE); 
  
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_14 | // Data -
									GPIO_Pin_15;  // Data +

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_OTG2_FS);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_OTG2_FS);
	
#ifndef USB_VCP_DISABLE_VBUS
	//Configure VBUS Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_OTG2_FS);
#endif

#ifndef USB_VCP_DISABLE_ID
	//Configure ID pin
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_OTG2_FS);
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE);

#endif
}

/**
* @brief  USB_OTG_BSP_EnableInterrupt
*         Enabele USB Global interrupt
* @param  None
* @retval None
*/
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev) {
#ifdef USE_USB_OTG_FS
	interruptSetupAndEnable(OTG_FS_IRQn, NVIC_PREMPTION_PRIORITY_USB);
#else
	interruptSetupAndEnable(OTG_HS_IRQn, NVIC_PREMPTION_PRIORITY_USB+2);
	interruptSetupAndEnable(OTG_HS_EP1_OUT_IRQn, NVIC_PREMPTION_PRIORITY_USB+1);
	interruptSetupAndEnable(OTG_HS_EP1_IN_IRQn, NVIC_PREMPTION_PRIORITY_USB);
#endif
}
/**
* @brief  USB_OTG_BSP_uDelay
*         This function provides delay time in micro sec
* @param  usec : Value of delay required in micro sec
* @retval None
*/
void USB_OTG_BSP_uDelay (const uint32_t usec) {
	uint32_t count = 0;
	const uint32_t utime = (120 * usec / 7);
	
	do
	{
		if ( ++count > utime ) {
			return ;
		}
	} while (1);
}


/**
* @brief  USB_OTG_BSP_mDelay
*          This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void USB_OTG_BSP_mDelay (const uint32_t msec) {
	USB_OTG_BSP_uDelay(msec * 1000);   
}


#ifdef USE_USB_OTG_FS

void OTG_FS_WKUP_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line18);
}

void OTG_FS_IRQHandler(void) {
	USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#else
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

void OTG_HS_WKUP_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line20);
}

void OTG_HS_IRQHandler(void) {
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void OTG_HS_EP1_IN_IRQHandler(void) {
	USBD_OTG_EP1IN_ISR_Handler(&USB_OTG_dev);
}

void OTG_HS_EP1_OUT_IRQHandler(void) {
	USBD_OTG_EP1OUT_ISR_Handler(&USB_OTG_dev);
}

#endif

