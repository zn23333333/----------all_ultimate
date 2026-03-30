/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  */

#include "stm32f4xx_it.h"
#include "main.h"

/* ====== FreeRTOS: 需要把 3 个异常入口映射到 port 层 ======
   注意：这三个函数在 FreeRTOS/portable 里实现（port.c 等）
*/
#include "FreeRTOS.h"
#include "task.h"

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}



//以下三个函数在移植freertos时，要被freertos接管

//void SVC_Handler(void)
//{
//  /* FreeRTOS uses SVC for starting the first task */
//  vPortSVCHandler();
//}

//void PendSV_Handler(void)
//{
//  /* FreeRTOS uses PendSV for context switching */
//  xPortPendSVHandler();
//}

//void SysTick_Handler(void)
//{
//  /* FreeRTOS tick */
//  xPortSysTickHandler();
//}



void DebugMon_Handler(void)
{
}





/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/******************************************************************************/
/* 在这里添加外设中断即可 */
