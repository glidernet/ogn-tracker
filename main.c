/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "console.h"
#include "gps.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void prvSetupHardware(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
   /* Configure PA5 (LED) in output pushpull mode */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   Console_Config();
   GPS_Config();
}

void vTaskPulse(void* pvParameters)
{
   for(;;)
   {
      GPIO_SetBits(GPIOA, GPIO_Pin_5);
      vTaskDelay(1000);
      GPIO_ResetBits(GPIOA, GPIO_Pin_5);
      vTaskDelay(1000);
   }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 
   prvSetupHardware();

   xTaskCreate(vTaskPulse,     (char *)"LED",     256,  NULL, tskIDLE_PRIORITY+1, NULL);
   xTaskCreate(vTaskConsole,   (char *)"Console", 1024, NULL, tskIDLE_PRIORITY+2, NULL);
   xTaskCreate(vTaskGPS,       (char *)"GPS",     1024, NULL, tskIDLE_PRIORITY+3, NULL);

	vTaskStartScheduler();
	return 0;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
