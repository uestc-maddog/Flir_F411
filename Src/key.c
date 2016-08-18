/*-----------------------------------------------------------------
 * Name:    key.c   
 * Purpose: key driver  
 *-----------------------------------------------------------------
 * 
 * Copyright (c) *reserve
 
||                       _      _               ||
||    /\  /\  __  _  __ | | __ | | ____  ___ _  ||
||   /  \/  \/ _ ` |/ _ ` |/ _ ` |/ _  \/ _ ` | ||
||  / /\  /\  (_|    (_|    (_|    (_)   (_)  | ||
|| /_/  \/  \_.__, |\__, _|\__, _|\____/\___. | ||
|| =====================================|____/  ||
||                                              ||

 -----------------------------------------------------------------*/
 
/********************************************************************************************************
 *                                               INCLUDES
 ********************************************************************************************************/
#include "stm32f4xx_hal.h"
#include "key.h"
#include "electricity.h"
#include "flir_lcd.h"
/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/
uint8_t  Key_Up = 0;
uint8_t  Key_Down = 0;
uint32_t Time_1ms = 0;                // 按键时间捕获
bool return_mark = false;
volatile uint8_t Time_Sleep = 0;      // Sleep Time counter

/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t Charge_Flag;
 
/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               LOCAL VARIABLES
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               PUBLIC FUNCTIONS
 ********************************************************************************************************/
 
/*********************************************************************
 * @fn        Key_Scan()
 *
 * @brief     按键扫描驱动 
 *
 * @param     None
 *
 * @return    Key_None：按键无效   Key_Short：按键短按   Key_Long：按键长按
 */
KeyStatus Key_Scan(void)
{
	if(Key_Down == 1)
	{
		if(Time_1ms > Long_Thre && return_mark == false)       
		{
			return_mark = true;
			//Time_1ms = 0;          // 启动定时器前已清零
			return Key_Long;         // 按键按下时间大于2s      长按
		}
	}
	if(Key_Up == 1)        // 有按键松起
	{
		Key_Up = 0;
		Key_Down = 0;
//		if(Time_1ms > Long_Thre)       
//		{
//			//Time_1ms = 0;          // 启动定时器前已清零
//			return Key_Long;         // 按键按下时间大于2s      长按
//		}
//		else 
		if(Time_1ms > Short_Thre && return_mark == false) 
		{
			//Time_1ms = 0;
			return Key_Short;        // 按键按下时间小于2s      短按 
		}		
		else if(Time_1ms > Long_Thre && return_mark == false) 
		{
			//Time_1ms = 0;
			return Key_Long;        // 按键按下时间da于2s        长按 
		}		
		else                           
		{
			return_mark = false;
			//Time_1ms = 0;
			return Key_None;         // 视作按键抖动
		}
	}
	return Key_None;
}
/*********************************************************************
 * @fn        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)()
 *
 * @brief     TIM2 detection callbacks.
 *
 * @param     htim：所用定时器结构体   
 *
 * @return    None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	if(htim->Instance == TIM2) Time_1ms++; 
	if(htim->Instance == TIM3) 
	{
		Time_Sleep++;
		//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	}
}
/*********************************************************************
 * @fn        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)()
 *
 * @brief     EXTI line detection callbacks.
 *
 * @param     GPIO_Pin: Specifies the pins connected EXTI line  
 *
 * @return    None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t CAPTURE_STA = 0;	   // 捕获状态
	static uint8_t Temp = 0;	       
	
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
		/* NOTE: This function Should not be modified, when the callback is needed,
						 the HAL_GPIO_EXTI_Callback could be implemented in the user file
		 */
	if(GPIO_Pin == GPIO_PIN_0)           // PBSTAT中断
	{
		Temp++;
		if(flir_conf.file_sys_LowPower == Not_LowPower) 
		{
			flir_conf.file_sys_LowPower = Is_LowPower;        // 状态切换
			setSandby();                     // 进入低功耗模式
		}
		else
		{
			flir_conf.file_sys_LowPower = Not_LowPower;       // 状态切换
			
			/*  SoftReset  */
			HAL_PWREx_DisableFlashPowerDown();
			__set_FAULTMASK(1);                               // 关闭所有中断
			NVIC_SystemReset();                               // 软件复位
		}
	}
	if(GPIO_Pin == GPIO_PIN_12)          // 按键中断
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		
		if(CAPTURE_STA == 1)               // 已经捕获到按键按下
		{
			GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
			HAL_TIM_Base_Stop_IT(&htim2);    // 按键按下，关闭定时器TIM2
			Key_Up = 1;
			CAPTURE_STA = 0;
		}
		else                               // 还未捕获到按键按下
		{
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;   
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
			Key_Down = 1;
			Time_1ms = 0;
			HAL_TIM_Base_Start_IT(&htim2);   // 按键按下，开启定时器TIM2
			CAPTURE_STA = 1;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_15)     // 充电中断
	{
		if(!(GPIOA->IDR&0x8000))           // 下降沿，进入充电
		{
			if(flir_conf.file_sys_LowPower == Not_LowPower)  // 非Stop模式，进入充电模式
			{
				flir_conf.file_sys_chargingMode = charging;
			}
			else
			{
				flir_conf.file_sys_LowPower = Not_LowPower;       // 状态切换

				/*  SoftReset  */
				HAL_PWREx_DisableFlashPowerDown();
				__set_FAULTMASK(1);                               // 关闭所有中断
				NVIC_SystemReset();                               // 软件复位
			}
		}
		else                                // 上降沿，退出充电
		{
			flir_conf.file_sys_chargingMode = normal; 
			Charge_Flag = 0;
		}
	}
}
 /********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 */
 
 


/*********************************************************************
 */
