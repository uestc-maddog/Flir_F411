/*-----------------------------------------------------------------
 * Name:    electricity.c   
 * Purpose:  
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
#include "electricity.h"
#include "flir_menu.h"
#include "key.h"

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/
#define POWER_STANDBY_PIN			GPIO_PIN_11	// using PA-11 for standby control in F411	
#define POWER_STANDBY_GPIOX		GPIOA		

#define LCD_POWER_PIN					GPIO_PIN_2	// using PA-2 for LCD power pwm out put in F411
#define LCD_POWER_GPIOX				GPIOA				// during sleep, this pin set high to disable lcd power

#define FLIR_POWER_DWN_PIN		GPIO_PIN_8	// using PA-8 for flir power down pin
#define FLIR_POWER_DWN_GPIOX	GPIOA				// assert power_down to shutdown flir camera
/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
extern ADC_HandleTypeDef hadc1;

volatile Quan_baterry temp;
uint8_t baterrychackcounter=0;    // 电量检测计数器。
extern 	KeyStatus Key_Value;
bool low_power = false;
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
 * @fn        Get_Elec()
 *
 * @brief     get the current electricity of the battery  
 *
 * @param     None
 *
 * @return    Elec_Empty	Elec_Low	Elec_Med	Elec_High
 */
Quan_baterry Get_Elec(void)
{
	uint8_t i = 0, Sample_Times = 0;
	float ADC_Temp = 0;
	
	for(i = 0; i < 8; i++)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,2) == HAL_OK)
		{
			ADC_Temp += (double)HAL_ADC_GetValue(&hadc1);
			Sample_Times++;
		}
		HAL_ADC_Stop(&hadc1);
	}
	ADC_Temp = (float)(ADC_Temp * 3.44f / 0x0FFF / Sample_Times);

	if(ADC_Temp > Elec_Thre1)      return Baterry_full;
	else if(ADC_Temp > Elec_Thre2) return Baterry_high;
	else if(ADC_Temp > Elec_Thre3) return Baterry_middle;
	else if(ADC_Temp > Elec_Thre4) return Baterry_low;
	else if(ADC_Temp > Elec_Thre5) return Baterry_empty;
	else  																
	{
		low_power = true;
		setSandby();
	}
}

/*********************************************************************
 * @fn        CLOCK_OFF()
 *
 * @brief     关闭外设时钟，GPIOA外部中断未关
 *						
 *						
 *
 * @param     none
 *
 * @return    none
 */
void CLOCK_OFF(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
//	GPIOA->ODR &= 0x0fc8f;
//	GPIOB->ODR &= 0x0ffbf;
//	GPIOC->ODR &= 0x1fff;
	
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7| GPIO_PIN_8| GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7| GPIO_PIN_8| GPIO_PIN_9, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12| GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12| GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15, GPIO_PIN_RESET);
	
//		/*Configure GPIO pin : PB_STAT_Pin */
//  GPIO_InitStruct.Pin = PB_STAT_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(PB_STAT_GPIO_Port, &GPIO_InitStruct);
//	/* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	
	
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
	
  __HAL_RCC_DMA1_CLK_DISABLE();
	__HAL_RCC_DMA2_CLK_DISABLE();
	
	__HAL_RCC_ADC1_CLK_DISABLE();
	
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_SPI2_CLK_DISABLE();
	
	__HAL_RCC_I2C1_CLK_DISABLE();
	
	__HAL_RCC_TIM2_CLK_DISABLE();
	__HAL_RCC_TIM3_CLK_DISABLE();
	__HAL_RCC_TIM9_CLK_DISABLE();
}


/*********************************************************************
 * @fn        setSandby()
 *
 * @brief     configure system to standby mode,
 *						disable LCD and flir camera.
 *						call this function before sleep.
 *
 * @param     none
 * 
 * @return    none
 */
void setSandby( void )
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	// configure power standby pin 
//	// driven high this pin to enable low power
//	HAL_GPIO_WritePin(POWER_STANDBY_GPIOX, POWER_STANDBY_PIN, GPIO_PIN_SET);
//	
//	// lock this pin till next reset/power on
//	HAL_GPIO_LockPin(POWER_STANDBY_GPIOX, POWER_STANDBY_PIN);
	
	//sleep state changge to enable.
	sleep_sta = Sleep_enable;
	Save_Parameter();                           // 保存8个系统参数到FLASH
	if(low_power == false)
	display_Byebye();
	else
	display_PowerOff();	
	HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);HAL_Delay(500);
//	// configure LCD back light power
//	// re-config the LCD power pin
//	GPIO_InitStruct.Pin = LCD_POWER_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(LCD_POWER_GPIOX, &GPIO_InitStruct);
//	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   // LDO OFF
//	
//	// set this pin to high to stop LCD back light power
//	HAL_GPIO_WritePin(LCD_POWER_GPIOX, LCD_POWER_PIN, GPIO_PIN_RESET);
//	
//	// lock this pin till next reset/power on
//	HAL_GPIO_LockPin(LCD_POWER_GPIOX, LCD_POWER_PIN);
//	
//	
//	// congifure flir camera sleep
//	// enable flir power down pin to disable flir camera
//	HAL_GPIO_WritePin(FLIR_POWER_DWN_GPIOX, FLIR_POWER_DWN_PIN, GPIO_PIN_RESET);	// logic-low enable, shutdown sequence
//	
//	// lock this pin till next reset/power on
//	HAL_GPIO_LockPin(FLIR_POWER_DWN_GPIOX, FLIR_POWER_DWN_PIN);
	
	CLOCK_OFF();                                 // 关闭除外部唤醒中断的外设时钟
	
	/*To minimize the consumption In Stop mode, FLASH can be powered off before 
      entering the Stop mode using the HAL_PWREx_EnableFlashPowerDown() function.
      It can be switched on again by software after exiting the Stop mode using
      the HAL_PWREx_DisableFlashPowerDown() function. */
	//HAL_PWREx_EnableFlashPowerDown();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/*********************************************************************
 * @fn        setSandby()
 *
 * @brief     configure system to standby mode,
 *						disable LCD and flir camera.
 *						call this function before sleep.
 *
 * @param     none
 *
 * @return    none
 */
void setSandby2( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// configure power standby pin 
	// driven high this pin to enable low power
	HAL_GPIO_WritePin(POWER_STANDBY_GPIOX, POWER_STANDBY_PIN, GPIO_PIN_SET);
	

	
	// configure LCD back light power
	// re-config the LCD power pin
	GPIO_InitStruct.Pin = LCD_POWER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_POWER_GPIOX, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   // LDO OFF
	
	// set this pin to high to stop LCD back light power
	HAL_GPIO_WritePin(LCD_POWER_GPIOX, LCD_POWER_PIN, GPIO_PIN_RESET);
	
	
	
	// congifure flir camera sleep
	// enable flir power down pin to disable flir camera
	HAL_GPIO_WritePin(FLIR_POWER_DWN_GPIOX, FLIR_POWER_DWN_PIN, GPIO_PIN_RESET);	// logic-low enable, shutdown sequence
	

	
}


/*********************************************************************
 * @fn        resetStandby()
 *
 * @brief     Disable LTC3553-2 after system reset. Call this function
 *						first thing during init stage 
 *
 * @param     none
 *
 * @return    none
 */
void resetStandby( void )
{
	// configure power standby pin 
	// driven high this pin to enable low power
	HAL_GPIO_WritePin(POWER_STANDBY_GPIOX, GPIO_PIN_11, GPIO_PIN_RESET);
}

 /********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn       Baterrycheck
 *
 * @brief   check the quantity of baterry,charging,and is used to main.
 *
 * @param   no
 *
 * @return  no
 */
 
 void Baterrycheck(void)
 {
		baterrychackcounter ++;
		if(baterrychackcounter > 100)
		 {
			 baterrychackcounter = 0;
			 flir_conf.flir_sys_Baterry = Get_Elec();
		 }
 }


/*********************************************************************
 */
