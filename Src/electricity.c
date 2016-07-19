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
uint8_t baterrychackcounter=0;    //一个电量检测计数器。
extern 	KeyStatus Key_Value;
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
	ADC_Temp = (float)(ADC_Temp * 3.30f / 0x0FFF / Sample_Times);
	if(ADC_Temp > Elec_Thre1)      return Baterry_high;
	else if(ADC_Temp > Elec_Thre2) return Baterry_middle;
	else if(ADC_Temp > Elec_Thre3) return Baterry_low;
	else                            return Baterry_empty;
}


/*********************************************************************
 * @fn        standbyPower()
 *
 * @brief     configure LTC3553-2 to standby mode,
 *						disable LCD and flir camera.
 *						call this function before sleep.
 *
 * @param     none
 *
 * @return    none
 */
void setSandby( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// configure power standby pin 
	// driven high this pin to enable low power
	HAL_GPIO_WritePin(POWER_STANDBY_GPIOX, POWER_STANDBY_PIN, GPIO_PIN_SET);
	
	// lock this pin till next reset/power on
	HAL_GPIO_LockPin(POWER_STANDBY_GPIOX, POWER_STANDBY_PIN);
	
	
	// configure LCD back light power
	// re-config the LCD power pin
	GPIO_InitStruct.Pin = LCD_POWER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_POWER_GPIOX, &GPIO_InitStruct);
	
	// set this pin to high to stop LCD back light power
	HAL_GPIO_WritePin(LCD_POWER_GPIOX, LCD_POWER_PIN, GPIO_PIN_SET);
	
	// lock this pin till next reset/power on
	HAL_GPIO_LockPin(LCD_POWER_GPIOX, LCD_POWER_PIN);
	
	
	// congifure flir camera sleep
	// enable flir power down pin to disable flir camera
	HAL_GPIO_WritePin(FLIR_POWER_DWN_GPIOX, FLIR_POWER_DWN_PIN, GPIO_PIN_RESET);	// logic-low enable, shutdown sequence
	
	// lock this pin till next reset/power on
	HAL_GPIO_LockPin(FLIR_POWER_DWN_GPIOX, FLIR_POWER_DWN_PIN);
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
