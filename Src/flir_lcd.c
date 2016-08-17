/*-----------------------------------------------------------------
 * Name:      
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
/* System ralted */
#include "stm32f4xx_hal.h"
#include <string.h> 	
#include "stdlib.h"
#include <math.h>
/* Special requirement */
#include "flir_lcd.h"

#include "flir_focusing.h"
#include "flir_compass.h"
#include "electricity.h"
#include "menufounction.h"
//#include "font.h"  

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/
// Basic LCD information 
_lcd_dev lcddev; 
volatile bool lcdTXcpl = true; // initialize transmit complete as true

/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
// SPI1 handler variable, delare in main.c
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef LCD_SPI_PORT;
extern DMA_HandleTypeDef LCD_DMA_PORT;
extern bool Compass_Swit;        // 0---compass on   1---compass off
extern uint8_t Charge_Flag ;

/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               LOCAL VARIABLES
 ********************************************************************************************************/
// Point color
static uint16_t POINT_COLOR=0x0000;	
// Background color
static uint16_t BACK_COLOR=0xFFFF;  

// buffer used to clear screen
static uint16_t screenClear[LCD_CLEAR_BUF_SIZ];

#ifdef FLIR_PROJ
// row data buffer, save for display
uint16_t rowBuf[FLIR_LCD_RAW][FLIR_LCD_COLUMNUM];
#endif

/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/
static void LCD_WR_REG(uint16_t);
static void LCD_WR_DATA(uint16_t);
static void LCD_WR_DATA8(uint8_t );
static void LCD_WR_REG_DATA(uint8_t , uint16_t );
void LCD_WriteRAM_Prepare(void);
static void LCD_startDisplay( uint16_t Xpos, uint16_t Ypos );
extern TIM_HandleTypeDef htim9;

/********************************************************************************************************
 *                                               PUBLIC FUNCTIONS
 ********************************************************************************************************/
 
/*********************************************************************
 * @fn      LCD_SetCursor
 *
 * @brief   Set cursor position. The size is 160*128.
 *          And the top left is the starting address, (0,0)
 *
 *			X axis is the long side, Y is the short side.
 *
 * @param   uint16_t Xpos - X position. 
 *			uint16_t Ypos - Y position.
 *
 * @return  none
 */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA8(Xpos>>8); 
	LCD_WR_DATA8(Xpos&0XFF);	 
	Xpos = 159;
	LCD_WR_DATA8(Xpos>>8); 
	LCD_WR_DATA8(Xpos&0XFF);	
	
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA8(Ypos>>8); 
	LCD_WR_DATA8(Ypos&0XFF);
	Ypos = 127;
	LCD_WR_DATA8(Ypos>>8); 
	LCD_WR_DATA8(Ypos&0XFF);	
} 	

/*********************************************************************
 * @fn      LCD_DrawPoint
 *
 * @brief   Draw a single point with x.y position provide.
 *
 *			X axis is the long side, Y is the short side.
 *
 * @param   uint16_t x - X position. 
 *					uint16_t y - Y position.
 *					uint16_t color - point color
 *
 * @return  none
 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	// set cursor location
	LCD_SetCursor(x,y);		 
	// start writing command
	LCD_WriteRAM_Prepare();	
	LCD_WR_DATA(color); 
} 

/*********************************************************************
 * @fn      LCD_Init
 *
 * @brief   Init LCD. Set basic information about the LCD
 *
 *			- Set color mode 65k
 *			- Revert X,Y axis
 *			- Set Frame Rate and power settings.
 *
 * @param   none
 *
 * @return  none
 */
void LCD_Init(void)
{ 	 	
	// Power on sequence first, HW/SW reset
	LCD_REST=0;		 
 	HAL_Delay(50); // delay 20 ms 
	LCD_REST=1;		 
 	HAL_Delay(50); // delay 20 ms 

	SPILCD_RST_RESET ;	//LCD_RST=0	 //SPI接口复位
	HAL_Delay(20); // delay 20 ms 
	SPILCD_RST_SET ;	//LCD_RST=1		
	HAL_Delay(20);

	lcddev.width=128;
	lcddev.height=160;
	lcddev.wramcmd=0X2C;
	lcddev.setxcmd=0X2A;
	lcddev.setycmd=0X2B; 	
	
	//Sleep out
	LCD_WR_REG(0x11);
	HAL_Delay(120); //Delay 120ms
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	//------------------------------------End ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB4); //Dot inversion
	LCD_WR_DATA8(0x00); // 0x03
	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0XC0);
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x2A);
	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);
	//---------------------------------End ST7735S Power Sequence-------------------------------------//
	LCD_WR_REG(0xC5); //VCOM
	LCD_WR_DATA8(0x1A);
	LCD_WR_REG(0x36); //MX, MY, RGB mode
	LCD_WR_DATA8(0xA0); // invert raw/column, invert horizeon
	//------------------------------------ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x22);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2A);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);
	//------------------------------------End ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0x3A); //65k mode
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0x29); //Display on

	// reset to white screen
	LCD_Clear(WHITE); 
	
#ifdef FLIR_PROJ
	// print the black area in the top and bottom
	// prepare black color
	uint8_t flir_txBuf[640];
	memset(flir_txBuf, BLACK, 640);
	
	// start from (0,0), set the top 4 raw pixels as black
	LCD_SetCursor(0, 0);
	// prepare to write
	LCD_WriteRAM_Prepare();    
	// set ready for graphic transmit
#ifndef FLIR_PROJ
	// flir project use hardware nss
	SPILCD_CS_RESET;
#endif  
	SPILCD_RS_SET;	
	
	// first 2 raw
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)flir_txBuf, 640);
	// polling for 10ms by defaul
	HAL_DMA_PollForTransfer(&LCD_DMA_PORT,HAL_DMA_FULL_TRANSFER,10);
	// next 2 raw
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)flir_txBuf, 640);
	// polling for 10ms by defaul
	HAL_DMA_PollForTransfer(&LCD_DMA_PORT,HAL_DMA_FULL_TRANSFER,10);	

	
	// set cursor again to print the last 4 raw pixels as black
	LCD_SetCursor(0, 124);
	// prepare to write
	LCD_WriteRAM_Prepare();    
	// set ready for graphic transmit
#ifndef FLIR_PROJ
	// flir project use hardware nss	
	SPILCD_CS_RESET;  
#endif
	SPILCD_RS_SET;	
	// first 2 raw
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)flir_txBuf, 640);
	// polling for 10ms by defaul
	HAL_DMA_PollForTransfer(&LCD_DMA_PORT,HAL_DMA_FULL_TRANSFER,10);
	// next 2 raw
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)flir_txBuf, 640);
	// polling for 10ms by defaul
	HAL_DMA_PollForTransfer(&LCD_DMA_PORT,HAL_DMA_FULL_TRANSFER,10);	
#endif
}  


/*********************************************************************
 * @fn      LCD_Clear
 *
 * @brief   Clear LCD. Set screen to sigle color. 
 *
 * @param   n
 *
 * @return  none
 */
void LCD_Clear(uint16_t color)
{
	uint32_t index=0;      
	
	// set cousor
	LCD_SetCursor(0,0);
	
	// prepare to write
	LCD_WriteRAM_Prepare();     
	
	for(index = 0; index < LCD_CLEAR_BUF_SIZ; index ++)
	{
		screenClear[index] = color;
	}
	
	// LCD_CS=0
#ifndef FLIR_PROJ
	// flir project use hardware nss	
	SPILCD_CS_RESET;  
#endif
	SPILCD_RS_SET;	
	
	// block sending
	for(index = 0; index < LCD_CLEAR_ROUND; index ++)
	{
		HAL_SPI_Transmit(&LCD_SPI_PORT, (uint8_t*)screenClear, LCD_CLEAR_BUF_SIZ, 100);	
	}

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	// LCD_CS=1		
	SPILCD_CS_SET;
#endif  
}  

#ifdef FLIR_PROJ
/*********************************************************************
 * @fn      LCD_WR_Frame
 *
 * @brief   Display a whole frame data
 *
 * @param   uint16_t * pdata - data pointer
 *
 * @return  transmit status
 */
bool LCD_WR_Frame(volatile uint16_t pdata[][80])
{
	uint16_t i,j;
	static uint16_t Angle = 0;
	
	if (lcdTXcpl)
	{
		// set complete flag invalid
		lcdTXcpl = false;
		
		for(i = 0; i < FLIR_LINE; i++)     // 摄像头数据
		{
			for(j = 0; j < FLIR_COLUMNUM; j++)
			{
				// get the buffer data correct	
				rowBuf[2*i][2*j] = pdata[i][j];
				rowBuf[2*i+1][2*j+1] = pdata[i][j];
				rowBuf[2*i][2*j+1] = pdata[i][j];
				rowBuf[2*i+1][2*j] = pdata[i][j];
			}
		}

		if(set_reticle_mark==true)              //如果这是二级菜单的显示，则显示瞄准调试界面
			setreticle_display();
		if(flir_conf.flir_sys_Focus == focus_enable)
			Add_focusing(flir_conf.flir_sys_Reticle);                         // add focusing lines data
		if(flir_conf.flir_sys_ComMode == enable)
		{
			Angle = angle;
			Add_compass(Angle);                     // add compass data         偏移temp行
		}
		if((GPIOA->IDR&0x8000))                     // jugge the charging mark
		{
			flir_conf.file_sys_chargingMode = normal;
			Charge_Flag = 0;          
		}
		else
		{
			flir_conf.file_sys_chargingMode = charging;
			Charge_Flag = 1;  		
		}
		Addbaterry_menu(flir_conf.file_sys_chargingMode,flir_conf.flir_sys_Baterry);          //添加电池图标。
	
		// if previous transmit finish, transmit new frame
		// set the start cursor first
		LCD_startDisplay(0, 4); // start from the 4th row, skip the black frame
		
		// use DMA to display a new frame
		/* Enable the TIM Update interrupt */
		HAL_TIM_Base_Stop_IT(&htim2);
//		HAL_TIM_Base_Stop_IT(&htim3);
		HAL_SPI_Transmit_DMA(&LCD_SPI_PORT, (uint8_t*)rowBuf, sizeof(rowBuf));
		HAL_TIM_Base_Start_IT(&htim2);
//		HAL_TIM_Base_Start_IT(&htim3);
		
		// return true
		return true;
	}
	return false;
}
#endif


/*********************************************************************
 * @fn      LCD_drawLine
 *
 * @brief   draw a line 
 *
 * @param   uint16_t x1	- point1 x location
 *					uint16_t y1 - point1 y location
 *					uint16_t x2 - point2 x location
 *					uint16_t y2 - point2 y location
 *					uint16_t color - line color
 *					
 * @return  none
 */
void LCD_drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	
	if(delta_x>0)
		incx=1; //设置单步方向 
	else if(delta_x==0)
		incx=0;//垂直线 
	else 
	{
		incx=-1;
		delta_x=-delta_x;
	} 
	
	if(delta_y>0)
		incy=1; 
	else if(delta_y==0)
		incy=0;//水平线 
	else
	{
		incy=-1;
		delta_y=-delta_y;
	} 
	
	if(delta_x>delta_y) //选取基本增量坐标轴 
		distance=delta_x; 
	else 
		distance=delta_y; 
	
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol,color);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}   


/*
bool LCD_WR_compass(uint16_t pdata[][80],uint16_t jiaodu,uint8_t x,uint8_t y)
{
	uint8_t i,j,m,n,p,q;

	for(j=0;j<5;j++)
	{
		for(i=0;i<(11-2*j);i++)
		{
			m=cos((double)jiaodu)*i+0.5;
			p=j*sin((double)jiaodu)+0.5;
			n=sin((double)jiaodu)*i+0.5;
			q=j*cos((double)jiaodu)+0.5;
			pdata[x+m-p][y+n+q]=0x1F00;
			pdata[x-m-p][y-n+q]=0xE0FF;
			pdata[x+m+p][y+n-q]=0x1F00;
			pdata[x-m+p][y-n-q]=0xE0FF;
		}

	//		pdata[x+i][y-j]=	pdata[x+i][y-j]+80;
	}
}*/

/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      LCD_WR_REG
 *
 * @brief   Send one byte register address, blocking send.
 *
 * @param   uint16_t regval - the address
 *
 * @return  none
 */
static void LCD_WR_REG(uint16_t regval)
{ 
	uint8_t temp;

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	// LCD_CS=0
	SPILCD_CS_RESET;  
#endif
	// Reset RS to indicate a register writing.
	SPILCD_RS_RESET;

	// send information
	temp = regval&0x00FF;
	HAL_SPI_Transmit(&LCD_SPI_PORT, &temp, 1 , 20);

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	//LCD_CS=1
	SPILCD_CS_SET;  	   		 
#endif	
}


/*********************************************************************
 * @fn      LCD_WR_DATA
 *
 * @brief   Write two bytes data address, blocking send.
 *
 * @param   uint16_t data - the data
 *
 * @return  none
 */
static void LCD_WR_DATA(uint16_t data)
{
#ifndef FLIR_PROJ
	// flir project use hardware nss	
	//LCD_CS=0
 	SPILCD_CS_RESET;
#endif	
	// Set RS to indicate a data transmit
	SPILCD_RS_SET;	
	
	// send data
	HAL_SPI_Transmit(&LCD_SPI_PORT, (uint8_t*)&data, 2, 20);

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	//LCD_CS=1
	SPILCD_CS_SET; 		
#endif
}

/*********************************************************************
 * @fn      LCD_WR_DATA8
 *
 * @brief   Write one byte data address, blocking send.
 *
 * @param   uint16_t da - the data
 *
 * @return  none
 */
static void LCD_WR_DATA8(uint8_t da)   
{
	//LCD_CS=0
#ifndef FLIR_PROJ
	// flir project use hardware nss	
 	SPILCD_CS_RESET;
#endif
	// Set RS to indicate a data transmit
	SPILCD_RS_SET;	

	// send data
	HAL_SPI_Transmit(&LCD_SPI_PORT, &da, 1 , 20);

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	//LCD_CS=1  	
	SPILCD_CS_SET;   			 
#endif
}	

/*********************************************************************
 * @fn      LCD_WR_DATA8
 *
 * @brief   Write two bytes data to an register address, blocking send.
 *
 * @param   uint16_t LCD_RegValue - the data
 *	        uint8_t LCD_Reg - the address
 *
 * @return  none
 */
static void LCD_WR_REG_DATA(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*********************************************************************
 * @fn      LCD_WriteRAM_Prepare
 *
 * @brief   Prepare to send graphic information to GRam.
 *			The graphic data should be transmit after this command. 
 *
 * @param   none
 *
 * @return  none
 */
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);  
}	

/*********************************************************************
 * @fn      LCD_startDisplay
 *
 * @brief   set parameters to start a display transmit. The function 
 *					should followed by flir_display function to transmit data
 *					At last flir_endDisplay should be call to end an transmision.
 *
 * @param   uint16_t Xpos -> X start position for this tranmission
 *					uint16_t Ypos -> Y start position for this tranmission
 *
 * @return  none
 */
static void LCD_startDisplay( uint16_t Xpos, uint16_t Ypos )
{
	// set cousor
	LCD_SetCursor(Xpos, Ypos);
	
	// prepare to write
	LCD_WriteRAM_Prepare();    

#ifndef FLIR_PROJ
	// flir project use hardware nss	
	// set ready for graphic transmit
	SPILCD_CS_RESET;  
#endif	
	SPILCD_RS_SET;	
}


/* background light levels set 0-100 */
void SET_BGLight(BrightnessCont_sta levels)
{

  TIM_OC_InitTypeDef sConfigOC;


  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	switch((int)levels)         // ²Ëµ¥À¸¶þ¼¶¹¦ÄÜ
	{
				case (int)Level1:
					
					sConfigOC.Pulse = 800;
					break;
				case (int)Level2:
					sConfigOC.Pulse = 600;
					break;
				case (int)Level3:
					sConfigOC.Pulse = 300;
					break;
				case (int)Level4:
					sConfigOC.Pulse = 0;
					break;
				default : 
					sConfigOC.Pulse = 0;
					break;
	}
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
}

/*********************************************************************
 */

