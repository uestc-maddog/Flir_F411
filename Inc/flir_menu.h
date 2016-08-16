/*-----------------------------------------------------------------
 * Name:      flir_menu.h
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
 #ifndef FLIR_MENU_H_
 #define FLIR_MENU_H_
 
/********************************************************************************************************
 *                                               INCLUDES
 ********************************************************************************************************/
/* System ralted */
#include "stm32f4xx_hal.h"
#include <string.h> 	
#include "stdint.h"
#include <stdbool.h>
#include <math.h>

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/
 #define SCREEN_SET_BUFSIZE	320
#define FLIR_VERSION 1

// menu color display related
#define OP1_TH		130
#define OP1_2_TH	118
#define OP2_2_TH	103
#define OP3_2_TH	86
#define OP4_2_TH	73
#define OP5_2_TH	58
#define OP6_2_TH	45
#define OP6_TH		20

//#define BGL1_TH   99
//#define BGL1_2_TH 86
//#define BGL2_2_TH 70
//#define BGL3_2_TH 54
//#define BGL4_2_TH 38
//#define BGL5_TH   20
#define SELECT_COLOR	YELLOW

#define MENU_CONTENT	     7
#define BRIGHTNESS_CONTENT 5
#define SLEEP_CONTENT      6
#define CH_CONTENT         4
#define CH1_TH		114
#define CH1_2_TH	88
#define CH2_2_TH	62
#define CH3_2_TH	40
 
// menu chosen status
typedef enum {
	Brightness = 0,
	Sleep,
	Reticle,
	Version,
	Reset,
	Compass,
	Exit,
	empty,
} menuCont_sta;

// Brightness chosen status

typedef enum {
	Level1 = 0,
	Level2,
	Level3,
	Level4,
	BGL_Exit,
	BGL_empty,
} BrightnessCont_sta;

typedef enum {
	Not_LowPower = 0,
	Is_LowPower,
} LowPower_sta;

typedef enum {
	Minutes_3 = 0,
	Minutes_5,
	Minutes_10,
	Minutes_15,
	Minutes_NA,
	SLP_Exit,
	SLP_empty,
} SleepCont_sta;

// sleepmode conditions

typedef enum {
	Sleep_enable,       // Sleep
	Sleep_disable,      // Run
} Sleep_sta;

//sleep state 
typedef enum {
	reticle_able = 0,
	reticle_Hor,
	reticle_Ver,
	reticle_back,
	reticle_empty,
} flir_reticle_sta;


// compassmode conditions

typedef enum {
	disable = 0,
	enable,
} CompassMode_sta;

// display mode conditions
typedef enum {
	greyscale = 0,
	color,
} DisplayMode_sta;

typedef enum {
	focus_disable = 0,
	focus_enable,
} FocusCont_sta;  		//标志focus使能状态

//baterry display mode

typedef enum {
	normal = 0,
	charging,}Baterrymode; 
// quantity of betarry
typedef enum {
	Baterry_high = 0,
	Baterry_middle,
	Baterry_low,
	Baterry_empty,
	Baterry_full,}Quan_baterry;

typedef struct{
	BrightnessCont_sta flir_sys_Bright;           // 系统亮度参数
	SleepCont_sta      flir_sys_Sleep;            // 系统休眠时间参数
	FocusCont_sta      flir_sys_Focus;            // 系统对焦准心开关
	int                flir_sys_Reticle[2];       // 系统Reticle参数,分别存储X、Y轴的偏移量
	DisplayMode_sta    flir_sys_DisMode;          // 系统显示模式参数
	CompassMode_sta    flir_sys_ComMode;          // 系统Compass模式参数
	Quan_baterry       flir_sys_Baterry;          // 系统电量参数
	Baterrymode        file_sys_chargingMode;			// 系统充电状态参数
	LowPower_sta	     file_sys_LowPower;         // 标记当前系统是否处于Stop模式	
} sysConf_t;
/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
extern sysConf_t flir_conf;

extern Sleep_sta sleep_sta;
 
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
 * @fn      display_Boot_UI
 *
 * @brief   display Boot_Interface
 *
 * @param   none
 *
 * @return  
 */
bool display_Boot_UI(void);
	
/*********************************************************************
 * @fn      display_menu
 *
 * @brief   
 *
 * @param   uint16_t pdata[][80] - screen buffer
 *
 * @return  
 */
bool display_menu(menuCont_sta Current_Sta);
/*********************************************************************
 * @fn      display_Brightness_menu
 *
 * @brief   
 *
 * @param   uint16_t pdata[][80] - screen buffer
 *
 * @return  
 */
bool display_Brightnessmenu(BrightnessCont_sta Current_Sta);

/*********************************************************************
 * @fn      display_Sleepmenu
 *
 * @brief   
 *
 * @param   uint16_t pdata[][80] - screen buffer
 *
 * @return  
 */
bool display_Sleepmenu(SleepCont_sta Current_Sta);

/*********************************************************************
 * @fn      display_Versionmenu
 *
 * @brief   display Version Message
 *
 * @param   none
 *
 * @return  
 */
bool display_Versionmenu(void);

/*********************************************************************
 * @fn      display Check
 *
 * @brief   display Check 符号
 *
 * @param   none
 *
 * @return  
 */
bool display_Check(uint8_t Offset);

/*********************************************************************
 * @fn      display_PowerOff
 *
 * @brief   display low power off
 *
 * @param   none
 *
 * @return  
 */
bool display_PowerOff(void);

/*********************************************************************
 * @fn      display_Byebye
 *
 * @brief   关机界面 
 *
 * @param   none
 *
 * @return  
 */
bool display_Byebye(void);

/*********************************************************************
 * @fn      sysConf_init
 *
 * @brief   init system configuration
 *
 * @param   none
 *
 * @return  
 */
void sysConf_init(void);
/*********************************************************************
 * @fn      sysConf_Reset
 *
 * @brief   reset system configuration
 *
 * @param   none
 *
 * @return  
 */
void sysConf_Reset(void);
void Save_Parameter(void);                           // 保存8个系统参数到FLASH
void Addbaterry_menu(Baterrymode mode,Quan_baterry value);

bool display_charging(Quan_baterry value);

void display_normal_baterry(Quan_baterry value);

bool display_Boot_Animation(uint8_t x);

void display_Animation(void);

void display_sleep_charging(Quan_baterry value);


#endif
/*********************************************************************
 */


