#include "menufounction.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "flir_lcd.h"
#include "flir_compass.h"
#include "flir_menu.h"
#include "flir_focusing.h"
#include "electricity.h"
#include "key.h"
#include "flir_menu.h"

extern TIM_HandleTypeDef htim3;
extern uint8_t SleepTime_Setting;


extern void Flir_Display(void);
bool set_reticle_mark=false;
/**
* name Brightnesschosen
* founction when chosen the brightness seting,display the set menu.
*
**/
void Brightnesschosen(void)
{
	static BrightnessCont_sta Brightness_Old_sta = Level1;
	
	BrightnessCont_sta BGL_value = Brightness_Old_sta;
	KeyStatus Key_Value = Key_None;
	uint8_t timer = 0;

	display_Brightnessmenu(BGL_value);
	while(1)
	{
		HAL_Delay(50);
		Key_Value = Key_Scan();   
		if(Key_Value)
		{
			timer = 0;
			if(Key_Value == Key_Short)        // �̰��л��˵���
			{
				//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);		// ���� �̰���������
				if(++BGL_value == BGL_empty) BGL_value = Level1;
				display_Brightnessmenu(BGL_value);
			}
			if(Key_Value == Key_Long)
			{
				switch((int)BGL_value)         // �˵�����������
				{
							case (int)Level1:
								// �����û�����
								SET_BGLight(Level1);
								flir_conf.flir_sys_Bright = Level1;
								Brightness_Old_sta = Level1;
								display_Check(OP1_2_TH);
								break;
							case (int)Level2:
								// �����û�����
								SET_BGLight(Level2);
								flir_conf.flir_sys_Bright = Level2;
								Brightness_Old_sta = Level2;
								display_Check(OP2_2_TH);
								break;
							case (int)Level3:
								// �����û�����
								SET_BGLight(Level3);
								Brightness_Old_sta = Level3;
								flir_conf.flir_sys_Bright = Level3;
								display_Check(OP3_2_TH);
								break;
							case (int)Level4:
								// �����û�����
								SET_BGLight(Level4);
								Brightness_Old_sta = Level4;
								flir_conf.flir_sys_Bright = Level4;
								display_Check(OP4_2_TH);
								break;

							case (int)BGL_Exit:
								timer = 200;                          // timer=200ʱ���˳��˵�����
								break;
				}
				//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);   // ���� ������������
			}									
		}
		if(timer++ == 200)                            // ����10s�ް�����Ӧ�����˳��˵�����
		{
			timer = 0;
			break;                      
		}
	}
}

/**
* name Sleepchosen
* founction when chosen the sleep time seting,display the set menu.
*
**/
void Sleepchosen(void)
{
	static SleepCont_sta Sleep_Old_sta = Minutes_3;
	
	SleepCont_sta SLP_value = Sleep_Old_sta;
	KeyStatus Key_Value = Key_None;
	uint8_t timer = 0;

	display_Sleepmenu(SLP_value);
	while(1)
	{
		HAL_Delay(50);
		Key_Value = Key_Scan();   
		if(Key_Value)
		{
			timer = 0;
			if(Key_Value == Key_Short)        // �̰��л��˵���
			{
				//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);		// ���� �̰���������
				if(++SLP_value == SLP_empty) SLP_value = Minutes_3;
				display_Sleepmenu(SLP_value);
			}
			if(Key_Value == Key_Long)
			{
				switch((int)SLP_value)         // �˵�����������
				{
					case (int)Minutes_3:
						// �����û�����
						Time_Sleep = 0;
						SleepTime_Setting = Time_Minu3;
						flir_conf.flir_sys_Sleep = Time_Minu3;
						Sleep_Old_sta = Minutes_3;           // ��������״̬
						display_Check(OP1_2_TH);
						break;
					case (int)Minutes_5:
						// �����û�����
						Time_Sleep = 0;
						SleepTime_Setting = Time_Minu5;
						flir_conf.flir_sys_Sleep = Time_Minu5;
						Sleep_Old_sta = Minutes_5;           // ��������״̬
						display_Check(OP2_2_TH);
						break;
					case (int)Minutes_10:
						// �����û�����
						Time_Sleep = 0;
						SleepTime_Setting = Time_Minu10;
						flir_conf.flir_sys_Sleep = Time_Minu10;
						Sleep_Old_sta = Minutes_10;          // ��������״̬
						display_Check(OP3_2_TH);
						break;
					case (int)Minutes_15:
						// �����û�����
						Time_Sleep = 0;
						SleepTime_Setting = Time_Minu15;
						flir_conf.flir_sys_Sleep = Time_Minu15;
						Sleep_Old_sta = Minutes_15;          // ��������״̬
						display_Check(OP4_2_TH);
						break;

					case (int)SLP_Exit:
						timer = 200;                         // timer=200ʱ���˳��˵�����
						break;
				}
				//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);   // ���� ������������
			}									
		}
		if(timer++ == 200)                            // ����10s�ް�����Ӧ�����˳��˵�����
		{
			timer = 0;
			break;                      
		}
	}
}

/**
* name Versionchosen
* founction when chosen the vertion seting,display the Version message.
*
**/
void Versionchosen(void)
{
	uint8_t timer = 0;

	display_Versionmenu();
	while(1)
	{
		HAL_Delay(100);
		if(timer++ == 30)                            // ����3s�ް�����Ӧ�����˳��˵�����
		{
			timer = 0;
			break;                      
		}
	}
}

/**
* name set_reticle
* founction when chosen the reticle seting,to here.
*
**/
flir_reticle_sta reticle_sta=reticle_able;
void set_reticle(void)
{
	reticle_sta=reticle_able;
	uint16_t timer = 0;
	while(1)
	{
		set_reticle_mark=true;    //�����˵���־����Ϊtrue
		Flir_Display();	       // Flir����
		KeyStatus Key_Value = Key_None;
		Key_Value = Key_Scan();                
		if(Key_Value)
		{
			timer = 0;
			if(Key_Value == Key_Short)           // �̰��л�display mode
			{
				switch((int)reticle_sta)
				{
					case reticle_able:
						if(flir_conf.flir_sys_Focus == focus_enable) flir_conf.flir_sys_Focus = focus_disable;   // focus����״̬��ת
						else                                         flir_conf.flir_sys_Focus = focus_enable;					
						//Focus_en = !Focus_en;
					break;
					case reticle_Hor:
						Hor++;
						if(Hor>18)
							Hor=0;
						flir_conf.flir_sys_Reticle[0]=Hor*4-36;
					break;
					case reticle_Ver:
						Ver++;
						if(Ver>18)
							Ver=0;
						flir_conf.flir_sys_Reticle[1]=Ver*6-54;
					break;
					case reticle_back:
						timer=4000;
					break;
					default :
						timer=4000;
					break;
				}
			}
			if(Key_Value == Key_Long)            // ��������˵�����
			{
				if(++reticle_sta == reticle_empty) reticle_sta = reticle_able;
			}
			
		}
		if(timer++ >= 4000)                            // ����10s�ް�����Ӧ�����˳��˵�����
		{
			timer = 0;
			break;                      
		}
		
	}
	set_reticle_mark=false;
}
