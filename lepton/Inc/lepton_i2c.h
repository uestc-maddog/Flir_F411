#ifndef LEPTON_I2C_H_
#define LEPTON_I2C_H_

#include "LEPTON_ErrorCodes.h"
#include "LEPTON_VID.h"
#include "LEPTON_Types.h"
#include "stm32f4xx.h"

HAL_StatusTypeDef agc_enable(void);
HAL_StatusTypeDef set_reg(unsigned int reg);
uint16_t read_reg(unsigned int reg);
HAL_StatusTypeDef read_data(void);
HAL_StatusTypeDef lepton_read_data(uint8_t * data);

HAL_StatusTypeDef init_lepton_command_interface(void);
HAL_StatusTypeDef enable_lepton_agc(void);
HAL_StatusTypeDef enable_telemetry(void);
HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg);
HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut);

#endif

