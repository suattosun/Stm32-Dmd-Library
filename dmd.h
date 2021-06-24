/*
 * dmd.h
 *
 *  Created on: 19 Haz 2021
 *      Author: SUAT-ARGE
 */
#include "stm32f4xx_hal.h"
#include "SystemFont5x7.h"
#include "Arial14.h"
#ifndef INC_DMD_H_
#define INC_DMD_H_
typedef enum {
	false, true
} bool_e;
void dmd_init( GPIO_TypeDef *Port_all ,
		uint16_t data_pin, uint16_t clk_pin, uint16_t sclk_pin, uint16_t a_pin,
		uint16_t b_pin, int16_t oe_pin);
void dmd_clear();
void dmd_config(uint8_t column , uint8_t row);
void dmd_call_data();
void dmd_write_string(char *str, uint8_t start_column, uint8_t start_row);
void dmd_write_char(char data, uint8_t column, uint8_t row);
void dmd_write_char_to_byte(uint8_t data, uint8_t column, uint8_t row);
void dmd_write_pixel(unsigned int bX, unsigned int bY, bool_e bPixel);
int dmd_get_up_to_four_div(unsigned int bDiv);
int dmd_get_up_to_four_mods(unsigned int bY);
int dmd_get_up_to_eight_div(unsigned int bDiv);
int dmd_get_up_to_eight_mods(unsigned int bY);
void dmd_get_point_y_data(uint8_t column_first_result);
void dmd_write_to_byte(uint8_t byte);
void dmd_sclk_config();
void dmd_write_byte_address(uint8_t byte);
void dmd_write_byte(uint8_t byte);
void dmd_write_max(uint8_t address, uint8_t data);
void dmd_oe_state(bool_e state);
void delayus(uint32_t timer);
#endif /* INC_DMD_H_ */
