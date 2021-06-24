 
#include "dmd.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#define DMD_WIDTH 32
#define DMD_HEIGHT 16
#define FOUR_BLOK 4
#define DMD_BLOK 15
uint8_t max_point_matrix;
uint8_t first_char = 4;
uint8_t char_width = 2;
uint8_t char_height = 3;

uint8_t dmd_max_column = 15;
uint8_t dmd_max_row_part = 3;
uint8_t config_start_column = 4;
uint8_t **data;
GPIO_TypeDef *GPIO_PORT;

uint16_t DATA_PIN, CLK_PIN, SCLK_PIN, A_PIN, B_PIN, OE_PIN;
void dmd_init(GPIO_TypeDef *gpio_port, uint16_t data_pin, uint16_t clk_pin,
		uint16_t sclk_pin, uint16_t a_pin, uint16_t b_pin, int16_t oe_pin) {

	GPIO_PORT = gpio_port;
	DATA_PIN = data_pin;
	CLK_PIN = clk_pin;
	SCLK_PIN = sclk_pin;
	A_PIN = a_pin;
	B_PIN = b_pin;
	OE_PIN = oe_pin;

}
void dmd_clear() {
	for (int j = 0; j < max_point_matrix; j++) {
		for (int i = 0; i < dmd_max_column; i++) {
			data[i][j] = 0;
		}
	}
}
void dmd_config(uint8_t column, uint8_t row) {
	dmd_max_column = ((dmd_max_column + 1) * column);
	dmd_max_row_part = ((dmd_max_row_part + 1) * row) - 1;
	max_point_matrix = dmd_max_row_part + 1;

	data = (uint8_t**) malloc((dmd_max_column) * sizeof(uint8_t*));

	for (int i = 0; i < (dmd_max_column); i++) {
		data[i] = (uint8_t*) malloc((max_point_matrix) * sizeof(uint8_t));
	}
	dmd_clear();
}

void dmd_call_data() {

	for (int j = 0; j < max_point_matrix; j++) {
		dmd_oe_state(false);
		for (int i = 0; i < dmd_max_column; i++) {
			dmd_write_max(j, (data[i][j]));
		}
		dmd_oe_state(true);
		dmd_sclk_config();

	}

}

static int select_font(int array_index) {
	return System5x7[array_index];
}
void dmd_write_string(char *str, uint8_t start_column, uint8_t start_row) {

	while (*str) {
		dmd_write_char(*str++, start_column, start_row);
		start_column += (select_font(char_width) + 1);

	}

}

void dmd_write_char(char data, uint8_t column, uint8_t row) {

	uint16_t ascii_code_start = ((data - select_font(first_char))
			* select_font(char_width)) + (select_font(char_width) + 1);
	uint16_t ascii_code = ascii_code_start + select_font(char_width);
	uint16_t ascii_code_32 = ascii_code - config_start_column - column;
	for (int array_index = ascii_code_start; array_index < ascii_code;
			array_index++)
		dmd_write_char_to_byte(select_font(array_index),
				array_index - ascii_code_32, row);

}
void dmd_write_char_to_byte(uint8_t data, uint8_t column, uint8_t row) {
	uint8_t last_row = row + select_font(char_height);
	column++;
	for (int j = last_row; j >= row; j--) {
		dmd_write_pixel(column, j, 0x80 & data);
		data = data << 1;
	}

}

void dmd_write_pixel(unsigned int bX, unsigned int bY, bool_e bPixel) {

	if (bX >= (dmd_max_column * DMD_WIDTH)
			|| bY >= (max_point_matrix * DMD_HEIGHT)) {
		return;
	}
	//dmd_oe_state(false);
	uint8_t bModY = dmd_get_up_to_four_mods(bY);
	//dmd_get_point_y_data(bModY);
	uint8_t bDivY = dmd_get_up_to_four_div(bY);
	uint8_t bDivX = dmd_get_up_to_eight_div(bX);
	uint8_t bModX = dmd_get_up_to_eight_mods(bX);
	uint8_t dmdSize = (4 * bDivX) + bDivY;

	dmdSize = DMD_BLOK - dmdSize;

	if (bPixel) {
		data[dmdSize][bModY] |= 1 << bModX;
	} else {
		data[dmdSize][bModY] &= ~(1 << bModX);
	}

}

int dmd_get_up_to_four_div(unsigned int bDiv) {
	return bDiv / FOUR_BLOK;
}
int dmd_get_up_to_four_mods(unsigned int bY) {
	return bY % FOUR_BLOK;
}
int dmd_get_up_to_eight_div(unsigned int bDiv) {
	bDiv = bDiv / (select_font(char_height) + 1);
	return 3 - bDiv;
}
int dmd_get_up_to_eight_mods(unsigned int bY) {
	bY = bY % (select_font(char_height) + 1);
	return select_font(char_height) - bY;
}
void dmd_get_point_y_data(uint8_t column_first_result) {

	switch (column_first_result) {
	case 0:
		HAL_GPIO_WritePin(GPIO_PORT, A_PIN, GPIO_PIN_RESET); // Pull the CLK HIGH
		HAL_GPIO_WritePin(GPIO_PORT, B_PIN, GPIO_PIN_RESET); // Pull the CLK HIGH
		break;
	case 1:
		HAL_GPIO_WritePin(GPIO_PORT, A_PIN, GPIO_PIN_SET); // Pull the CLK HIGH
		HAL_GPIO_WritePin(GPIO_PORT, B_PIN, GPIO_PIN_RESET); // Pull the CLK HIGH
		break;
	case 2:
		HAL_GPIO_WritePin(GPIO_PORT, A_PIN, GPIO_PIN_RESET); // Pull the CLK HIGH
		HAL_GPIO_WritePin(GPIO_PORT, B_PIN, GPIO_PIN_SET); // Pull the CLK HIGH
		break;
	case 3:
		HAL_GPIO_WritePin(GPIO_PORT, A_PIN, GPIO_PIN_SET); // Pull the CLK HIGH
		HAL_GPIO_WritePin(GPIO_PORT, B_PIN, GPIO_PIN_SET); // Pull the CLK HIGH
		break;

	}

}

void dmd_sclk_config() {
	HAL_GPIO_WritePin(GPIO_PORT, SCLK_PIN, GPIO_PIN_SET);
	DWT_Delay_us(1000);
	HAL_GPIO_WritePin(GPIO_PORT, SCLK_PIN, GPIO_PIN_RESET);

}
void dmd_write_byte_address(uint8_t byte) {

	dmd_get_point_y_data(byte);

}
void dmd_write_byte(uint8_t byte) { // ikisinden birini çıkar

	for (uint8_t i = 0; i < select_font(char_height) + 1; i++) {

		HAL_GPIO_WritePin(GPIO_PORT, CLK_PIN, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIO_PORT, DATA_PIN, (~byte & 0x80));
		byte = byte << 1;

		HAL_GPIO_WritePin(GPIO_PORT, CLK_PIN, GPIO_PIN_SET);

	}

}
void dmd_write_max(uint8_t address, uint8_t data) {

	dmd_write_byte(data);
	dmd_write_byte_address(address);

}
void dmd_oe_state(bool_e state) {
	HAL_GPIO_WritePin(GPIO_PORT, OE_PIN, (GPIO_PinState) state);
}

uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}
