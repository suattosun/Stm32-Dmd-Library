# Stm32-Dmd-Library
![Ekran Alıntısı](https://user-images.githubusercontent.com/44433690/123240561-15043d80-d4e9-11eb-9abe-444940ebb6f0.PNG)

	
//////	

dmd_init( GPIOA, GPIO_PIN_7, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_1,
GPIO_PIN_2); 
dmd_config(3, 1);
//////
These two codes are required for init.


dmd_write_pixel(bX, bY, bPixel); // to create a point if bPixel is true , light is on
dmd_write_char(data, column, row);// to create a char 
dmd_write_string(str, start_column, start_row);// to create  string
 
