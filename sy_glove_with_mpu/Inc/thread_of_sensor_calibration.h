#ifndef __THREAD_OF_SENSOR_CALIBRATION_H__
#define __THREAD_OF_SENSOR_CALIBRATION_H__


#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "thread_of_host_connect.h"
#include "i2c_software.h"
#include "misc.h"
#include "adc.h"

#include "internal_flash.h"

extern osThreadId tid_thread_of_sensor_calibration;
int init_thread_of_sensor_calibration (void);
void main_data_encode(struct data_host_uart_tx_t *p, __IO float *ptr);

void glove_calibration_process(void);
int glove_calibration_check(void);

//uint8_t read_data_from_eeprom(void *msg, uint16_t Address, uint8_t msg_len);
//uint8_t write_data_to_eeprom(void *msg, uint16_t Address, uint8_t msg_len);

void user_timer_start(void);
void send_software_version_to_host(void);
#endif
/****************************************END OF FILE*************************************/
