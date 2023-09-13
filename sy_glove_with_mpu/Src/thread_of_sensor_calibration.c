#include "thread_of_sensor_calibration.h"

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 50
extern ADC_HandleTypeDef hadc1;
__IO float ADC_ConvertedValueLocal[16];
uint16_t ADC_ConvertedValue[5];
uint16_t adc_transformed_resualt[5];

typedef struct sensor_calibration_resualt_t  sensor_calibration_resualt_s;
struct sensor_calibration_resualt_t
{
	uint16_t max[5];
	uint16_t min[5];
} sensor_calibration_resualt = {
	{200, 200, 200, 200, 200},
	{30, 30, 30, 30, 30},
	// {348,890,568,496,565},
	// {240,291,260,195,201},
},
  sensor_calibration_defualt = {
	  {200, 200, 200, 200, 200},
	  {30, 30, 30, 30, 30},
};

#if 1
/*****************************************************************************************/
/*uart command and sensor calibration thread*/
void thread_of_sensor_calibration(void const *argument);								 // thread function
osThreadId tid_thread_of_sensor_calibration;											 // thread id
osThreadDef(sensor_calibration, thread_of_sensor_calibration, osPriorityNormal, 0, 128); // thread object

int init_thread_of_sensor_calibration(void)
{

	tid_thread_of_sensor_calibration = osThreadCreate(osThread(sensor_calibration), NULL);
	if (!tid_thread_of_sensor_calibration)
		return (-1);

	return (0);
}

/*****************************************************************************************/
#endif
uint8_t read_data_from_eeprom(void *msg, uint16_t Address, uint8_t msg_len);

uint8_t is_calibration, is_glove_calibration, return_resualt = 0;
uint32_t return_mark;
void thread_of_sensor_calibration(void const *argument)
{

	user_signal_info_t user_timer_info = {osThreadGetId(), SIG_USER_TIMER};
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, 50);
	for (;;)
	{

		osDelay(100);
		send_software_version_to_host();
		osDelay(100);

		// int ret = read_data_from_eeprom(&sensor_calibration_resualt, 0, sizeof(sensor_calibration_resualt));
		Internal_Flash_Read_Bytes(FLASH_USER_START_ADDR, (uint16_t *)&sensor_calibration_resualt,
														sizeof(sensor_calibration_resualt));

		if (sensor_calibration_resualt.max[0] == 0 || sensor_calibration_resualt.max[0] == 0xffff)
		{
			memcpy(&sensor_calibration_resualt, &sensor_calibration_defualt, sizeof(sensor_calibration_resualt));
		}

		struct calibration_data_t *calibration = SerialDatagramEvtAlloc(sizeof(*calibration));
		if (calibration)
		{
			SERIAL_DATAGRAM_INIT((*calibration), calibration_data);
			memcpy(calibration->tx_buff, &sensor_calibration_resualt, sizeof(sensor_calibration_resualt));
			SerialDatagramEvtSend(calibration); // 
		}

		osDelay(100);
#if 1
		for (;;)
		{

			osEvent event = osMailGet(mail_queue_id_for_cmd_calibration_cmd, 0);
			if (event.status == osEventMail)
			{
				struct serial_calibration_cmd_t *s = event.value.p;
				if (s)
				{
					is_calibration = s->cmd;
				}
				osMailFree(mail_queue_id_for_cmd_calibration_cmd, s);
			}
			else
			{
				osSignalWait(SIG_USER_TIMER, osWaitForever);
			}

			//			osEvent event = osSignalWait(0, 50);
			//			int32_t signal = event.value.signals;
			//			if (event.status != osEventSignal) {
			//				signal = 0;
			//			}
			//
			//			if(signal == SIG_USER_0){
			//				event = osMailGet(mail_queue_id_for_cmd_calibration_cmd, osWaitForever);
			//				struct serial_calibration_cmd_t *s = event.value.p;
			//				if(event.status == osEventMail){
			//					is_calibration = s->cmd;
			//				}
			//				osMailFree(mail_queue_id_for_cmd_calibration_cmd, s);
			//			}else if(signal == SIG_USER_TIMER){
			//				;
			//			}

			memcpy(ADC_ConvertedValue, ADC_Original_Value, sizeof(ADC_Original_Value));
			for (uint8_t i = 0; i < 5; i++)
			{
				ADC_ConvertedValueLocal[i] = (float)(ADC_ConvertedValue[i] & 0xFFF) * 3.3 / 4096;
				adc_transformed_resualt[i] = (uint16_t)(ADC_ConvertedValueLocal[i] * 1000);
				if (!is_calibration)
				{
					if (adc_transformed_resualt[i] < sensor_calibration_resualt.min[i])
					{
						adc_transformed_resualt[i] = sensor_calibration_resualt.min[i];
					}

					if (adc_transformed_resualt[i] > sensor_calibration_resualt.max[i])
					{
						adc_transformed_resualt[i] = sensor_calibration_resualt.max[i];
					}

					uint16_t diff = sensor_calibration_resualt.max[i] - sensor_calibration_resualt.min[i];
					if (adc_transformed_resualt[i] < (0.35 * diff + sensor_calibration_resualt.min[i]))
					{
						adc_transformed_resualt[i] = ((adc_transformed_resualt[i] - sensor_calibration_resualt.min[i]) * 90) / (0.35 * diff);
					}
					else if (adc_transformed_resualt[i] >= (0.35 * diff + sensor_calibration_resualt.min[i]))
					{
						adc_transformed_resualt[i] = (((adc_transformed_resualt[i] - sensor_calibration_resualt.min[i] - 0.35 * diff) * 90) / (0.65 * diff)) + 90;
					}
					adc_transformed_resualt[i] = 180 - adc_transformed_resualt[i];
				}
			}

			if (!is_calibration)
			{
#if 0
				struct data_host_uart_tx_t *p = SerialDatagramEvtAlloc(sizeof (*p));			
				if(p){
					SERIAL_DATAGRAM_INIT((*p), data_host_uart_tx);			
					memcpy(p->adc, adc_transformed_resualt, sizeof(adc_transformed_resualt));
					SerialDatagramEvtSend(p);
				}
#endif
			}
			if (is_calibration == 1)
			{
				struct serial_calibration_noti_t *p = SerialDatagramEvtAlloc(sizeof(*p));
				if (p)
				{
					SERIAL_DATAGRAM_INIT((*p), serial_calibration_noti);
					SerialDatagramEvtSend(p); 
				}
				glove_calibration_process();
			}
			else if (is_calibration == 2)
			{
				is_calibration = 0;

				if ((HAL_GetTick() - return_mark) > 10000)
				{
					return_mark = HAL_GetTick();
					return_resualt = glove_calibration_check();
				}
				osDelay(10); // don't delete this sytax or calibration will failed
				struct calibration_resualt_t *calibration_resualt_p = SerialDatagramEvtAlloc(sizeof(*calibration_resualt_p));
				if (calibration_resualt_p)
				{
					SERIAL_DATAGRAM_INIT((*calibration_resualt_p), calibration_resualt);
					calibration_resualt_p->tx_buff = glove_calibration_check();
					SerialDatagramEvtSend(calibration_resualt_p); 
				}
				is_glove_calibration = 0;
				//				write_data_to_eeprom(&sensor_calibration_resualt,0,sizeof(sensor_calibration_resualt));
				Internal_Flash_Write_Bytes(FLASH_USER_START_ADDR, (uint16_t *)&sensor_calibration_resualt,
										   sizeof(sensor_calibration_resualt));
			}
			else if (is_calibration == 3)
			{
				struct rawdata_tx_cmd_t *rawdata_tx_p = SerialDatagramEvtAlloc(sizeof(*rawdata_tx_p));
				if (rawdata_tx_p)
				{
					SERIAL_DATAGRAM_INIT((*rawdata_tx_p), rawdata_tx_cmd);
					uint8_t i;
					for (i = 0; i < 5; i++)
					{
						rawdata_tx_p->adc_value[i] = (uint16_t)(ADC_ConvertedValueLocal[i] * 1000);
					}
					SerialDatagramEvtSend(rawdata_tx_p); 
				}
			}
		}
#endif
	}
}

struct sensor_calibration_resualt_t sensor_calibrantion_register;

void glove_calibration_process(void)
{
	if (is_glove_calibration == 0)
	{
		is_glove_calibration = 1;
		memcpy(&sensor_calibrantion_register, &sensor_calibration_resualt, sizeof(struct sensor_calibration_resualt_t));
		memcpy(&sensor_calibrantion_register.max, adc_transformed_resualt, sizeof(adc_transformed_resualt));
		memcpy(&sensor_calibrantion_register.min, adc_transformed_resualt, sizeof(adc_transformed_resualt));
	}

	for (uint8_t i = 0; i < 5; i++)
	{
		if (sensor_calibrantion_register.max[i] <= adc_transformed_resualt[i])
		{
			sensor_calibrantion_register.max[i] = adc_transformed_resualt[i];
		}

		if (sensor_calibrantion_register.min[i] >= adc_transformed_resualt[i])
		{
			sensor_calibrantion_register.min[i] = adc_transformed_resualt[i];
		}
	}
}

int glove_calibration_check(void)
{
	int i;

	for (i = 0; i < 5; i++)
	{
		if (sensor_calibrantion_register.max[i] > 3295 || sensor_calibrantion_register.min[i] < 5)
		{
			return 1;
		}
	}

	if (sensor_calibrantion_register.max[0] - sensor_calibrantion_register.min[0] < 50)
	{
		return 2;
	}

	for (i = 1; i < 5; i++)
	{
		if (sensor_calibrantion_register.max[i] - sensor_calibrantion_register.min[i] < 70)
		{
			break;
		}
		if (i == 4)
		{
			for (i = 0; i < 5; i++)
			{
				uint16_t reusalt_diff = 0;
				reusalt_diff = sensor_calibrantion_register.max[i] - sensor_calibrantion_register.min[i];
				sensor_calibrantion_register.max[i] = sensor_calibrantion_register.max[i] - reusalt_diff * 0.05;
				sensor_calibrantion_register.min[i] = sensor_calibrantion_register.min[i] + reusalt_diff * 0.05;
			}
			memcpy(&sensor_calibration_resualt, &sensor_calibrantion_register, sizeof(struct sensor_calibration_resualt_t));
			return 2;
		}
	}

	return 2;
}
uint8_t read_data_from_eeprom(void *msg, uint16_t Address, uint8_t msg_len)
{
	uint8_t *p = msg;
	return EEPROM_ReadBytes(p, Address, msg_len);
}

uint32_t eeprom_mark;
uint8_t write_data_to_eeprom(void *msg, uint16_t Address, uint8_t msg_len)
{
	eeprom_mark = HAL_GetTick();
	uint8_t *p = msg;
	return EEPROM_WriteBytes(p, Address, msg_len);
}

void user_timer_start(void)
{
	/*creat and start a timer*/
	user_signal_info_t user_timer_info = {osThreadGetId(), SIG_USER_TIMER};
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, 50);
}

void send_software_version_to_host(void)
{
	struct software_version_t *version = SerialDatagramEvtAlloc(sizeof(software_version_s));
	if (version)
	{
		SERIAL_DATAGRAM_INIT((*version), software_version);
		version->tx_buff[0] = 0x0000;
		version->tx_buff[1] = (uint16_t)((*(__IO uint32_t *)(0X1FFFF7E8)) & 0x0000ffff);
		version->tx_buff[2] = (uint16_t)(((*(__IO uint32_t *)(0X1FFFF7E8)) & 0xffff0000) >> 16);
		SerialDatagramEvtSend(version);
	}
}

/****************************************END OF FILE*************************************/
