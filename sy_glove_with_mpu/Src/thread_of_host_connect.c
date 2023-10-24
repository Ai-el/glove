#include "thread_of_host_connect.h"

#define EVENT_LOOP_TIME_IN_MILLI_SECOND 10

typedef struct mailBuff_s
{
	uint8_t buf[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT];
} mailBuff_t;

// osMailQDef(host_uart_tx, 30, mailBuff_t);
osMailQDef(host_uart_tx, 30, uint8_t[MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT]);
static osMailQId mail_queue_id_for_host_tx;

DEFINE_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(calibration_cmd, 1);

static const struct gloves_msg_process_func_t msg_process_func_list[] = {
	FUNC_TAB_ITEM(calibration_cmd),
};

#if 1
/*****************************************************************************************/
/*uart send to host thread*/
void thread_of_host_uart_tx(void const *argument);						   // thread function
osThreadId tid_thread_of_host_uart_tx;									   // thread id
osThreadDef(host_uart_tx, thread_of_host_uart_tx, osPriorityHigh, 0, 128); // thread object

/*uart recvive from host thread*/
void thread_of_host_uart_rx(void const *argument);						   // thread function
osThreadId tid_thread_of_host_uart_rx;									   // thread id
osThreadDef(host_uart_rx, thread_of_host_uart_rx, osPriorityHigh, 0, 128); // thread object

int init_thread_of_host_uart_tx(void)
{

	mail_queue_id_for_host_tx = osMailCreate(osMailQ(host_uart_tx), NULL);

	tid_thread_of_host_uart_tx = osThreadCreate(osThread(host_uart_tx), NULL);
	if (!tid_thread_of_host_uart_tx)
		return (-1);

	return (0);
}

int init_thread_of_host_uart_rx(void)
{

	INIT_MAIL_QUEUE_FOR_SERIAL_DATAGRAM_CMD(calibration_cmd);

	tid_thread_of_host_uart_rx = osThreadCreate(osThread(host_uart_rx), NULL);
	if (!tid_thread_of_host_uart_rx)
		return (-1);

	return (0);
}
/*****************************************************************************************/
#endif

// thread for host tx
void thread_of_host_uart_tx(void const *argument)
{

	for (;;)
	{
		uint8_t len;
		osEvent evt = osMailGet(mail_queue_id_for_host_tx, osWaitForever);
		struct uart_head_t *head = evt.value.p;
		if (evt.status == osEventMail && head != NULL)
		{
			len = head->body_len;
			if (len <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT)
			{
				host_uart_datagram_send(head, len);
			}
		}
		osMailFree(mail_queue_id_for_host_tx, head);
	}
}

// thread for host rx
void thread_of_host_uart_rx(void const *argument)
{

	for (;;)
	{
		uint8_t datagram[128]={0};
		uint8_t buf[128];
		size_t datagram_len;
		size_t buf_size;
		size_t skipped_count;

		uint8_t buf_tx[24] = {0};
		while (1)
		{
			int ret = get_raw_datagram_from_serial(buf, sizeof buf, &buf_size, &skipped_count);

			struct serial_calibration_cmd_t * p = (struct serial_calibration_cmd_t *)datagram;

			p->head.type = buf[1];
			p->head.body_len = 4;
			p->cmd = buf[3];

			struct uart_head_t *head = (const struct uart_head_t *)datagram;

			if (head->type >= 65 && head->type <= 70)
			{
				head->type -= 55;
			}
			else if (head->type >= 48 && head->type <= 57)
			{
				head->type -= 48;
			}
			if (buf[3] >= 65 && buf[3] <= 70)
			{
				buf[3] -= 55;
				p->cmd = buf[3];

			}
			if (buf[3] >= 48 && buf[3] <= 57)
			{
				buf[3] -= 48;
				p->cmd = buf[3];
			}

			// sprintf(buf_tx, "\r\n%d,%d\r\n", head->type, head->body_len);
			// uart_tx(buf_tx, 24);

			switch ((head->type))
			{
			case calibration_cmd:
				// uart_tx("yes", sizeof("yes"));
				osSignalSet(tid_thread_of_sensor_calibration, SIG_USER_0);
				break;
			}
			for (size_t i = 0; i < 1; i++)
			{
				if (head->type == msg_process_func_list[i].id)
				{
					// uart_tx("yes2", sizeof("yes2"));
					// sprintf(buf_tx, "\r\n PUT = %x\r\n", msg_process_func_list[i].ptr);
					// uart_tx(buf_tx, 24);
					serial_datagram_msg_process_common_func(datagram, 4, msg_process_func_list[i].ptr);
				}
			}
		}
#if 0
		while (0)
		{
			int ret = get_raw_datagram_from_serial(buf, sizeof buf, &buf_size, &skipped_count);
			if (!ret)
			{
				break;
			}
			uint16_t *d = (uint16_t *)datagram;
			uint8_t *s = buf;

			if (sscanf((char *)s, "%02X", (uint32_t *)d) != 1)
			{
				break;
			}
			d++;
			s += 3;
			while (s < buf + buf_size && d < (uint16_t *)(datagram + sizeof datagram))
			{
				if (sscanf((char *)s, "%04X", (uint32_t *)d) != 1)
				{
					break;
				}
				d++;
				s += 5; // format likes '00000000 '
			}
			datagram_len = ((uint8_t *)d) - datagram;

			size_t i = 0;
			const struct uart_head_t *head = (const struct uart_head_t *)datagram;
			switch (head->type)
			{
			case calibration_cmd:
			 		sprintf(buf_tx, "on");
					 uart_tx(buf_tx, 24);
				osSignalSet(tid_thread_of_sensor_calibration, SIG_USER_0);
				break;
			}
			for (i = 0; i < 1; i++)
			{
				if (head->type == msg_process_func_list[i].id)
				{
					
					 serial_datagram_msg_process_common_func(datagram, datagram_len, msg_process_func_list[i].ptr);
				}
			}
		}
#endif
	}
}

void *SerialDatagramEvtAlloc(size_t size)
{
	void *ret = NULL;

	if (size <= MAX_SIZE_OF_SERIAL_DATAGRAM_EVENT)
	{
		ret = osMailAlloc(mail_queue_id_for_host_tx, 0);
		// ret = osMailAlloc(mail_queue_id_for_host_tx, osWaitForever);
	}
	if (!ret)
	{
		return NULL;
	}
	return ret;
}

int SerialDatagramEvtSend(void *ptr)
{

	osStatus status = osMailPut(mail_queue_id_for_host_tx, ptr);
	// osStatus status = osErrorOS;//= osMailPut(mail_queue_id_for_host_tx, ptr);
	return status == osOK;
}

void SerialDatagramEvtFree(void *ptr)
{
	osMailFree(mail_queue_id_for_host_tx, ptr);
}

int host_uart_datagram_send(void *msg, const size_t msg_len)
{
	uint8_t buf[128] = {0};
	uint8_t *p = buf;
	int ret;

	if (*(uint8_t *)msg == imu_motion_distance)
	{

		// return 0;
		*p++ = SERIAL_DATAGRAM_START_CHR;
		uint8_t *s = msg;
		struct imu_motion_distance_t *distance = msg;
		memset(p, 0, 128);
		int8_t x, y;
		x = distance->x_distance;
		y = distance->y_distance;

		size_t len = sprintf(p, "%02X %02X%02X\r\n05 %04X %04X %04X %04X %04X \n",
							 *s, (uint8_t)y, (uint8_t)x,
							 distance->angle[0], distance->angle[1], distance->angle[2], distance->angle[3], distance->angle[4]);
		
		// size_t len = sprintf(p, "\r\n%d, %d, %d, %d, %d \r\n",
		// 					 distance->angle[0], distance->angle[1], distance->angle[2], distance->angle[3], distance->angle[4]);
		// size_t len = sprintf(p, "\r\nΔx=%d, Δy=%d \r\n",
		// 					(uint8_t)y, (uint8_t)x);
		
		ret = send_raw_datagram_to_serial(buf, len);

		return ret;
	}

	//	struct serial_datagram_head_t *head = msg;
	*p++ = SERIAL_DATAGRAM_START_CHR;
	uint8_t *s = msg;

	size_t len = sprintf((char *)p, "%02X ", *s); // 头
	p += len;
	s++;
	s++;
	while ((uint8_t *)s < (uint8_t *)msg + msg_len && p < buf + (sizeof buf) - 9)
	{
		len = sprintf((char *)p, "%02X", *(s + 1));
		p += len;
		len = sprintf((char *)p, "%02X ", *s);
		s += 2;
		p += len;
	}

	if ((uint8_t *)s != (uint8_t *)msg + msg_len)
	{
		return 0; // too long datagram
	}
	p--;
	*p++ = SERIAL_DATAGRAM_END_CHR;

	ret = send_raw_datagram_to_serial(buf, p - buf);

	return ret;
}

#include "usart.h"
void uart_tx(void *buf, uint32_t len)
{
	// if(len >= 64)
	// {
	// 	return ;
	// }

	HAL_UART_Transmit(&huart1, buf, len, 10);
}

int send_raw_datagram_to_serial(const void *raw_datagram, size_t raw_datagram_len)
{
#if 1
	taskENTER_CRITICAL();
	uart_tx(raw_datagram, raw_datagram_len);
	taskEXIT_CRITICAL();
#else
	// struct AsyncIoResult_t IoResult = {0, osThreadGetId()};
	// osStatus status = StartUartTx(1, raw_datagram, raw_datagram_len, NotifyAsyncIoFinished, &IoResult);
	// if (status != osOK)
	// {
	// 	return 0;
	// }
	// osSignalWait(SIG_SERVER_FINISHED, osWaitForever); // osWaitForever
	// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.

#endif
	return 1;
}

int get_raw_datagram_from_serial(uint8_t *raw_datagram, size_t max_size, size_t *actual_size_ptr, size_t *skipped_byte_count_ptr)
{
	struct AsyncIoResult_t IoResult = {0, osThreadGetId()};
	int i;

	*skipped_byte_count_ptr = 0;

	for (i = 0;; i++)
	{
		osStatus status = StartUartRx(1, raw_datagram, max_size, SERIAL_DATAGRAM_END_CHR, NotifyAsyncIoFinished, &IoResult);
		size_t len;
		uint8_t *start_pos;
		size_t offset;

		if (status != osOK)
		{
			continue;
		}
		osEvent evt = osSignalWait(SIG_SERVER_FINISHED, 1000);
		if( evt.status == osEventTimeout)
		{
			StopUartRx(1);
			continue;
		}
		// TODO: Wait time should not be 'forever'. if it's out of time, should Call StopUartXX.
		len = IoResult.IoResult;
		if (len < 2)
		{
			*skipped_byte_count_ptr += len;
			continue;
		}
		if (raw_datagram[len - 1] != SERIAL_DATAGRAM_END_CHR)
		{
			*skipped_byte_count_ptr += len;
			continue;
		}

		start_pos = memchr(raw_datagram, SERIAL_DATAGRAM_START_CHR, len - 1);
		if (start_pos == NULL)
		{
			*skipped_byte_count_ptr += len;
			continue;
		}

		// we found it.
		offset = start_pos - raw_datagram;
		*skipped_byte_count_ptr += offset;
		*actual_size_ptr = len - 2 - offset;
		memcpy(raw_datagram, start_pos + 1, *actual_size_ptr);
		raw_datagram[*actual_size_ptr] = 0;
		if (i)
		{
			// easy to set a breakpoint when debugging.
			i = 0;
		}
		return 1;
	}
}

static int serial_datagram_msg_process_common_func(const void *msg, size_t msg_len, const void *ptr)
{
	// every type of datagram has a mail-queue, then we should put msg into it
	const osMailQId *mq_ptr = ptr;
	osMailQId mq = *mq_ptr;
	void *msg_copy = osMailAlloc(mq, 0);
	if (msg_copy == NULL)
	{

		// try to discard the 1st command in the queue
		osEvent evt = osMailGet(mq, 0);
		if (evt.status == osEventMail && evt.value.p != NULL)
		{
			osMailFree(mq, evt.value.p);
		}
		// and re-try to queue it
		msg_copy = osMailAlloc(mq, 0);

		if (msg_copy == NULL)
		{
			return 0;
		}
	}
	memcpy(msg_copy, msg, msg_len);
	osMailPut(mq, msg_copy);
	return 1;
}

/****************************************END OF FILE*************************************/
