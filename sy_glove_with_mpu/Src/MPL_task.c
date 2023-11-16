#include "MPL_task.h"
#include "usart.h"
#include "math.h"
#include "algo.h"
#include "protocol.h"

#define IMU_SENS 1

float Pitch = 0, Roll = 0, Yaw = 0;
float pitch_old, yaw_old;

uint32_t count, count_old;
static struct hal_s hal = {0};

static signed char gyro_orientation[9] =
	{
		-1, 0, 0,
		0, -1, 0,
		0, 0, 1};

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(
	const signed char *mtx);

osThreadId MPL_TaskHandle;

void init_MPL_task(void)
{
	osThreadDef(MPL_task, Start_MPL_task, osPriorityNormal, 0, 128);
	MPL_TaskHandle = osThreadCreate(osThread(MPL_task), NULL);
}

void Start_MPL_task(void const *argument)
{
	osDelay(500);
	int result;
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	//	unsigned long timestamp;
	struct int_param_s int_param;

	I2C_Bus_Init();
	//	uint8_t addr1 = 0;
	//	Sensors_I2C_ReadRegister(0x68, 0x75, 1, &addr1);

	/* Set up gyro.
	 * Every function preceded by mpu_ is a driver function and can be found
	 * in inv_mpu.h.
	 */
	int_param.cb = gyro_data_ready_cb;

	result = mpu_init(&int_param);
	if (result)
	{
		//		MPU_INFO("Could not initialize gyro.result =  %d\n",result);
	}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);

	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);

	/* Initialize HAL state variables. */
	memset(&hal, 0, sizeof(hal));
	hal.sensors = ACCEL_ON | GYRO_ON;
	hal.report = PRINT_QUAT;

	/* To initialize the DMP:
	 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_mpu_dmp_motion_driver.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	 * an event at the four orientations where the screen should rotate.
	 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	 * no motion.
	 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	 */

	dmp_load_motion_driver_firmware();
	dmp_set_orientation(
		inv_orientation_matrix_to_scalar(gyro_orientation));
	/*
	 * Known Bug -
	 * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
	 * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
	 * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
	 * there will be a 25Hz interrupt from the MPU device.
	 *
	 * There is a known issue in which if you do not enable DMP_FEATURE_TAP
	 * then the interrupts will be at 200Hz even if fifo rate
	 * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
	 *
	 * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
	 已知错误-
			DMP启用后将以200Hz采样传感器数据，并以dmp_set_fifo_rate API中指定
			的速率输出到FIFO。 一旦将样本放入FIFO，DMP将发送一个中断。 因此，
			如果dmp_set_fifo_rate为25Hz，则MPU设备会产生25Hz中断。如果不启用
			DMP_FEATURE_TAP，则即使将fifo速率设置为其他速率，中断也会以200Hz
			发生。 为避免此问题，包括DMP_FEATURE_TAP，DMP传感器融合仅适用于
			+ -2000dps和加速+ -2G的陀螺仪*/
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
					   DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
					   DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	hal.dmp_on = 1;

	user_signal_info_t user_timer_info = {osThreadGetId(), SIG_USER_TIMER};
	osTimerDef(main_timer, SetUserSignal);
	osTimerId main_timer_id = osTimerCreate(osTimer(main_timer), osTimerPeriodic, &user_timer_info);
	osTimerStart(main_timer_id, 10);

	count_old = osKernelSysTick();
	for (;;)
	{
		osSignalWait(SIG_USER_TIMER, osWaitForever);
		unsigned long sensor_timestamp;
		//		get_tick_count(&timestamp);

		if (!hal.sensors || !hal.new_gyro)
		{
			/* Put the MSP430 to sleep until a timer interrupt or data ready
			 * interrupt is detected.
			__bis_SR_register(LPM0_bits + GIE);
			 */
			continue;
		}

		if (hal.new_gyro && hal.dmp_on)
		{
			short gyro[3], accel[3], sensors;
			unsigned char more;
			long quat[4];
			/* This function gets new data from the FIFO when the DMP is in
			 * use. The FIFO can contain any combination of gyro, accel,
			 * quaternion, and gesture data. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
			 * the FIFO isn't being filled with accel data.
			 * The driver parses the gesture data to determine if a gesture
			 * event has occurred; on an event, the application will be notified
			 * via a callback (assuming that a callback function was properly
			 * registered). The more parameter is non-zero if there are
			 * leftover packets in the FIFO.
			 */
			dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
						  &more);
			if (!more)
			{
				hal.new_gyro = 0;
			}
			/* Gyro and accel data are written to the FIFO by the DMP in chip
			 * frame and hardware units. This behavior is convenient because it
			 * keeps the gyro and accel outputs of dmp_read_fifo and
			 * mpu_read_fifo consistent.
			 */
			//			if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
			//			{
			//				MPU_INFO("PRINT_GYRO\r\n");
			//			}
			//
			//			if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
			//			{
			//				MPU_INFO("PRINT_ACCEL\r\n");
			//			}

			/* Unlike gyro and accel, quaternions are written to the FIFO in
			 * the body frame, q30. The orientation is set by the scalar passed
			 * to dmp_set_orientation during initialization.
			 */
			count = osKernelSysTick();

			if (count - count_old < 40)
			{
				continue;
			}

			count_old = count;
			if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
			{
				uint8_t buf[128] = {0};
				uint8_t *p = buf;

				long data[3];
				inv_get_sensor_type_euler(data, quat);
				Pitch = data[0] * 1.0 / (1 << 16);
				Roll = data[1] * 1.0 / (1 << 16);
				Yaw = data[2] * 1.0 / (1 << 16);

				int32_t data_temp[4];
				data_temp[0] = Pitch;
				data_temp[1] = Roll;
				// data_temp[2] = rep[0];
				// data_temp[3] = rep[1];

				// set_computer_value(SEND_FACT_CMD,CURVES_CH4,&data_temp[3],1);
				// set_computer_value(SEND_FACT_CMD,CURVES_CH3,&data_temp[2],1);
				// set_computer_value(SEND_TARGET_CMD,CURVES_CH2,&data_temp[1],1);
				// set_computer_value(SEND_TARGET_CMD,CURVES_CH1,&data_temp[0],1);



				Pitch = lowPassFilter(&LowPassFilter_pitch, Pitch);	
				Roll = lowPassFilter(&LowPassFilter_roll, Roll);
				Yaw = lowPassFilter(&LowPassFilter_yaw, Yaw);
				

				// Pitch = kalman_filter_update(&kf_x,Pitch,0.01);
				// Roll = kalman_filter_update(&kf_y,Roll,0.01);
				// Yaw = kalman_filter_update(&kf_z,Yaw,0.01);



				int8_t rep[2];

				// 保留三位数，避免过于灵敏
				rep[0] = (int8_t)((roundf((yaw_old - Yaw) * 1000) / 1000) * 11);
				if (rep[0] <= IMU_SENS && rep[0] >= -IMU_SENS)
				{
					rep[0] = 0;
				}

				// rep[0] = (yaw_old - Yaw) * 11;
				// rep[1] = -((pitch_old - Pitch) * 11);
				rep[1] = (int8_t)((roundf((pitch_old - Pitch) * 1000) / 1000) * 11);
				if (rep[1] <= IMU_SENS && rep[1] >= -IMU_SENS)
				{
					rep[1] = 0;
				}

				// rep[0] = movingAverageFilter_x(rep[0]);
				// rep[1] = movingAverageFilter_y(rep[1]);

				// rep[0] = iirFilter_x(rep[0]);
				// rep[1] = iirFilter_x(rep[1]);
				
				rep[0] = butterworthFilter_x(rep[0]);
				rep[1] = butterworthFilter_y(rep[1]);
					// rep[0] = kalman_filter_update(&kf_x,rep[0],0.01);
					// rep[1] = kalman_filter_update(&kf_y,rep[1],0.01);

				// size_t len = sprintf(p, "Δx=%d \t Δy=%d \t Pitch=%.3f \t Roll=%.3f \tYaw=%.3f\r\n",
				// 					 rep[0], rep[1],
				// 					 Pitch, Roll, Yaw);

				// size_t len = sprintf(p, "Pitch=%.3f \t Roll=%.3f \t Yaw=%.3f \r\n",Pitch, Roll, Yaw);
				// size_t len = sprintf(p, "Pitch \t %d.%d \t Roll \t %d.%d \t Yaw \t %d.%d \r\n",
				// (int16_t)Pitch, abs((int16_t)((Pitch - ((int16_t)Pitch))*10000)),
				// (int16_t)Roll, abs((int16_t)((Roll - ((int16_t)Roll))*10000)),
				// (int16_t)Yaw,abs((int16_t)((Yaw - ((int16_t)Yaw))*10000)));
				
				// send_raw_datagram_to_serial(buf, len);

				
				// data_temp[0] = Pitch;
				// data_temp[1] = Roll;
				// data_temp[2] = rep[0];
				// data_temp[3] = rep[1];

				// set_computer_value(SEND_FACT_CMD,CURVES_CH4,&data_temp[3],1);
				// set_computer_value(SEND_FACT_CMD,CURVES_CH3,&data_temp[2],1);
				// set_computer_value(SEND_FACT_CMD,CURVES_CH2,&data_temp[1],1);
				// set_computer_value(SEND_FACT_CMD,CURVES_CH1,&data_temp[0],1);


				struct imu_motion_distance_t *distance = SerialDatagramEvtAlloc(sizeof(*distance));
				if (distance)
				{
					SERIAL_DATAGRAM_INIT((*distance), imu_motion_distance);
					distance->x_distance = rep[0];
					distance->y_distance = rep[1];

					extern uint16_t adc_transformed_resualt[5];
					memcpy(distance->angle, adc_transformed_resualt, sizeof(adc_transformed_resualt));
					SerialDatagramEvtSend(distance);
				}

				pitch_old = Pitch;
				yaw_old = Yaw;
				// MPU_INFO("%.4f %.4f %.4f\r\n",Pitch, Roll, Yaw);
			}
		}
	}
}

void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == MPU_INT_GPIO_PIN)
	{
		gyro_data_ready_cb();
	}
}

/** Performs a multiply and shift by 29. These are good functions to write in assembly on
 * with devices with small memory where you want to get rid of the long long which some
 * assemblers don't handle well
 * @param[in] a
 * @param[in] b
 * @return ((long long)a*b)>>29
 */
long inv_q29_mult(long a, long b)
{
	long long temp;
	long result;
	temp = (long long)a * b;
	result = (long)(temp >> 29);
	return result;
}

/**
 *  @brief      Body-to-world frame euler angles.
 *  The euler angles are output with the following convention:
 *  Pitch: -180 to 180
 *  Roll: -90 to 90
 *  Yaw: -180 to 180
 *  @param[out] data        Euler angles in degrees, q16 fixed point.
 *  @param[out] accuracy    Accuracy of the measurement from 0 (least accurate)
 *                          to 3 (most accurate).
 *  @param[out] timestamp   The time in milliseconds when this sensor was read.
 *  @return     1 if data was updated.
 */
int inv_get_sensor_type_euler(long *data, long *quat)
{
	long t1, t2, t3;
	long q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
	float values[3];

	q00 = inv_q29_mult(quat[0], quat[0]);
	q01 = inv_q29_mult(quat[0], quat[1]);
	q02 = inv_q29_mult(quat[0], quat[2]);
	q03 = inv_q29_mult(quat[0], quat[3]);
	q11 = inv_q29_mult(quat[1], quat[1]);
	q12 = inv_q29_mult(quat[1], quat[2]);
	q13 = inv_q29_mult(quat[1], quat[3]);
	q22 = inv_q29_mult(quat[2], quat[2]);
	q23 = inv_q29_mult(quat[2], quat[3]);
	q33 = inv_q29_mult(quat[3], quat[3]);

	/* X component of the Ybody axis in World frame */
	t1 = q12 - q03;

	/* Y component of the Ybody axis in World frame */
	t2 = q22 + q00 - (1L << 30);
	values[2] = -atan2f((float)t1, (float)t2) * 180.f / (float)M_PI;

	/* Z component of the Ybody axis in World frame */
	t3 = q23 + q01;
	values[0] =
		atan2f((float)t3,
			   sqrtf((float)t1 * t1 +
					 (float)t2 * t2)) *
		180.f / (float)M_PI;
	/* Z component of the Zbody axis in World frame */
	t2 = q33 + q00 - (1L << 30);
	if (t2 < 0)
	{
		if (values[0] >= 0)
			values[0] = 180.f - values[0];
		else
			values[0] = -180.f - values[0];
	}

	/* X component of the Xbody axis in World frame */
	t1 = q11 + q00 - (1L << 30);
	/* Y component of the Xbody axis in World frame */
	t2 = q12 + q03;
	/* Z component of the Xbody axis in World frame */
	t3 = q13 - q02;

	values[1] =
		(atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) *
			 180.f / (float)M_PI -
		 90);
	if (values[1] >= 90)
		values[1] = 180 - values[1];

	if (values[1] < -90)
		values[1] = -180 - values[1];
	data[0] = (long)(values[0] * 65536.f);
	data[1] = (long)(values[1] * 65536.f);
	data[2] = (long)(values[2] * 65536.f);

	return 1;
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7; // error
	return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
	const signed char *mtx)
{
	unsigned short scalar;

	/*
	   XYZ  010_001_000 Identity Matrix
	   XZY  001_010_000
	   YXZ  010_000_001
	   YZX  000_010_001
	   ZXY  001_000_010
	   ZYX  000_001_010
	 */

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;

	return scalar;
}
