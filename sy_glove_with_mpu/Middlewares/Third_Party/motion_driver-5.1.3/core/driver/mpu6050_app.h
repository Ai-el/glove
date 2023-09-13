/*
 * @Author: Aijw aijw@siyizn.com
 * @Date: 2023-09-01 09:04:32
 * @LastEditors: Aijw aijw@siyizn.com
 * @LastEditTime: 2023-09-01 09:23:40
 * @FilePath: \sy_glove_with_mpu\Middlewares\Third_Party\motion_driver-5.1.3\core\driver\mpu6050_app.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/************************************************************
  * Copyright @shanghai.siyi
  * FileName: mpu6050_app.h
  * CreatDate: 2020.3.6
  * Description:
  * Version:		1
  * Subversion:	1
  * Revision:		1
  * change log:
	* 2020.3.6:first edit
  ***********************************************************/
	
#ifndef __MPU6050__APP_H__
#define __MPU6050__APP_H__


#include "mpu6050_io.h"

// #include "RTE_Components.h"
#ifdef	RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif



/* Data read from MPL. */
#define PRINT_ACCEL            (0x01)
#define PRINT_GYRO             (0x02)
#define PRINT_QUAT             (0x04)
#define PRINT_COMPASS          (0x08)
#define PRINT_EULER            (0x10)
#define PRINT_ROT_MAT          (0x20)
#define PRINT_HEADING          (0x40)
#define PRINT_PEDO             (0x80)
#define PRINT_LINEAR_ACCEL     (0x100)
#define PRINT_GRAVITY_VECTOR   (0x200)


#define ACCEL_ON               (0x01)
#define GYRO_ON                (0x02)




struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
};

void eMPL_start_cfg(void);

void gyro_data_ready_cb(void);
long inv_q29_mult(long a, long b);
int inv_get_sensor_type_euler(float *data, long *quat);



#endif
