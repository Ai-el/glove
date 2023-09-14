/*
 * @Author: Aijw aijw@siyizn.com
 * @Date: 2023-09-01 09:04:31
 * @LastEditors: Aijw aijw@siyizn.com
 * @LastEditTime: 2023-09-14 09:11:24
 * @FilePath: \sy_glove_with_mpu\Inc\MPL_task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MPL_TASK_H__
#define __MPL_TASK_H__

#include <string.h>
#include <math.h>

#include "cmsis_os.h"


#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

#include "mpu6050_io.h"

#include "thread_of_host_connect.h"


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




void gyro_data_ready_cb(void);
long inv_q29_mult(long a, long b);
int inv_get_sensor_type_euler(long *data, long *quat);

void init_MPL_task(void);
void Start_MPL_task(void const * argument);

extern osThreadId MPL_TaskHandle;


void MPU_PRINT(void);
#endif
