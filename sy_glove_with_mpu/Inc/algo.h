
#ifndef __ALGO_H_
#define __ALGO_H_

#include "stdio.h"
#include "math.h"
#include "stdlib.h"

// 定义低通滤波器结构体
typedef struct
{
  float alpha;     // 滤波器的衰减系数，取值范围 [0, 1]
  float prevValue; // 上一次滤波器的输出结果

  //   float pitch;
  //   float roll;
  //   float yaw;
} LowPassFilter_t;

// extern LowPassFilter_t LowPassFilter;
extern LowPassFilter_t LowPassFilter_pitch;
extern LowPassFilter_t LowPassFilter_roll;
extern LowPassFilter_t LowPassFilter_yaw;

float lowPassFilter(LowPassFilter_t *filter, float inputValue);

float movingAverageFilter_x(float newSample);
float movingAverageFilter_y(float newSample);

float iirFilter_x(float newSample);
float iirFilter_y(float newSample);

typedef struct
{
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float P[2][2];
} KalmanFilter;

extern KalmanFilter kf_x,kf_y,kf_z;

float kalman_filter_update(KalmanFilter *filter, float measurement, float dt);

float butterworthFilter_x(float inputSample);
float butterworthFilter_y(float inputSample);
#endif
