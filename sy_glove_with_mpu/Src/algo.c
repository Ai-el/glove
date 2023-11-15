
#include "algo.h"


#define alpha_k 0.675//0.6755//越大效果越差
LowPassFilter_t LowPassFilter_pitch={
    .alpha = alpha_k,
    .prevValue = 0,
    };
LowPassFilter_t LowPassFilter_yaw={
    .alpha = alpha_k,
    .prevValue = 0
};
LowPassFilter_t LowPassFilter_roll={
    .alpha = alpha_k,
    .prevValue = 0
};


// 初始化低通滤波器
void initLowPassFilter(LowPassFilter_t* filter, float alpha, float initialValue) {
  filter->alpha = alpha;
  filter->prevValue = initialValue;
}

// 低通滤波器算法函数
float lowPassFilter(LowPassFilter_t* filter, float inputValue) {
  float filteredValue = filter->alpha * inputValue + (1 - filter->alpha) * filter->prevValue;
  filter->prevValue = filteredValue;
  return filteredValue;
}


//定义加权平均滤波器结构体
typedef struct {
  double* weights;    // 权重数组
  int numWeights;     // 权重数量
  double* buffer;     // 数据缓冲区
  int bufferSize;     // 缓冲区大小
  int currentIndex;   // 当前数据索引
} WeightedAverageFilter;

// 初始化加权平均滤波器
void initWeightedAverageFilter(WeightedAverageFilter* filter, double* weights, int numWeights) {
  filter->weights = weights;
  filter->numWeights = numWeights;
  filter->bufferSize = numWeights;
  filter->buffer = (double*)malloc(filter->bufferSize * sizeof(double));
  filter->currentIndex = 0;

  // 初始化缓冲区
  for (int i = 0; i < filter->bufferSize; i++) {
    filter->buffer[i] = 0.0;
  }
}

// 加权平均滤波器算法函数
double weightedAverageFilter(WeightedAverageFilter* filter, double inputValue) {
  // 将输入值存入缓冲区
  filter->buffer[filter->currentIndex] = inputValue;

  // 计算加权平均值
  double filteredValue = 0.0;
  for (int i = 0; i < filter->numWeights; i++) {
    int index = (filter->currentIndex - i + filter->bufferSize) % filter->bufferSize;
    filteredValue += filter->weights[i] * filter->buffer[index];
  }

  // 更新当前数据索引
  filter->currentIndex = (filter->currentIndex + 1) % filter->bufferSize;

  return filteredValue;
}

// 释放加权平均滤波器资源
void freeWeightedAverageFilter(WeightedAverageFilter* filter) {
  free(filter->buffer);
}



#define BUFFER_SIZE 5

float movingAverageFilter_x(float newSample) {
    static float buffer[BUFFER_SIZE];
    static int index = 0;
    static float sum = 0;

    // 从buffer中减去最旧的样本
    sum -= buffer[index];
    
    // 将新样本添加到buffer中
    buffer[index] = newSample;
    sum += newSample;

    // 增加index并确保循环
    index++;
    if (index >= BUFFER_SIZE) {
        index = 0;
    }

    // 返回均值
    return sum / BUFFER_SIZE;
}

float movingAverageFilter_y(float newSample) {
    static float buffer[BUFFER_SIZE];
    static int index = 0;
    static float sum = 0;

    // 从buffer中减去最旧的样本
    sum -= buffer[index];
    
    // 将新样本添加到buffer中
    buffer[index] = newSample;
    sum += newSample;

    // 增加index并确保循环
    index++;
    if (index >= BUFFER_SIZE) {
        index = 0;
    }

    // 返回均值
    return sum / BUFFER_SIZE;
}



#define FILTER_ORDER 2

float iirFilter_x(float newSample) {
    static float buffer[FILTER_ORDER];
    static float coefficients[FILTER_ORDER+1] = { 0.1, 0.2, 0.3 };  // IIR滤波器的系数

    float output = newSample;  // 初始输出等于输入样本
    
    // 应用滤波器
    for (int i = 0; i < FILTER_ORDER; i++) {
        output -= coefficients[i] * buffer[i];
    }
    
    // 更新滤波器缓冲区
    for (int i = FILTER_ORDER-1; i > 0; i--) {
        buffer[i] = buffer[i-1];
    }
    buffer[0] = newSample;

    return output;
}

float iirFilter_y(float newSample) {
    static float buffer[FILTER_ORDER];
    static float coefficients[FILTER_ORDER+1] = { 0.1, 0.2, 0.3 };  // IIR滤波器的系数

    float output = newSample;  // 初始输出等于输入样本
    
    // 应用滤波器
    for (int i = 0; i < FILTER_ORDER; i++) {
        output -= coefficients[i] * buffer[i];
    }
    
    // 更新滤波器缓冲区
    for (int i = FILTER_ORDER-1; i > 0; i--) {
        buffer[i] = buffer[i-1];
    }
    buffer[0] = newSample;

    return output;
}



 KalmanFilter kf_x = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
        .angle = 0.0f,
        .bias = 0.0f,
        .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    };

     KalmanFilter kf_y = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
        .angle = 0.0f,
        .bias = 0.0f,
        .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    };

         KalmanFilter kf_z = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
        .angle = 0.0f,
        .bias = 0.0f,
        .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
    };

    float kalman_filter_update(KalmanFilter *filter, float measurement, float dt) {
    // 预测步骤
    filter->angle += dt * (filter->bias);
    filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += filter->Q_bias * dt;
  
    // 更新步骤
    float y = measurement - filter->angle;
    float S = filter->P[0][0] + filter->R_measure;
    float K[2];
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;
  
    filter->angle += K[0] * y;
    filter->bias += K[1] * y;
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];
  
    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;
  
    return filter->angle;
}









//********************************************************************************************************
#define N 3  // 滤波器阶数
#define SAMPLING_RATE 1000.0  // 采样频率，单位为 Hz
#define CUTOFF_FREQUENCY 150.0  // 截止频率，单位为 Hz
#define pi 3.14159
//线性滤波器
float butterworthFilter_x(float inputSample) {
    static float input[N+1] = {0};
    static float output[N+1] = {0};
    static float coefficients[N+1] = {0};
    static int initialized = 0;
    
    if (!initialized) {
        // 初始化滤波器参数，这部分只会执行一次
        float wc = 2.0f * pi * CUTOFF_FREQUENCY / SAMPLING_RATE;
        float den = 1.0f;
        for (int i = 0; i < N/2; i++) {
            float c = 1.0f / tanf(wc * (2 * i + 1) / 2);
            den *= c * c + sqrtf(2.0f) * c + 1;
        }
        float num = powf(wc, N);
        float scale = num / den;
        
        // 设置滤波器系数
        for (int i = 0; i <= N; i++) {
            coefficients[i] = scale;
        }
        
        initialized = 1;
    }
    
    // 移位更新采样历史数据
    for (int i = N; i > 0; i--) {
        input[i] = input[i-1];
    }
    input[0] = inputSample;
    
    // 计算滤波输出
    float outputSample = 0.0f;
    for (int i = 0; i <= N; i++) {
        outputSample += coefficients[i] * input[i];
    }
    for (int i = N; i > 0; i--) {
        outputSample -= coefficients[i] * output[i];
    }
    output[0] = outputSample;
    
    return outputSample;
}
float butterworthFilter_y(float inputSample) {
    static float input[N+1] = {0};
    static float output[N+1] = {0};
    static float coefficients[N+1] = {0};
    static int initialized = 0;
    
    if (!initialized) {
        // 初始化滤波器参数，这部分只会执行一次
        float wc = 2.0f * pi * CUTOFF_FREQUENCY / SAMPLING_RATE;
        float den = 1.0f;
        for (int i = 0; i < N/2; i++) {
            float c = 1.0f / tanf(wc * (2 * i + 1) / 2);
            den *= c * c + sqrtf(2.0f) * c + 1;
        }
        float num = powf(wc, N);
        float scale = num / den;
        
        // 设置滤波器系数
        for (int i = 0; i <= N; i++) {
            coefficients[i] = scale;
        }
        
        initialized = 1;
    }
    
    // 移位更新采样历史数据
    for (int i = N; i > 0; i--) {
        input[i] = input[i-1];
    }
    input[0] = inputSample;
    
    // 计算滤波输出
    float outputSample = 0.0f;
    for (int i = 0; i <= N; i++) {
        outputSample += coefficients[i] * input[i];
    }
    for (int i = N; i > 0; i--) {
        outputSample -= coefficients[i] * output[i];
    }
    output[0] = outputSample;
    
    return outputSample;
}