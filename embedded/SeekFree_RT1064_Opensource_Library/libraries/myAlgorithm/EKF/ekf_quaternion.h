#ifndef EKF_QUATERNION_H
#define EKF_QUATERNION_H

#include "math.h"
#include <stdlib.h>
#include "arm_math.h"
#include "kalman.h"
/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct {

    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;
    //7维状态变量
    float q[4];
    float GyroBias[3];

    //量测状态
    float gyro[4];
    float acc[3];

    //常规值用于判断是否静止
    float acc_norm;
    float gyro_norm;

    //欧拉角
    float roll,pitch,yaw;

    //四元数,零偏过程噪声
    float Q1,Q2;
    float GyroNoiseVariance[3];
    uint8_t GyroNoiseCalibrated;
    float AccelNoiseVariance[3];

    //加速度量测噪声
    float R;

    //姿态跟新周期
    float dt;

		float AdaptiveGainScale;
		float OrientationCosine[3];
    //卡方检验,渐消因子
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda; 
}QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda,
                             float gyro_noise_x_dps2, float gyro_noise_y_dps2, float gyro_noise_z_dps2,
                             float accel_noise_x_norm2, float accel_noise_y_norm2, float accel_noise_z_norm2);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
