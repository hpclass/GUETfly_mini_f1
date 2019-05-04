
#ifndef __KALMAN_FILTER_H_
#define __KALMAN_FILTER_H_

#define Pi 	3.141592654
//卡尔曼噪声参数
#define dt           0.012
#define R_angle      0.5
#define Q_angle      0.0001
#define Q_gyro       0.0001//越大越滞后

//引用外部变量
extern short aacx, aacy, aacz;		//加速度传感器原始数据
extern short gyrox, gyroy, gyroz;	//陀螺仪原始数据

double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void Kalman_Filter_Z(float Accel, float Gyro);
void yijiehubu_P(float angle_m, float gyro_m);
void erjiehubu_P(float angle_m, float gyro_m);
void Erjielvbo(float angle_m,float gyro_m);

#endif