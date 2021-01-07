#include "Kalman.h"

/*  State Initialization
 * 初始化状态，包括角度、偏差等状态数据初始值的设定
 * 以及相关方差的设定。状态初始值的取值取决于传感器的标定，
 * 而初始方差的定义则取决于自己
*/
Kalman::Kalman()
{
    angle = 0.0f;
    bias = 0.0f;

    Q_Angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    /* 假设初始状态下偏差为零，且初始角度已知，则协方差矩阵可以定义为以下形式： */
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};


float Kalman::getAngle(float newAngle, float newRate, float dt)
{
    /* Discrete KF time updata equation - Time Update ("Predict") */
    /* Step 1 - Update  hatover x - Project the state ahead */
    rate = newRate - bias;
    angle += dt * rate;

    /* Step - 2 Update estimation error covariance - Project the error covariance   */
    P[0][0] += dt *( dt * P[1][1] -  P[0][1] - P[1][0] + Q_Angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += dt * Q_bias;

    float y = newAngle - angle;

    float S = P[0][0] + R_measure;

    float K[2];

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    

    return angle;

};

void Kalman::setAngle(float angle){this->angle = angle; };
float Kalman::getRate(){return this->rate; };

void Kalman::setQangle(float Q_Angle){this->Q_Angle = Q_Angle; };
void Kalman::setQbias(float Q_bias){this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure){this->R_measure = R_measure; };


float Kalman::getQbais(){return this->Q_bias; };
float Kalman::getQangle(){return this->Q_Angle; };
float Kalman::getRmeasure(){return this->R_measure; };