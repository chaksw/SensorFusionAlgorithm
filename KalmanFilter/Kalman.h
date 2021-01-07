/*
 * @Author: Runsheng WU 
 * @Date: 2021-01-07 13:20:40 
 * @Last Modified by: Runsheng WU
 * @Last Modified time: 2021-01-07 17:24:37
 * 
 * Kalman Filter for IMU(Gyroscopes + Accelerometer) mesurement estimation
 */

#ifndef KALMAN_H
#define KALMAN_H

class Kalman
{
    /* Kalman filter parameters and varibles */
private:
    float angle;    // Angle calculated (estimated) by KF which is one of the element in state vector 2x1
    float bias;     // gyro bias calculated (estimated) by KF which is one of the element in state vector 2x1
    float rate;     // unbias rate, control input of system

    // the (co)variances of Guaussian process noise 
    float Q_Angle;  // Process noise varicance for the accelerometer
    float Q_bias;   // Process noise variance for the gyro bias
    float R_measure;    // Measurement noise variance

    float P[2][2];  // Error covariance matrix - This is a 2x2 matrix

public:
    Kalman();
    ~Kalman();
    float getAngle(float newAngle, float newRate, float dt);
    void setAngle(float angle);

    void setRate(float rate);
    float getRate();

    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbais();
    float getRmeasure();



};


#endif