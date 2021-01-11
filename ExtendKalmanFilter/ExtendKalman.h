/*
 * @Author: Runsheng WU 
 * @Date: 2021-01-08 09:55:40 
 * @Last Modified by: Runsheng WU
 * @Last Modified time: 2021-01-10 22:41:02
 * 
 * Extend Kalman Filter Implement
 * --------------------
 * Radar Objects state estimation for nearly constant velocity model
 */

#include "eigen3/Eigen/Dense"

#ifndef ExtendKalman_H
#define ExtendKalman_H

class ExtendKalman
{
private:
    // flag of initialization
    bool is_initialized_;
    
    // State vetor
    Eigen::VectorXd x_;

    // State transistion matrix
    Eigen::MatrixXd F_;

    // State convariance matrix
    Eigen::MatrixXd P_;

    // Process noise covariance matrix
    Eigen::MatrixXd Q_;

    // jacobian measurement matirx
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;
public:
    ExtendKalman();
    ~ExtendKalman();
    
    Eigen::VectorXd GetX();
    bool IsInitialized();
    void Initialization(Eigen::VectorXd x_in);
    void SetF(Eigen::MatrixXd F_in);
    void SetP(Eigen::MatrixXd P_in);
    void SetQ(Eigen::MatrixXd Q_in);
    void SetR(Eigen::MatrixXd R_in);

    void Prediction();

    void CalculateJacobianMatrix();

    void MeasurementUpdate(const Eigen::VectorXd &z);
};





#endif