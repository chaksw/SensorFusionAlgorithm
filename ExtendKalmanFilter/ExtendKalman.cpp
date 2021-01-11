#include "math.h"
#include "ExtendKalman.h"
#include "eigen3/Eigen/Dense"
using namespace Eigen;


ExtendKalman::ExtendKalman()
{
    is_initialized_ = false;
}

Eigen::VectorXd ExtendKalman::GetX()
{
    return x_;
}
bool ExtendKalman::IsInitialized(){return is_initialized_; };
void ExtendKalman::Initialization(Eigen::VectorXd x_in)
{
    x_ = x_in;
    is_initialized_ = true;
}

void ExtendKalman::SetF(Eigen::MatrixXd F_in){F_ = F_in; };
void ExtendKalman::SetP(Eigen::MatrixXd P_in){P_ = P_in; };
void ExtendKalman::SetQ(Eigen::MatrixXd Q_in){Q_ = Q_in; };
void ExtendKalman::SetR(Eigen::MatrixXd R_in){R_ = R_in; };

void ExtendKalman::Prediction()
{
    // 新的状态向量和协方差矩阵不需要重新申请内存，只需要将原有的变量代替即可
    x_ = F_ * x_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void ExtendKalman::CalculateJacobianMatrix()
{
    Eigen::MatrixXd Hj(3,4);
    // get state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // prec-compute a set of terms to avoid repeated calculation
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        H_ = Hj;
        return;
    }

    // compute the Jacobien matrix
    Hj << (px/c2), (py/2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py * (vx * py - vy * px)/c3, px * (px * vy - py * vx)/c3, px/c2, py/c2;
    H_ = Hj;
    return;
};

void ExtendKalman::MeasurementUpdate(const Eigen::VectorXd &z)
{
    double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    double theta = atan2(x_(1), x_(0));
    double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
    VectorXd h = VectorXd(3);
    h << rho, theta, rho_dot;

    VectorXd y = z - h;

    CalculateJacobianMatrix();
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
}