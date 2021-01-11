#include "math.h"
#include <iostream>
#include "ExtendKalman.h"
using namespace std;

int main()
{
    double m_rho = 0.0, m_theta = 0.0, m_rho_dot = 0.0;
    double last_timestamp = 0.0, now_timestamp = 0.0;
    ExtendKalman ekf;
    while (GetRadarData(&m_rho, &m_theta, &m_rho_dot, &now_timestamp))
    {
        if(!ekf.IsInitialized())
        {
            last_timestamp = now_timestamp;
            Eigen::VectorXd x_in(4,1);
            x_in << m_rho * cos(m_theta), m_rho * sin(m_theta),
                    m_rho_dot * cos(m_theta), m_rho_dot * sin(m_theta);
            ekf.Initialization(x_in);

            // state covariance matrix P
            Eigen::MatrixXd P_in(4,4);
            P_in << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 10.0, 0.0,
                    0.0, 0.0, 0.0, 10.0;
            ekf.SetP(P_in);

            // Process covariance matrix
            Eigen::MatrixXd Q_in(4,4);
            Q_in << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;    
            ekf.SetQ(Q_in);

            // measurement covariance matrix
            // R is provided by Sensor supplier, in datasheet
            Eigen::MatrixXd R_in(3,3);
            R_in << 0.09, 0.0, 0.0,
                    0.0, 0.0009, 0.0,
                    0.0, 0.0, 0.09;
            ekf.SetR(R_in);
            continue;
        }
        
        // State transition
        double delta_t = now_timestamp - last_timestamp;
        last_timestamp = now_timestamp;
        Eigen::MatrixXd F_in(4,4);
        F_in << 1.0, 0.0, delta_t, 0.0,
                0.0, 1.0, 0.0, delta_t,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;        
        ekf.SetF(F_in);
        ekf.Prediction();

        // Measurement value;
        Eigen::VectorXd z(3,1);
        z << m_rho, m_theta, m_rho_dot;
        ekf.MeasurementUpdate(z);

        Eigen::VectorXd x_out = ekf.GetX();
        cout << "Kalman output x : " << x_out(0) << "y :" << x_out(1) << endl;
    }
    
}