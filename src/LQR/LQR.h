#ifndef _LQR_H
#define _LQR_H

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>

struct State_Veh
{
    double x;
    double y;
    double yaw;
    double v;
    State_Veh() : x(0.0), y(0.0), yaw(0.0), v(0.0) {}
    State_Veh(double _x, double _y, double _yaw, double _v) : x(_x), y(_y), yaw(_yaw), v(_v) {}
};

struct State_X
{
    double cte;
    double d_cte;
    double theta;
    double d_theta;
    double v;
    double d_v;
    State_X() { this->reset(); }
    State_X(double _cte, double _d_cte, double _theta, double _d_theta, double _v, double _d_v) : cte(_cte), d_cte(_d_cte), theta(_theta), d_theta(_d_theta), v(_v), d_v(_d_v) {}
    void reset()
    {
        cte = 0.0, d_cte = 0.0, theta = 0.0, d_theta = 0.0, v = 0.0, d_v = 0.0;
    }
};

class LQR
{
public:
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    double dt;
    double L;

    State_X stateX;

public:
    /**
    * Constructor
    */
    LQR(double _dt = 0.05, double _L = 0.7);

    /**
    * Destructor.
    */
    virtual ~LQR() = default;

    /**
     * n polynomial curve fitting
     */
    Eigen::MatrixXd polyfit(const std::vector<cv::Point> &in_point, int n);

    /**
     * calculate k
     */
    double cal_k(const Eigen::MatrixXd &K, double _x);

    /**
     * calculate y
     */
    double cal_y(const Eigen::MatrixXd &K, double _x);

    /**
     * solve Riccati
     * @return P
     */
    bool solveRiccatiArimotoPotter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                   const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                                   Eigen::MatrixXd &P) const;

    /**
     * @return K, P, eigen_values
     */
    bool dlqr(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
              const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
              Eigen::MatrixXd &K, Eigen::MatrixXd &P, Eigen::MatrixXcd &EV) const;

    Eigen::MatrixXd control(const State_X &curX);

    void reset();
};

#endif /* _LQR_H */