#include "LQR.h"

LQR::LQR(double _dt, double _L)
{
    Q.setIdentity(5, 5);
    R.setIdentity(2, 2);
    A.setZero(5, 5);
    B.setZero(5, 2);

    this->dt = _dt;
    this->L = _L;
}

Eigen::MatrixXd LQR::polyfit(const std::vector<cv::Point> &in_point, int n)
{
    int _size = in_point.size();
    // the number of parameters
    int x_num = n + 1;
    // Y = UK
    // y = k0 + x*K1 + x^2 * K2 + ...
    Eigen::MatrixXd mat_u(_size, x_num);
    Eigen::MatrixXd mat_y(_size, 1);

    for (int i = 0; i < _size; ++i)
        for (int j = 0; j < x_num; ++j)
        {
            mat_u(i, j) = pow(in_point[i].x, j);
        }

    for (int i = 0; i < _size; ++i)
    {
        mat_y(i, 0) = in_point[i].y;
    }
    // K =(U_T * U).inv() * U_T * Y
    Eigen::MatrixXd mat_k(x_num, 1);
    mat_k = (mat_u.transpose() * mat_u).inverse() * mat_u.transpose() * mat_y;
    return mat_k;
}

double LQR::cal_k(const Eigen::MatrixXd &K, double _x)
{
    double k = 0;
    for (size_t j = 1, n = K.rows(); j < n; ++j)
    {
        k += K(j, 0) * pow(_x, j - 1) * j;
    }
    return k;
}

double LQR::cal_y(const Eigen::MatrixXd &K, double _x)
{
    double y = 0;
    for (size_t j = 0, n = K.rows(); j < n; ++j)
    {
        y += K(j, 0) * pow(_x, j);
    }
    return y;
}

bool LQR::solveRiccatiArimotoPotter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                                    Eigen::MatrixXd &P) const
{
    const uint dim_x = A.rows();
    const uint dim_u = B.cols();

    // set Hamilton matrix
    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    int j = 0;
    for (int i = 0; i < 2 * dim_x; ++i)
    {
        if (Eigs.eigenvalues()[i].real() < 0.)
        {
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
            ++j;
        }
    }

    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (Vs_2 * Vs_1.inverse()).real();

    return true;
}

bool LQR::dlqr(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
               Eigen::MatrixXd &K, Eigen::MatrixXd &P, Eigen::MatrixXcd &EV) const
{
    bool res = this->solveRiccatiArimotoPotter(A, B, Q, R, P);
    K = (B.transpose() * P * B + R).inverse() * (B.transpose() * P * A);
    Eigen::EigenSolver<Eigen::MatrixXd> es(A - B * K);
    EV = es.eigenvalues();
    return res;
}

Eigen::MatrixXd LQR::control(const State_X &curX)
{
    A.setZero();
    A(0, 0) = 1.0;
    A(0, 1) = this->dt;
    A(1, 2) = curX.v;
    A(2, 2) = 1.0;
    A(2, 3) = this->dt;
    A(4, 4) = 1.0;

    B.setZero();
    B(3, 0) = curX.v / this->L; //this->dt;
    B(4, 1) = this->dt;
    Eigen::MatrixXd K, P;
    Eigen::MatrixXcd EV;
    dlqr(A, B, Q, R, K, P, EV);

    this->stateX.d_cte = (curX.cte - this->stateX.cte) / this->dt;
    this->stateX.cte = curX.cte;
    this->stateX.d_theta = (curX.theta - this->stateX.theta) / this->dt;
    this->stateX.theta = curX.theta;
    this->stateX.d_v = curX.d_v;

    Eigen::MatrixXd X(5, 1);
    X << this->stateX.cte, this->stateX.d_cte, this->stateX.theta, this->stateX.d_theta, this->stateX.d_v;

    // [w, a]
    Eigen::MatrixXd U = -1 * (K * X);
    U(1, 0) *= this->dt;
    return U;
}

void LQR::reset()
{
    this->stateX.reset();
}