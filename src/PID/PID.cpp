#include <limits>
#include <iostream>
#include "PID.h"

//using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::_init(double Kp, double Ki, double Kd)
{
    std::vector<double>().swap(this->p);
    std::vector<double>().swap(this->dp);
    std::vector<double>().swap(this->ep);
    this->p = {Kp, Ki, Kd};
    this->dp = {0.1 * Kp, 0.1 * Ki, 0.1 * Kd};
    for (int i = 0; i < 3; ++i)
    {
        if (this->dp[i] == 0)
            this->dp[i] = 1;
    }
    this->reset();
}

void PID::reset()
{
    this->ep = {0, 0, 0};

    this->step = 1;
    this->index = 0;
    this->fail = 0;

    this->best_error = std::numeric_limits<double>::max();
    this->total_error = 0;
}

void PID::_update_error(double cte)
{
    int T = 50;
    int TI = 5;
    if (this->step == 1)
    {
        this->ep[0] = cte;
    }
    this->ep[2] = cte - this->ep[0];
    this->ep[0] = cte;
    this->ep[1] += cte;

    if (step % T > TI)
    {
        this->total_error += cte * cte;
    }
    if (step % T == 0)
    {
        if (step == T)
        {
            if (this->total_error < this->best_error)
            {
                this->best_error = this->total_error;
            }
            this->p[this->index] += this->dp[this->index];
        }
        else
        {
            if (this->total_error < this->best_error)
            {
                this->best_error = this->total_error;
                this->dp[this->index++] *= 1.1;
                this->index %= 3;
                this->p[this->index] += this->dp[this->index];
                this->fail = 0;
            }
            else if (this->fail == 0)
            {
                this->p[this->index] -= (2 * this->dp[this->index]);
                this->fail++;
            }
            else
            {
                this->p[this->index] += (2 * this->dp[this->index]);
                this->dp[this->index++] *= 0.9;
                this->index %= 3;
                this->p[this->index] += this->dp[this->index];
                this->fail = 0;
            }
        }
        std::cerr << "\033[31mcurr desision: "
                  << "==============  step " << step << " ==============" << std::endl;
        std::cerr << "\033[31mcurr desision: "
                  << "P: " << this->p[0] << " I: " << this->p[1] << " D: " << this->p[2] << std::endl;
        std::cerr << "\033[31mcurr desision: "
                  << "best_error: " << best_error << " total_error: " << total_error << std::endl;
        std::cerr << "\033[31mcurr desision: " << std::endl;
        total_error = 0;
    }
    ++this->step;
}

double PID::_total_error()
{
    double res = -this->p[0] * this->ep[0] - this->p[1] * this->ep[1] - this->p[2] * this->ep[2];
    res = fmod(res, 2 * M_PI);
    if (res > M_PI)
    {
        res -= 2 * M_PI;
    }
    else if (res <= -M_PI)
    {
        res += 2 * M_PI;
    }
    return res;
}