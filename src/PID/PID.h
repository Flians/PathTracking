#ifndef _PID_H
#define _PID_H

#include <cmath>
#include <vector>

class PID
{
private:
    int step;
    int index; // [0 -> p, 1 -> i, 2 -> d]
    int fail;
    /**
    * Coefficients, the order is P, I, D
    */
    std::vector<double> p;
    /**
    * Coefficients, the order is dP, dI, dD
    */
    std::vector<double> dp;
    /**
    * Coefficients, the order is eP, eI, eD
    */
    std::vector<double> ep;
    double best_error;
    double total_error;

public:
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void _init(double Kp, double Ki, double Kd);

    void reset();

    /*
    * Update the PID error variables given cross track error.
    */
    void _update_error(double cte);

    /*
    * Calculate the total PID error.
    */
    double _total_error();
};

#endif /* PID_H */