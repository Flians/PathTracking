#ifndef PID_H
#define PID_H

#include <cmath>
#include <vector>
#include <climits>
#include <iostream>
#include <sys/timeb.h>
#include <opencv2/opencv.hpp>

//Constants used in some of the functions below
#define AUTOMATIC 1
#define MANUAL 0

class PID
{

public:
    PID(double Kp, double Ki, double Kd);

    void SetMode(int Mode); // * sets PID to either Manual (0) or Auto (non-0)

    double Compute(double dth, double v, cv::Point cur, cv::Point A, cv::Point B); // * performs the PID calculation.  it should be
                                                                                   //   called every time loop() cycles. ON/OFF and
                                                                                   //   calculation frequency can be set using SetMode
                                                                                   //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
                                          //   it's likely the user will want to change this depending on
                                          //   the application

    //available but not commonly used functions ********************************************************
    void SetTunings(double, double, // * While most users will set the tunings once in the
                    double);        //   constructor, this function gives the user the option
                                    //   of changing tunings during runtime for Adaptive control

    void SetControllerDirection(int); // * Sets the Direction, or "Action" of the controller. DIRECT
                                      //   means the output will increase when error is positive. REVERSE
                                      //   means the opposite.  it's very unlikely that this will be needed
                                      //   once it is set in the constructor.
    void SetSampleTime(int);          // * sets the frequency, in Milliseconds, with which
                                      //   the PID calculation is performed.  default is 100

    // [-pi, pi]
    double RoundTheta(double angle);
    // point (x0,y0) -> line (x1,y1), (x2,y2)
    double distance_from_point_to_line(
        const double &x0, const double &y0,
        const double &x1, const double &y1,
        const double &x2, const double &y2);
    // point (x0,y0) -> line (x1,y1), (x2,y2)
    bool point_in_left_line(const double &x0, const double &y0,
                            const double &x1, const double &y1,
                            const double &x2, const double &y2);
    cv::Point2d get_foot_point(cv::Point point, cv::Point pnt1, cv::Point pnt2);

private:
    void Initialize();

    /**
    * Coefficients, the order is P, I, D
    * (P)roportional Tuning Parameter
    * (I)ntegral Tuning Parameter
    * (D)erivative Tuning Parameter
    */
    std::vector<double> kp;
    /**
    * Coefficients, the order is dP, dI, dD
    */
    std::vector<double> dp;
    /**
    * Coefficients, the order is eP, eI, eD
    */
    std::vector<double> ep;

    int controllerDirection;

    unsigned long lastTime, SampleTime;
    double outMin, outMax;
    bool inAuto;
};
#endif /* PID_H */