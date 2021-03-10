#include "Pid.h"

unsigned long millis()
{
    struct timeb t1;
    ftime(&t1);
    return t1.millitm + t1.time * 1000;
}

PID::PID(double Kp, double Ki, double Kd)
{
    this->inAuto = true;
    this->kp.resize(3, 0);
    this->ep.resize(3, 0);
    PID::SetOutputLimits(-0.3, 0.3); //default output limit

    SampleTime = 50; //default Controller Sample Time is 0.05 seconds

    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - SampleTime;
}

double PID::Compute(double dth, double v, cv::Point cur, cv::Point A, cv::Point B)
{
    if (!inAuto)
        return false;
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    if (timeChange >= SampleTime)
    {
        double cte = distance_from_point_to_line(cur.x, cur.y, A.x, A.y, B.x, B.y);
        cv::Point foot = get_foot_point(cur, A, B);
        bool left = (foot.x - cur.x) * (B.y - A.y) - (B.x - B.y) * (foot.y - cur.y) > 0;
        if (dth > M_PI)
        {
            dth -= 2 * M_PI;
        }
        else if (dth < -M_PI)
        {
            dth += 2 * M_PI;
        }
        /*Compute all the working error variables*/
        this->ep[2] = this->kp[2] * dth;
        this->ep[0] = this->kp[0] * cte;
        this->ep[1] += this->kp[1] * cte * v * SampleTime / 1000.0;
        /* limit integral*/
        if (this->ep[1] > outMax)
        {
            this->ep[1] = outMax;
        }
        else if (this->ep[1] < outMin)
        {
            this->ep[1] = outMin;
        }

        /*Compute PID Output*/
        double output;
        if (left)
        {
            output = -this->ep[0] - this->ep[1] + this->ep[2];
        }
        else
        {
            output = this->ep[0] + this->ep[1] + this->ep[2];
        }

        RoundTheta(output);
        /* limit output*/
        if (output > outMax)
        {
            output = outMax;
        }
        else if (output < outMin)
        {
            output = outMin;
        }

        /*Remember some variables for next time*/
        lastTime = now;
        return output;
    }
    else
        return __DBL_MAX__;
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    double SampleTimeInSec = ((double)SampleTime) / 1000;
    this->kp = {Kp, Ki * SampleTimeInSec, Kd / SampleTimeInSec};
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        this->kp[1] *= ratio;
        this->kp[2] /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max)
{
    if (Min >= Max)
        return;
    outMin = Min;
    outMax = Max;

    if (inAuto)
    {
        if (this->ep[1] > outMax)
            this->ep[1] = outMax;
        else if (this->ep[1] < outMin)
            this->ep[1] = outMin;
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto)
    { /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
    if (this->ep[1] > outMax)
        this->ep[1] = outMax;
    else if (this->ep[1] < outMin)
        this->ep[1] = outMin;
}

double PID::RoundTheta(double angle)
{
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double PID::distance_from_point_to_line(
    const double &x0, const double &y0,
    const double &x1, const double &y1,
    const double &x2, const double &y2)
{
    double d = (fabs((y2 - y1) * x0 + (x1 - x2) * y0 + ((x2 * y1) - (x1 * y2)))) /
               (sqrt(pow(y2 - y1, 2) + pow(x1 - x2, 2)));
    return d;
}

bool PID::point_in_left_line(const double &x0, const double &y0,
                             const double &x1, const double &y1,
                             const double &x2, const double &y2)
{
    return (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0) >= 0;
}

cv::Point2d PID::get_foot_point(cv::Point point, cv::Point pnt1, cv::Point pnt2)
{
    double A = pnt2.y - pnt1.y;                   //y2-y1
    double B = pnt1.x - pnt2.x;                   //x1-x2;
    double C = pnt2.x * pnt1.y - pnt1.x * pnt2.y; //x2*y1-x1*y2
    if (A * A + B * B < 1e-13)
    {
        return cv::Point2d(pnt1.x, pnt1.y); //pnt1与pnt2重叠
    }
    else if (abs(A * point.x + B * point.y + C) < 1e-13)
    {
        return cv::Point2d(point.x, point.y); //point在直线上(pnt1_pnt2)
    }
    else
    {
        double x = (B * B * point.x - A * B * point.y - A * C) / (A * A + B * B);
        double y = (-A * B * point.x + A * A * point.y - B * C) / (A * A + B * B);
        cv::Point2d fpoint(x, y);
        return fpoint;
    }
}