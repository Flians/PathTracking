#ifndef _SPLINE_H
#define _SPLINE_H

#include <cassert>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>

namespace frl
{
    class Spline
    {
    public:
        enum bd_type
        {
            Natural = 1,
            Clamped = 2,
            NotAKnot = 3
        };

    private:
        /**
    * f_i(x) = d_i*(x-x_i)^3 + c_i*(x-x_i)^2 + b_i*(x-x_i) + a_i
    * f_i(x_i) = y_i
    * f_i(x_i+1) = y_i+1
    * d_f_i(x_i+1) = d_f_i+1(x_i+1)
    * dd_f_i(x_i+1) = dd_f_i+i(x_i+1)
    * -->
    * A*M = 6*B -> A*(M/2) = 3*B
    * -->
    * a_i = y_i
    * b_i = (a_i+1 - a_i)/h_i - h_i*(2*c_i + c_i+1)/3, h_i = x_i+1 - x_i
    * c_i = M_i / 2
    * d_i = (c_i+1 - c_i) / (3 * h_i)
    */
        std::size_t n;
        std::vector<double> m_x;
        std::vector<double> m_a, m_b, m_c, m_d; // spline coefficients

        double m_b0, m_c0; // for left extrapol
        bd_type m_left, m_right;
        double m_left_value, m_right_value;
        bool m_force_linear_extrapolation;

        void reset(std::size_t n);
        std::size_t index(double x) const;

    public:
        // set default boundary condition to be zero curvature at both ends
        Spline() : m_left(Natural), m_right(Natural),
                   m_left_value(0.0), m_right_value(0.0),
                   m_force_linear_extrapolation(false)
        {
        }

        ~Spline() = default;

        void clean();

        // optional, but if called it has to come be before set_points()
        void set_boundary(bd_type left, double left_value,
                          bd_type right, double right_value,
                          bool force_linear_extrapolation = false);

        void set_points(const std::vector<cv::Point2d> &_points);

        double calculate_y(double x) const;
        double calculate_dy(double x) const;
        double calculate_ddy(double x) const;
    };

    class Spline2D
    {
    private:
        Spline sx;
        Spline sy;

    public:
        std::size_t IN;
        std::vector<double> IS;
        std::vector<cv::Point2d> IPoints;

        Spline2D() = default;
        ~Spline2D() = default;

        void clean();

        template <typename T = double>
        void set_points(const std::vector<cv::Point_<T>> &_points, bool is_spline = false, double ds = 0.1);

        cv::Point2d calculate_position(double s) const;
        double calculate_curvature(double s) const;
        double calculate_yaw(double s) const;
    };

    template <typename T>
    void Spline2D::set_points(const std::vector<cv::Point_<T>> &_points, bool is_spline, double ds)
    {
        this->clean();
        std::size_t n = _points.size();
        std::vector<cv::Point2d> px(n, {0, 0}), py(n, {0, 0});
        px[0].y = _points[0].x;
        py[0].y = _points[0].y;
        double s = 0;
        for (std::size_t i = 1; i < n; i++)
        {
            s += hypot(_points[i].x - _points[i - 1].x, _points[i].y - _points[i - 1].y);
            px[i] = {s, (double)_points[i].x};
            py[i] = {s, (double)_points[i].y};
        }
        this->sx.set_points(px);
        this->sy.set_points(py);

        if (is_spline)
        {
            this->IN = 0;
            this->IPoints.reserve(ceil(s / ds) + 1);
            this->IS.reserve(ceil(s / ds) + 1);
            double cur_s = 0;
            while (cur_s <= s)
            {
                ++this->IN;
                this->IS.emplace_back(cur_s);
                this->IPoints.emplace_back(this->calculate_position(cur_s));
                cur_s += ds;
                if (s > cur_s && s - cur_s < ds)
                    cur_s = s;
            }
        }
        else
        {
            this->IN = n;
            this->IPoints.reserve(n);
            this->IS.reserve(n);
            for (std::size_t i = 0; i < n; i++)
            {
                this->IS.push_back(px[i].x);
                this->IPoints.emplace_back(px[i].y, py[i].y);
            }
        }
    }
} // namespace frl
#endif /* _SPLINE_H */
