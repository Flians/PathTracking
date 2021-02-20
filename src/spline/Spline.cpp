#include "Spline.h"

namespace frl
{

    void Spline::clean()
    {
        std::vector<double>().swap(this->m_x);
        std::vector<double>().swap(this->m_a);
        std::vector<double>().swap(this->m_b);
        std::vector<double>().swap(this->m_c);
        std::vector<double>().swap(this->m_d);
    }

    void Spline::reset(std::size_t n)
    {
        this->n = n;
        this->m_x.resize(n, 0);
        this->m_a.resize(n, 0);
        this->m_b.resize(n - 1, 0);
        this->m_c.resize(n, 0);
        this->m_d.resize(n - 1, 0);
    }

    void Spline::set_boundary(bd_type left, double left_value,
                              bd_type right, double right_value,
                              bool force_linear_extrapolation)
    {
        assert(this->n == 0); // set_points() must not have happened yet
        this->m_left = left;
        this->m_right = right;
        this->m_left_value = left_value;
        this->m_right_value = right_value;
        this->m_force_linear_extrapolation = force_linear_extrapolation;
    }

    void Spline::set_points(const std::vector<cv::Point2d> &_points)
    {
        std::size_t n = _points.size();
        assert(n > 2);
        this->reset(n);
        this->m_x[0] = _points[0].x;
        this->m_a[0] = _points[0].y;
        std::vector<double> h(n - 1, 0);
        for (std::size_t i = 1; i < n; i++)
        {
            assert(_points[i - 1].x < _points[i].x);
            this->m_x[i] = _points[i].x;
            this->m_a[i] = _points[i].y;
            h[i - 1] = _points[i].x - _points[i - 1].x;
        }

        Eigen::MatrixXd A;
        A.setZero(n, n);
        Eigen::MatrixXd B;
        B.setZero(n, 1);
        // calculate A
        for (std::size_t i = 0; i < n - 1; ++i)
        {
            if (i < n - 2)
            {
                A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
            }
            A(i + 1, i) = h[i];
            A(i, i + 1) = h[i];
        }
        // boundary conditions
        if (this->m_left == Natural)
        {
            A(0, 0) = 1.0;
            A(0, 1) = 0.0;
        }
        else if (this->m_left == Clamped)
        {
            A(0, 0) = 2.0 * h[0];
            B(0, 1) = 3.0 * ((this->m_a[1] - this->m_a[0]) / h[0] - this->m_left_value);
        }
        else
        {
            A(0, 0) = -h[0];
            A(0, 1) += h[1];
            A(0, 2) = -h[0];
        }
        if (this->m_right == Natural)
        {
            A(n - 1, n - 2) = 0.0;
            A(n - 1, n - 1) = 1.0;
        }
        else if (this->m_right == Clamped)
        {
            A(n - 1, n - 1) = 2.0 * h[n - 2];
            B(n - 2, 0) = 3.0 * (this->m_right_value - (this->m_a[n - 1] - this->m_a[n - 2]) / h[n - 2]);
        }
        else
        {
            A(n - 1, n - 3) = -h[n - 2];
            A(n - 1, n - 2) += h[n - 3];
            A(n - 1, n - 1) = -h[n - 3];
        }

        // calculat B
        for (std::size_t i = 0; i < n - 2; ++i)
        {
            B(i + 1, 0) = 3.0 * (this->m_a[i + 2] - this->m_a[i + 1]) / h[i + 1] -
                          3.0 * (this->m_a[i + 1] - this->m_a[i]) / h[i];
        }

        // calculat c
        Eigen::MatrixXd c = A.colPivHouseholderQr().solve(B);
        // std::cout << A << std::endl;
        // std::cout << B << std::endl;
        // std::cout << c << std::endl;

        this->m_c.assign(c.data(), c.data() + c.rows() * c.cols());

        // calculat b, d
        for (std::size_t i = 0; i < n - 1; ++i)
        {
            this->m_b[i] = (this->m_a[i + 1] - this->m_a[i]) / h[i] - (2 * this->m_c[i] + this->m_c[i + 1]) * h[i] / 3.0;
            this->m_d[i] = (this->m_c[i + 1] - this->m_c[i]) / (3.0 * h[i]);
        }

        // for left extrapolation coefficients
        this->m_b0 = m_b[0];
        this->m_c0 = (m_force_linear_extrapolation == false) ? this->m_c[0] : 0.0;
    }

    std::size_t Spline::index(double x) const
    {
        // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
        std::vector<double>::const_iterator it;
        it = std::lower_bound(this->m_x.begin(), m_x.end(), x);
        return std::max(int(it - m_x.begin()) - 1, 0);
    }

    double Spline::calculate_y(double x) const
    {
        // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
        std::size_t idx = this->index(x);

        double dx = x - this->m_x[idx];
        double interpol;
        if (x < this->m_x[0])
        {
            // extrapolation to the left
            interpol = (this->m_c0 * dx + this->m_b0) * dx + this->m_a[0];
        }
        else if (x > this->m_x[n - 1])
        {
            // extrapolation to the right
            interpol = (this->m_c[n - 1] * dx + this->m_b[n - 1]) * dx + this->m_a[n - 1];
        }
        else
        {
            // interpolation
            interpol = ((this->m_d[idx] * dx + this->m_c[idx]) * dx + this->m_b[idx]) * dx + this->m_a[idx];
        }
        return interpol;
    }

    double Spline::calculate_dy(double x) const
    {
        std::size_t idx = this->index(x);
        double dx = x - this->m_x[idx];
        return this->m_b[idx] + dx * (2 * this->m_c[idx] + 3 * dx * this->m_d[idx]);
    }

    double Spline::calculate_ddy(double x) const
    {
        std::size_t idx = this->index(x);
        double dx = x - this->m_x[idx];
        return 2 * this->m_c[idx] + 6 * dx * this->m_d[idx];
    }

    void Spline2D::clean()
    {
        this->sx.clean();
        this->sy.clean();

        this->IN = 0;
        std::vector<double>().swap(this->IS);
        std::vector<cv::Point2d>().swap(this->IPoints);
    }

    cv::Point2d Spline2D::calculate_position(double s) const
    {
        return {this->sx.calculate_y(s), this->sy.calculate_y(s)};
    }

    double Spline2D::calculate_curvature(double s) const
    {
        double dx = this->sx.calculate_dy(s);
        double ddx = this->sx.calculate_ddy(s);
        double dy = this->sy.calculate_dy(s);
        double ddy = this->sy.calculate_ddy(s);
        return (ddy * dx - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
    }

    double Spline2D::calculate_yaw(double s) const
    {
        return atan2(this->sy.calculate_dy(s), this->sx.calculate_dy(s));
    }
} // namespace frl