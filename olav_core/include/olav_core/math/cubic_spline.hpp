/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( O ) ( L ) ( A ) ( V )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +                  OLAV: Off-Road Light Autonomous Vehicle                  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

namespace OLAV {
namespace ROS {

/**
 * @brief One-dimensional cubic spline.
 */
typedef Eigen::Spline<double, 1, Eigen::Dynamic> Spline1d;

/**
 * @brief One-dimensional interpolator using a cubic spline.
 */
typedef Eigen::SplineFitting<Spline1d> SplineFitting1d;

/**
 * @brief Class defining a one-dimensional interpolator using a cubic spline.
 */
class CubicSpline {
  public:
    /**
     * @brief Construct a new cubic spline objects from a vector of knots.
     *
     * @param knots Vector of knots used to build the cubic spline.
     */
    CubicSpline(const Eigen::RowVectorXd& knots,
                const Eigen::RowVectorXd& values,
                const int& degree);

    /**
     * @brief Evaluate the one-dimensional cubic spline interpolator at a
     * specified point.
     *
     * @param point The point on the one-dimensional grid where the spline will
     * be evaluated.
     * @return double The value of the spline at the specified point.
     */
    double Evaluate(const double& point) const;

  private:
    /**
     * @brief One-dimensional interpolator using a cubic spline.
     */
    Spline1d interpolator_;
};

} // namespace ROS
} // namespace OLAV
