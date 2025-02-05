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

#include <sstream>
#include <vector>

#include <Eigen/Core>

namespace OLAV {
namespace ROS {

/**
 * @brief Calculate the s-th derivative of a k-th order Gram polynomial
 * evaluated at i over (2 * m + 1) points.
 *
 * @param i
 * @param m
 * @param k
 * @param s
 * @return double
 */
double
ComputeGramPolynomial(const int i, const int m, const int k, const int s);

/**
 * @brief Compute the generalized factorial \f$ (a)(a-1)(a-2)...(a-b+1) \f$.
 *
 * @param a
 * @param b
 * @return double
 */
double ComputeGeneralizedFactorial(const int a, const int b);

/*!
 * Weight Calculates the weight of the ith data point for the t'th
 * Least-Square point of the s'th derivative, over `2m+1` points, order n
 */
double
ComputeWeight(const int i, const int t, const int m, const int n, const int s);

/*!
 * Computes the weights for each data point over the window of size `2*m+1`,
 * evaluated at time t, with order n and derivative s
 */
std::vector<double>
ComputeWeights(const int m, const int t, const int n, const int s);

class SavitzkyGolayFilterSettings {
  public:
    //! Window size is 2*m+1
    unsigned m = 5;
    //! Time at which the filter is applied
    // For real-time, should be t=m
    int t = 5;
    //! Polynomial order
    unsigned n = 3;
    //! Derivation order (0 for no derivation)
    unsigned s = 0;
    //! Time step
    double dt = 1;

    /**
     * @brief Construct a filter with default weights
     */
    SavitzkyGolayFilterSettings() {}

    /**
     * @brief Construct a filter with the specified configuration
     *
     * @param m Window size is `2*m+1`
     * @param t Time at which the filter is applied
     * - `t=m` for real-time filtering.
     *   This uses only past information to determine the filter value and thus
     *   does not introduce delay. However, this comes at the cost of filtering
     *   accuracy as no future information is available.
     * - `t=0` for smoothing. Uses both past and future information to determine
     * the optimal filtered value
     * @param n Polynamial order
     * @param s Derivation order
     * - `0`: No derivation
     * - `1`: First order derivative
     * - `2`: Second order derivative
     * @param dt Time step
     */
    SavitzkyGolayFilterSettings(unsigned m,
                                int t,
                                unsigned n,
                                unsigned s,
                                double dt = 1.)
        : m(m),
          t(t),
          n(n),
          s(s),
          dt(dt) {}

    /**
     * @brief Time at which the filter is evaluated
     */
    int data_point() const { return t; }

    /**
     * @brief Derivation order
     */
    unsigned derivation_order() const { return s; }

    /**
     * @brief Polynomial order
     */
    unsigned order() const { return n; }

    /**
     * @brief Full size of the filter's window `2*m+1`
     */
    unsigned window_size() const { return 2 * m + 1; }

    /**
     * @brief Time step
     */
    double time_step() const { return dt; }

    friend std::ostream& operator<<(std::ostream& os,
                                    const SavitzkyGolayFilterSettings& conf);
};

struct SavitzkyGolayFilter {
    SavitzkyGolayFilter() { Initialize(); }

    SavitzkyGolayFilter(unsigned m,
                        int t,
                        unsigned n,
                        unsigned s,
                        double dt = 1.)
        : conf_(m, t, n, s, dt) {
        Initialize();
    }

    SavitzkyGolayFilter(const SavitzkyGolayFilterSettings& conf) : conf_(conf) {
        Initialize();
    }

    void configure(const SavitzkyGolayFilterSettings& conf) {
        conf_ = conf;
        Initialize();
    }

    /**
     * @brief Filter data using the Savitsky-Golay convolution.
     *
     * @param v Container with the data to be filtered.
     * Should have 2*m+1 elements
     * Type of elements needs to be compatible with multiplication by a scalar,
     * and addition with itself
     * Common types would be std::vector<double>, std::vector<Eigen::VectorXd>,
     * boost::circular_buffer<Eigen::Vector3d>...
     *
     * @return Filtered value according to the precomputed filter weights.
     */
    template <typename T>
    typename T::value_type filter(const T& v) const {
        assert(v.size() != weights_.size() || v.size() < 1);
        using Tv = typename T::value_type;
        Tv res = weights_[0] * v[0];
        for(size_t i = 1; i < v.size(); ++i) { res += weights_[i] * v[i]; }
        return res / dt_;
    }

    /**
     * @brief Return the Savitsky-Golay filter weights.
     *
     * @return std::vector<double> Savitsky-Golay filter weights.
     */
    std::vector<double> GetWeights() const { return weights_; }

    /**
     * @brief Get the Savitzky-Golay filter settings.
     *
     * @return SavitzkyGolayFilterSettings Savitzky-Golay filter settings.
     */
    SavitzkyGolayFilterSettings GetSettings() const { return conf_; }

  private:
    void Initialize();

    SavitzkyGolayFilterSettings conf_;

    std::vector<double> weights_;

    double dt_;
};

} // namespace ROS
} // namespace OLAV
