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

#include <olav_control/core/filters/savitzky_golay.hpp>

namespace OLAV {
namespace ROS {

double
ComputeGramPolynomial(const int i, const int m, const int k, const int s) {
    if(k > 0) {
        return (4. * k - 2.) / (k * (2. * m - k + 1.)) *
            (i * ComputeGramPolynomial(i, m, k - 1, s) +
             s * ComputeGramPolynomial(i, m, k - 1, s - 1)) -
            ((k - 1.) * (2. * m + k)) / (k * (2. * m - k + 1.)) *
            ComputeGramPolynomial(i, m, k - 2, s);
    } else {
        if(k == 0 && s == 0) return 1.;
        else
            return 0.;
    }
}

double ComputeGeneralizedFactorial(const int a, const int b) {
    double gf = 1.;

    for(int j = (a - b) + 1; j <= a; j++) { gf *= j; }
    return gf;
}

double
ComputeWeight(const int i, const int t, const int m, const int n, const int s) {
    double w = 0;
    for(int k = 0; k <= n; ++k) {
        w = w +
            (2 * k + 1) *
                (ComputeGeneralizedFactorial(2 * m, k) /
                 ComputeGeneralizedFactorial(2 * m + k + 1, k + 1)) *
                ComputeGramPolynomial(i, m, k, 0) *
                ComputeGramPolynomial(t, m, k, s);
    }
    return w;
}

std::vector<double>
ComputeWeights(const int m, const int t, const int n, const int s) {
    std::vector<double> weights(2 * static_cast<size_t>(m) + 1);

    for(int i = 0; i < 2 * m + 1; ++i) {
        weights[static_cast<size_t>(i)] = ComputeWeight(i - m, t, m, n, s);
    }

    return weights;
}

void SavitzkyGolayFilter::Initialize() {
    // Compute weights for the time window 2*m+1, for the t'th least-square
    // point of the s'th derivative
    weights_ = ComputeWeights(static_cast<int>(conf_.m),
                              conf_.t,
                              static_cast<int>(conf_.n),
                              static_cast<int>(conf_.s));
    dt_ = std::pow(conf_.time_step(), conf_.derivation_order());
}

std::ostream& operator<<(std::ostream& os,
                         const SavitzkyGolayFilterSettings& conf) {
    os << "m                       : " << conf.m << std::endl
       << "Window Size (2*m+1)     : " << 2 * conf.m + 1 << std::endl
       << "n (Order)               :" << conf.n << std::endl
       << "s (Differentiate)       : " << conf.s << std::endl
       << "t: Filter point ([-m,m]): " << conf.t << std::endl;
    return os;
}

} // namespace ROS
} // namespace OLAV