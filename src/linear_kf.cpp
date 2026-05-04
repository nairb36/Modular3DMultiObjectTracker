// Implementation of LinearKF: predict() and update() using standard KF equations.
// Matrix setup (F, H, Q, R, P) happens in the constructor.


#include "linear_kf.hpp"

LinearKF::LinearKF()
{
    x_.resize(6);
    P_.resize(6,6);
    F_.resize(6,6);
    Q_.resize(6,6);
    H_.resize(3,6);
    R_.resize(3,3);
}