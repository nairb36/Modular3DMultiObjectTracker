// Abstract base class for state estimation models.
// Defines the interface: predict(), update(), getState(), getCovariance().
// Concrete implementations: LinearKF (v1), UKF (v2).

#include <Eigen/Dense>


class MotionModel
{
    private:
    // State
    Eigen::VectorXd x_;
    // Covariance
    Eigen::MatrixXd P_;

    public:
    virtual Eigen::VectorXd get_position() = 0;
    virtual Eigen::MatrixXd get_covariance() = 0;
    virtual void predict(double dt) = 0;
    virtual void update(Eigen::VectorXd measurement) = 0;

};