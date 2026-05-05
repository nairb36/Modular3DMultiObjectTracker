// Abstract base class for state estimation models.
// Defines the interface: predict(), update(), getState(), getCovariance().
// Concrete implementations: LinearKF (v1), UKF (v2).

#include <Eigen/Dense>


class MotionModel
{
    private:
    Eigen::VectorXd x_; // State
    Eigen::MatrixXd P_; // Covariance

    public:
    virtual Eigen::Vector3d get_position() = 0;
    virtual Eigen::MatrixXd get_covariance() = 0;
    virtual void predict(double dt) = 0;
    virtual void update(Eigen::VectorXd measurement) = 0;

};