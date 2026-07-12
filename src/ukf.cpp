// Implementation of Unscented Kalman Filter

#include "ukf.hpp"

// Wraps an angle to [-pi, pi]
static double wrap_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}


UKF::UKF(Eigen::Vector3d position)
{
    // Tunable Params
    Eigen::VectorXd sigma_squared_state(6);
    sigma_squared_state<< 100, 100, 100, 20, 3, 1; // sigma squared for x, y, z, v, yaw, yaw_d
    Eigen::VectorXd sigma_squared_measurement(3);
    sigma_squared_measurement<< 0.1, 0.1, 0.1; // sigma squared for x, y, z
    Eigen::VectorXd sigma_squared_process(2);
    sigma_squared_process << 1, 1;

    // UKF Matrices Initialization
    n_ = 6;
    n_aug_ = 8;
    x_ = Eigen::VectorXd::Zero(n_); // [x, y, z, v, yaw, yaw_d]
    x_.head(3) = position;

    x_aug_ = Eigen::VectorXd::Zero(n_aug_); // [x, y, z, v, yaw, yaw_d, nu_a, nu_yaw_dd]

    P_ = sigma_squared_state.asDiagonal();

    Q_ = sigma_squared_process.asDiagonal();

    P_aug_ =  Eigen::MatrixXd::Zero(n_aug_, n_aug_);
    P_aug_.bottomRightCorner(2, 2) = Q_;

    H_ = Eigen::MatrixXd::Zero(3, n_);
    H_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();

    R_ = sigma_squared_measurement.asDiagonal();
}


void UKF::predict(double dt)
{
    x_aug_.head(n_) = x_;
    P_aug_.topLeftCorner(n_,n_) = P_;

    // Generate Sigma Points
    Eigen::MatrixXd X_aug = generate_sigma_points();

    // Propogate Sigma Points
    Eigen::MatrixXd X_pred = propagate_sigma_points(X_aug, dt);

    // Compute mean and covariance of predicted Sigma Points
    compute_predicted_mean_and_covariance(X_pred);
}

Eigen::MatrixXd UKF::generate_sigma_points() const
{
    double lambda = 3.0 - n_aug_;

    // Matrix square root of P_aug via Cholesky decomposition: P_aug = L*L'
    Eigen::MatrixXd L = P_aug_.llt().matrixL();
    Eigen::MatrixXd scaled_L = std::sqrt(lambda + n_aug_) * L;

    Eigen::MatrixXd X_aug(n_aug_, 2*n_aug_ + 1);
    X_aug.col(0) = x_aug_;
    X_aug.block(0, 1, n_aug_, n_aug_) = scaled_L.colwise() + x_aug_;
    X_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = (-scaled_L).colwise() + x_aug_;
    return X_aug;
}

Eigen::MatrixXd UKF::propagate_sigma_points(const Eigen::MatrixXd& X_aug, double dt)
{
    Eigen::MatrixXd X_pred(n_, 2*n_aug_ + 1);

    for (int i = 0; i < 2*n_aug_ + 1; i++)
    {
        double x = X_aug(0, i);
        double y = X_aug(1, i);
        double z = X_aug(2, i);
        double v = X_aug(3, i);
        double yaw = X_aug(4, i);
        double yaw_d = X_aug(5, i);
        double nu_a = X_aug(6, i);
        double nu_yaw_dd = X_aug(7, i);

        double x_pred, y_pred;
        if (std::abs(yaw_d) < 1e-4)
        {
            // Near-zero turn rate: CTRV degenerates to straight-line motion
            x_pred = x + v*cos(yaw)*dt + 0.5*nu_a*cos(yaw)*dt*dt;
            y_pred = y + v*sin(yaw)*dt + 0.5*nu_a*sin(yaw)*dt*dt;
        }
        else
        {
            x_pred = x + (v/yaw_d)*(sin(yaw + yaw_d*dt) - sin(yaw)) + 0.5*nu_a*cos(yaw)*dt*dt;
            y_pred = y + (v/yaw_d)*(-cos(yaw + yaw_d*dt) + cos(yaw)) + 0.5*nu_a*sin(yaw)*dt*dt;
        }
        double z_pred = z;
        double v_pred = v + nu_a*dt;
        double yaw_pred = yaw + yaw_d*dt + 0.5*nu_yaw_dd*dt*dt;
        double yaw_d_pred = yaw_d + nu_yaw_dd*dt;

        X_pred.col(i) << x_pred, y_pred, z_pred, v_pred, yaw_pred, yaw_d_pred;
    }  

    return X_pred;
}


void UKF::compute_predicted_mean_and_covariance(const Eigen::MatrixXd& X_pred)
{
    double lambda = 3.0 - n_aug_;

    // New mean state
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < 2*n_aug_ + 1; i++)
    {
        double w;
        if (i == 0)
        {
            w = lambda/(lambda + n_aug_);
        }
        else
        {
            w = 0.5/(lambda + n_aug_);
        }

        x_new += w*X_pred.col(i);
    }
    x_new(4) = wrap_angle(x_new(4));

    // New state covariance matrix
    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(n_, n_);
    for (int i = 0; i < 2*n_aug_ + 1; i++)
    {
        double w;
        if (i == 0)
        {
            w = lambda/(lambda + n_aug_);
        }
        else
        {
            w = 0.5/(lambda + n_aug_);
        }

        Eigen::VectorXd residual = X_pred.col(i) - x_new;
        residual(4) = wrap_angle(residual(4));
        P_new += w*residual*residual.transpose();
    }

    // Assign to UKF State Variables
    x_ = x_new;
    P_ = P_new;
}


void UKF::update(const Eigen::VectorXd& z, const double yaw)
{
    // Linear update function implemented here since our measurements follow a linear model
    // Would need to do the unscented transform if measurement follows a non-linear model

    Eigen::VectorXd y = z - H_*x_;
    Eigen::MatrixXd S = H_*P_*H_.transpose() + R_;
    Eigen::MatrixXd K = P_*H_.transpose()*S.inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);

    x_ = x_ + K*y;
    P_ = (I - K*H_)*P_;

    // Guard against detector heading flips (pi ambiguity): if the measured yaw
    // disagrees with the current estimate by more than pi/2, flip it by pi.
    double corrected_yaw = yaw;
    if (std::abs(wrap_angle(yaw - x_(4))) > M_PI/2.0)
    {
        corrected_yaw = wrap_angle(yaw + M_PI);
    }

    // TODO: Treat yaw as a proper measurement (4D z = [x, y, z, yaw] with its own
    // R entry and wrapped innovation) instead of hard-overwriting the state yaw.
    yaw_ = corrected_yaw;
    x_(4) = corrected_yaw;
}
