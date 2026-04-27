// Implementation of Track: wraps KalmanFilter and manages per-track metadata.
// predict() delegates to KF, update() corrects state and refreshes bbox/yaw.
