// Linear Kalman Filter for 3D constant-velocity motion model.
// State: [x, y, z, vx, vy, vz]. Measurement: [x, y, z].
// Owns F, H, Q, R, P matrices and provides predict() and update().
