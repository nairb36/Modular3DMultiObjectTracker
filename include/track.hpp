// Represents a single tracked object through time.
// Owns a KalmanFilter for state estimation plus passive metadata (bbox dims, yaw).
// Tracks lifecycle info: age, hits, consecutive misses, track ID.
