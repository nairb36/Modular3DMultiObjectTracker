// Abstract interface for computing pairwise cost between a track and a detection.
// Concrete implementations: Euclidean distance, Mahalanobis distance, 3D IoU.
// Pluggable — Associator takes a cost function as a strategy.
