// Matches predicted tracks to current detections each timestep.
// Builds a cost matrix, applies gating, and solves with Hungarian algorithm.
// Returns matched pairs, unmatched tracks, and unmatched detections.
