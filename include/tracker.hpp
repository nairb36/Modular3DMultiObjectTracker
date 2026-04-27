// Main orchestrator that manages all tracks through their lifecycle.
// Runs the per-frame pipeline: predict → associate → update → create → delete.
// Owns the list of active tracks, the Associator, and the Detector.
