# Official SUPER frontend restoration

Reference: hku-mars/SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096`.

Restored A* and corridor generator source/header files byte-for-byte. Restored SuperPlanner::PathSearch from that revision. Removed experimental body-clearance and route-memory code, options and associated tests. A* uses the original inflated-map-first search, original fallback, and 0.1s default budget.

Other existing integration changes remain. comparison.json and diffs/ enumerate differences in matching planner C++ files; this is not a claim that the whole stack matches upstream. In particular trajectory optimization limit checks, diagnostic changes, ROS2 integration and runtime execution integration still require separate assessment. User-requested 0.2m inflation remains an optional configuration; restoring the algorithm does not prove that configuration is traversable or safe for the configured 0.43m IMU-centered robot radius.

The preceding body-clearance dense trial failed its outbound goal timeout and landed/disarmed normally, with zero forest contact reports. No flight with the restored code has been claimed as a pass. Simulator and owned controller processes were stopped after landing.
