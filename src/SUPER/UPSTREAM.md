Source: https://github.com/hku-mars/SUPER
Commit: 2ad3419c127a617c6d7df6925e81a14175a9c096 (master)

Imported only planner/ROG-Map sources, their required headers, two trajectory
messages, and two reference planner configurations. ROS1 adapters, original
simulators/renderers, mission executable, unused messages, tuning applications,
figures/videos and unrelated maps are excluded. The original benchmark PCDs
live in simulator/assets/super; the RViz2 layout is in src/zav_super/rviz.

Local changes: ROS2-only builds/dependency declarations; portable std::isnan;
external sensor clock ownership; honor use_sim_time; simulation-clock timers;
serialized executor; absolute configuration paths; opt-in ZAV controller-status
gate to stop replanning after landing/fault. Planning algorithms and
trajectory optimizers remain upstream. See docs/super-navigation.md for the
separate ZAV sensor, controller and mission adapters and parameter overrides.

Source headers specify LGPL-3.0-or-later for SUPER/ROG-Map, even where the
upstream ROS manifests say BSD. Original per-file notices are retained.
