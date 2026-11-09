# Recent visited trail validation — 2026-11-09

Production implementation: src/omove_description/src/visited_trail_layer.cpp.
Configuration and operational instructions: src/omove_description/README.md.

Validation:

- Build: colcon build --packages-select omove_description, successful.
- CTest (source ROS and overlay first, ROS_DOMAIN_ID=95): 3/3 targets passed.
  Includes the existing 29 Python tests, real plugin loading with a simulated
  clock (grace, decay, expiry, reset, clock rewind), preservation of special
  cost values, and actual NavFn alternate-route / sole-corridor tests.
- Isolated domain 94 / Gazebo port 11361 corner regression with production
  parameters: NavigateToPose status 4, 36.66 simulated seconds. corner.json
  stores sampled commands, poses and DWB evaluations. This validates successful
  corner navigation, not a guarantee that all real layouts avoid repetition.
- Restarted only the existing production navigation launch; preserved Gazebo
  and SLAM Toolbox localization. All managed navigation nodes active and global
  costmap logs confirm visited_trail_layer initialized.
- Read live /global_costmap/global_costmap/visited_trail: map-frame position
  present. /global_costmap/global_costmap/clear_visited_trail service present.

Trajectory is bounded, in-memory history retained across goals. It does not
recover trajectories from before plugin activation. Cost clearing, time rewind,
large localization jump, expiry, and navigation restart can remove that history.
Runtime log: /tmp/omove_visited_trail_navigation.log.
