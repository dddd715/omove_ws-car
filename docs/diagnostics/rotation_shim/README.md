# Forward tracking with rotation shim — 2026-11-09

The production Nav2 configuration now wraps DWB with RotationShimController.
A heading error above 0.65 rad triggers collision-checked rotation in place;
rotation disengages at 0.25 rad. Sampling distance is 0.4 m. DWB continues to
use forward-only tracking (vx >= 0, vy = 0). Maximum yaw speed is 0.65 rad/s,
and yaw acceleration/deceleration is 1.0 rad/s² in both controller and smoother.
Recovery behaviors retain their existing ability to back up.

Validated using isolated ROS domain 94 / Gazebo port 11361 and the world01
corner reproduction setup. The initial map pose is (-1.16, -2.43, 0), with
Gazebo spawn (-1.16, -3.43, 0); the map has an approximately 1 m y offset.

- corner.json: target (0.21, -0.11, 0), navigation succeeded (status 4), 36.55 simulated seconds.
- uturn.json: target (-1.6, -2.43, pi), navigation succeeded (status 4), 13.51 simulated seconds. Turned in place before advancing.
- Both reports sampled nonnegative forward command velocity and zero lateral command velocity. Samples are periodic rather than an exhaustive command recording.

The first U-turn trial used x=-2.1, outside the saved map; it was discarded
and corrected to x=-1.6. These tests do not establish performance at every
corner or in footprints where rotation is obstructed.

The source and installed nav2_params.yaml were synchronized. The controller
plugin requires navigation restart to take effect. The existing navigation
launch was restarted alone, preserving the user's simulator and SLAM Toolbox
localization. Runtime launch log: /tmp/omove_rotation_shim_navigation.log.
