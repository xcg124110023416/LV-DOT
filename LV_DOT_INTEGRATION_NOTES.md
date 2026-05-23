# LV-DOT Integration Notes for the Smart Cane Simulation

This note records the current integration risks found while reviewing LV-DOT
against the Gazebo + FAST-LIO-SAM-QN + MPC simulation.

## Recommended Simulation Strategy

For the current Gazebo pedestrian simulation, the preferred approach is to get
dynamic pedestrian ground truth directly from Gazebo and publish it through the
planner-facing interface:

```text
/onboard_detector/dynamic_obstacles_info
```

The message type is:

```text
onboard_detector/DynamicObstacles
```

This keeps navigation/MPC evaluation separate from perception errors and avoids
tuning LV-DOT before the dynamic avoidance behavior itself is stable.

## LV-DOT Planner Interface

The planner already subscribes to:

```text
/onboard_detector/dynamic_obstacles_info
```

LV-DOT also publishes this topic from `dynamicDetector::publishDynamicObstacles`.
So the high-level interface between LV-DOT and MPC is already compatible.

## Point Cloud Frame Assumption

The current LV-DOT LiDAR callback assumes the input point cloud is in a local
body/LiDAR-like frame, not already in the global map frame.

In `dynamicDetector::lidarOdomCB`, the code:

1. Converts the input `PointCloud2` to PCL.
2. Filters points using the point cloud's own `x/y` values within
   `localLidarRange_`.
3. Applies the current LiDAR pose transform to move the cloud into the map
   frame.

Because of this, the input cloud should normally be a body/local cloud.

### Better Default: `/cloud_registered_body`

FAST-LIO publishes:

```text
/cloud_registered_body
```

with `frame_id = body`. This matches LV-DOT's current assumption best.

### Risky Default: `/cloud_registered`

FAST-LIO publishes:

```text
/cloud_registered
```

in the global `camera_init` frame. Feeding this directly into the current
LV-DOT LiDAR pipeline is likely wrong because:

1. The cloud is already global, but LV-DOT applies odometry/extrinsics again.
   This can double-transform detections.
2. LV-DOT filters by raw cloud `x/y` before transforming. For a global cloud,
   as the robot moves away from the map origin, valid local points can be
   incorrectly removed by the `+/- localLidarRange_` filter.

Therefore, `/cloud_registered` is only appropriate if LV-DOT is modified to
support a "cloud already in map frame" mode and skips the body-to-map transform.

### Also Risky: `/corrected_current_pcd`

FAST-LIO-SAM-QN publishes:

```text
/corrected_current_pcd
```

This is also a corrected global-frame cloud. It is suitable for the local
planning ESDF in the current navigation stack, but it does not match LV-DOT's
current LiDAR input assumption.

## Odometry Choice

For unmodified LV-DOT with `/cloud_registered_body`, the odometry should be in
the same coordinate convention used to transform body-frame points into the map.
The likely choices are:

```text
/Odometry
```

or, for the localization pipeline:

```text
/localization_odom
```

Use only one consistent pair:

```text
body/local cloud + matching global odometry
```

Avoid mixing a global cloud with global odometry unless LV-DOT is changed to
skip the transform.

## Current Code Issue: One-Frame LiDAR Pose Lag

In `dynamicDetector::lidarOdomCB`, the point cloud is transformed using
`positionLidar_` and `orientationLidar_` before those values are updated from
the current odometry message.

This creates a one-frame pose lag, and the first callback may use uninitialized
or stale transform values. If LV-DOT is connected seriously, update the LiDAR
pose from the current odometry before transforming the point cloud.

The same pattern should be checked in `lidarPoseCB`.

## Current Simulation Gaps for Full LV-DOT

The current smart cane Gazebo model has a simulated Velodyne LiDAR, but no RGB-D
camera topics:

```text
/camera/depth/image_rect_raw
/camera/color/image_raw
```

The default LV-DOT launch starts the YOLOv11 node, which expects a color image
topic. Full LiDAR-visual LV-DOT integration would require:

1. Adding an RGB-D camera to the Gazebo URDF/SDF.
2. Publishing compatible depth and color topics.
3. Setting correct camera intrinsics.
4. Setting correct body-to-camera and body-to-LiDAR extrinsics.
5. Verifying YOLO runtime dependencies and model weights.

This is possible, but it is a larger perception integration task and should not
block simulation-side MPC dynamic avoidance testing.

## Practical Next Steps

1. For navigation experiments, publish Gazebo pedestrian ground truth as
   `onboard_detector/DynamicObstacles`.
2. For LV-DOT experiments, start with LiDAR-only mode:
   - Disable YOLO in the launch file.
   - Use `/cloud_registered_body` as the LiDAR input.
   - Use a matching odometry topic.
   - Fix the one-frame pose lag in the LiDAR callback.
3. Only use `/cloud_registered` or `/corrected_current_pcd` after adding an
   explicit global-cloud mode to LV-DOT.
