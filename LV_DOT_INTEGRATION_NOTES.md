# LV-DOT Integration Notes

This note records the current LV-DOT integration state for the smart cane
navigation stack. It focuses on the planner-facing interface, sensor-topic
assumptions, and remaining perception-side caveats.

## Core Flow

```mermaid
flowchart TD
    START["ROS launch entry<br/>roslaunch onboard_detector run_detector.launch"] --> A["run_detector.launch"]
    A --> B["detector_node<br/>dynamicDetector"]
    A --> C["yolov11_detector_node.py"]

    B --> DENTRY["Depth callback path<br/>depthOdomCB / depthPoseCB<br/>then detectionCB"]
    B --> LENTRY["LiDAR callback path<br/>lidarOdomCB / lidarPoseCB<br/>then lidarDetectionCB"]
    C --> YENTRY["YOLO callback path<br/>image_callback<br/>then detect_callback"]

    subgraph YOLO["YOLO semantic detection branch"]
        YENTRY --> C1["Color image topic<br/>/camera/color/image_raw"]
        C1 --> C2["YOLOv11 inference<br/>detect person"]
        C2 --> C3["Publish 2D person boxes<br/>/yolo_detector/detected_bounding_boxes"]
    end

    subgraph DEPTH["Depth image detection branch"]
        DENTRY --> D0["Depth image topic<br/>/camera/depth/image_rect_raw"]
        D0 --> D1["UV-map branch"]
        D0 --> D2["DBSCAN branch"]

        D1 --> D11["Depth image -> U-map<br/>horizontal pixel x depth histogram"]
        D11 --> D12["Extract continuous regions<br/>on U-map"]
        D12 --> D13["Recover depth bbox / bird-view"]
        D13 --> D14["Generate 3D UV BBox"]

        D2 --> D21["Project depth image<br/>to 3D point cloud"]
        D21 --> D22["Voxel / height / range filtering"]
        D22 --> D23["DBSCAN point cloud clustering"]
        D23 --> D24["Generate 3D DBSCAN BBox"]

        D14 --> D3["Depth BBox fusion<br/>UV BBox + DBSCAN BBox"]
        D24 --> D3
    end

    subgraph LIDAR["LiDAR detection branch"]
        LENTRY --> L0["LiDAR point cloud topic<br/>/cloud_registered_body"]
        L0 --> L1["LiDAR DBSCAN clustering"]
        L1 --> L2["Size filtering"]
        L2 --> L3["Generate 3D LiDAR BBox"]
    end

    subgraph FUSION["Multi-source fusion and human confirmation"]
        D3 --> F1["Depth / LiDAR 3D BBox fusion<br/>based on 3D IoU"]
        L3 --> F1
        C3 --> F2["YOLO 2D box"]
        F1 --> F3["Project 3D BBox<br/>to image plane"]
        F2 --> F4["2D IoU matching<br/>YOLO person box vs projected 3D box"]
        F3 --> F4
        F4 --> F5["Matched boxes are marked<br/>is_human = true<br/>is_dynamic = true"]
        F4 --> F6["If multiple YOLO boxes match one 3D box<br/>split 3D point cloud by projected points"]
    end

    subgraph TRACK["Tracking and dynamic classification"]
        F5 --> T1["Current filteredBBoxes"]
        F6 --> T1
        T1 --> T2["boxAssociation<br/>match current boxes to history"]
        T2 --> T3["Kalman Filter<br/>estimate position / velocity / acceleration"]
        T3 --> T4["Maintain boxHist / pcHist"]
        T4 --> T5["dynamic classification"]

        T5 --> T51["If is_human<br/>use as dynamic obstacle directly"]
        T5 --> T52["Otherwise compare current and history point clouds<br/>motion voting"]
        T52 --> T53["Combine with Kalman velocity threshold"]
        T53 --> T54["Multi-frame consistency check"]
        T51 --> T6["dynamicBBoxes"]
        T54 --> T6
    end

    subgraph OUTPUT["Planner output"]
        T6 --> O1["Publish DynamicObstacles"]
        O1 --> O2["/onboard_detector/dynamic_obstacles_info"]
        O2 --> O3["position[]<br/>velocity[]<br/>size[]"]
        O3 --> O4["cane_planner subscribes<br/>for dynamic avoidance"]
    end
```

## Planner Interface

LV-DOT publishes dynamic obstacles through:

```text
/onboard_detector/dynamic_obstacles_info
```

The message type is:

```text
onboard_detector/DynamicObstacles
```

The planner consumes the same interface, so the high-level connection between
LV-DOT and `cane_planner` is compatible. The message carries:

```text
position[]
velocity[]
size[]
```

## Current Defaults

The current detector configuration uses:

```text
lidar_pointcloud_topic: /cloud_registered_body
odom_topic: /Odometry
enable_lidar_detection: true
enable_depth_detection: true
enable_yolo_filtering: true
```

`run_detector.launch` also exposes lightweight runtime controls:

```text
enable_yolo
yolo_detection_rate
color_image_topic
publish_yolo_visualization
```

This allows YOLO to be disabled or retargeted without editing code.

## Point Cloud Frame Assumption

The LV-DOT LiDAR callback assumes the input point cloud is in a local
body/LiDAR-like frame, not already in the global map frame.

In `dynamicDetector::lidarOdomCB` and `dynamicDetector::lidarPoseCB`, the code:

1. Converts the input `PointCloud2` to PCL.
2. Filters points using the cloud's own local `x/y` values within
   `localLidarRange_`.
3. Applies the current LiDAR pose transform to move the cloud into the map
   frame.

Because of this, the preferred LiDAR input is:

```text
/cloud_registered_body
```

FAST-LIO publishes this topic with `frame_id = body`, which matches the current
LV-DOT assumption.

## Global Cloud Caveat

These topics are global-frame clouds:

```text
/cloud_registered
/corrected_current_pcd
```

They are not suitable as direct LV-DOT LiDAR inputs under the current callback
logic, because LV-DOT would filter them as if they were local clouds and then
apply another body-to-map transform.

Use a global cloud only after adding an explicit "cloud already in map frame"
mode that skips the local-frame filtering and transform path.

## Resolved Integration Issues

The following integration issues have been addressed in the current branch:

1. LiDAR input now defaults to `/cloud_registered_body`, matching the local cloud
   assumption.
2. `lidarOdomCB` and `lidarPoseCB` update the current LiDAR pose before
   transforming the incoming cloud, removing the previous one-frame pose lag.
3. Depth, LiDAR, YOLO filtering, debug visualization, and timer rates are exposed
   through ROS parameters.
4. YOLOv11 launch parameters now support configurable image topic, detection
   rate, visualization publishing, and launch-time disabling.

## Remaining Caveats

Full LiDAR-visual LV-DOT still requires valid RGB-D inputs:

```text
/camera/depth/image_rect_raw
/camera/color/image_raw
```

If these topics are not available, disable the unavailable branch in
`detector_param.yaml` or through launch arguments where supported.

For a full perception run, also verify:

1. Camera intrinsics match the actual RGB-D stream.
2. `body_to_camera_depth`, `body_to_camera_color`, and `body_to_lidar` extrinsics
   match the robot model.
3. YOLO runtime dependencies and `yolo11n.pt` are available.
4. `/Odometry` is consistent with the body-frame point cloud used by LV-DOT.
