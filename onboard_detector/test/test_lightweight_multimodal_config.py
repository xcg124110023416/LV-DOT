#!/usr/bin/env python3
import pathlib
import re
import unittest

import yaml


PKG = pathlib.Path(__file__).resolve().parents[1]


class LightweightMultimodalConfigTest(unittest.TestCase):
    def test_detector_yaml_exposes_mode_and_rate_controls(self):
        params = yaml.safe_load((PKG / "cfg" / "detector_param.yaml").read_text())

        self.assertTrue(params["enable_lidar_detection"])
        self.assertTrue(params["enable_depth_detection"])
        self.assertTrue(params["enable_yolo_filtering"])
        self.assertFalse(params["enable_debug_visualization"])

        self.assertGreaterEqual(params["tracking_rate"], params["lidar_detection_rate"])
        self.assertGreaterEqual(params["tracking_rate"], params["depth_detection_rate"])
        self.assertLessEqual(params["yolo_detection_rate"], params["depth_detection_rate"])
        self.assertEqual(params["lidar_pointcloud_topic"], "/cloud_registered_body")

    def test_launch_can_disable_yolo_without_changing_code(self):
        launch = (PKG / "launch" / "run_detector.launch").read_text()

        self.assertRegex(launch, r'<arg\s+name="enable_yolo"\s+default="true"')
        self.assertRegex(
            launch,
            r'<node[^>]+yolov11_detector_node.py[^>]+if="\$\(arg enable_yolo\)"',
        )
        self.assertIn("yolo_detection_rate", launch)

    def test_dynamic_detector_uses_independent_timers_and_feature_flags(self):
        header = (PKG / "include" / "onboard_detector" / "dynamicDetector.h").read_text()
        impl = (PKG / "include" / "onboard_detector" / "dynamicDetector.cpp").read_text()

        for field in (
            "enableDepthDetection_",
            "enableLidarDetection_",
            "enableYoloFiltering_",
            "enableDebugVisualization_",
            "depthDetectionDt_",
            "lidarDetectionDt_",
            "trackingDt_",
            "classificationDt_",
            "publishDt_",
        ):
            self.assertIn(field, header)

        self.assertIn("enable_depth_detection", impl)
        self.assertIn("enable_lidar_detection", impl)
        self.assertIn("enable_yolo_filtering", impl)
        self.assertIn("enable_debug_visualization", impl)
        self.assertIn("depthDetectionDt_", impl)
        self.assertIn("lidarDetectionDt_", impl)
        self.assertIn("publishTimer_", header)

        timer_durations = {
            duration.replace("this->", "")
            for duration in re.findall(r"ros::Duration\(([^)]+Dt_)\)", impl)
        }
        self.assertIn("depthDetectionDt_", timer_durations)
        self.assertIn("lidarDetectionDt_", timer_durations)
        self.assertIn("trackingDt_", timer_durations)
        self.assertIn("classificationDt_", timer_durations)
        self.assertIn("publishDt_", timer_durations)

    def test_yolo_node_reads_rate_and_topic_from_ros_params(self):
        yolo = (PKG / "scripts" / "yolo_detector" / "yolov11_detector.py").read_text()

        self.assertIn("~image_topic", yolo)
        self.assertIn("~detection_rate", yolo)
        self.assertIn("~publish_visualization", yolo)
        self.assertNotIn('img_topic = "/camera/color/image_raw"', yolo)


if __name__ == "__main__":
    unittest.main()
