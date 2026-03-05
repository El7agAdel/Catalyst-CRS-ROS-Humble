#!/usr/bin/env python3
import math
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def make_sensor_qos():
    # Gazebo image streams are typically BEST_EFFORT.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("topic", "/cam/rgb/image_raw")
        self.declare_parameter("window_name", "Camera Feed")
        self.declare_parameter("depth", False)  # set True for depth topics
        self.declare_parameter("fps", 30.0)  # GUI refresh
        self.declare_parameter("print_encoding_once", True)

        # Stereo mode parameters.
        self.declare_parameter("stereo_mode", False)
        self.declare_parameter("combined_mode", False)
        self.declare_parameter("left_topic", "/cam/stereo/left/image_raw")
        self.declare_parameter("right_topic", "/cam/stereo/right/image_raw")
        self.declare_parameter("rgb_topic", "/cam/rgb/image_raw")
        self.declare_parameter("depth_topic", "/cam/depth/image_raw")
        self.declare_parameter("stereo_max_dt_sec", 0.05)
        self.declare_parameter("stereo_baseline_m", 0.06)  # From camera.sdf
        self.declare_parameter("stereo_hfov_rad", 1.047)  # From camera.sdf
        self.declare_parameter("stereo_focal_px", 0.0)  # <=0 => derive from width+hfov
        self.declare_parameter("stereo_min_disparity", 0)
        self.declare_parameter("stereo_num_disparities", 128)  # Must be multiple of 16
        self.declare_parameter("stereo_block_size", 7)  # Odd
        self.declare_parameter("stereo_uniqueness_ratio", 10)
        self.declare_parameter("stereo_speckle_window_size", 100)
        self.declare_parameter("stereo_speckle_range", 2)
        self.declare_parameter("stereo_disp12_max_diff", 1)

        self.topic = self.get_parameter("topic").value
        self.window_name = self.get_parameter("window_name").value
        self.is_depth = bool(self.get_parameter("depth").value)
        self.fps = float(self.get_parameter("fps").value)
        self.print_encoding_once = bool(self.get_parameter("print_encoding_once").value)

        self.stereo_mode = bool(self.get_parameter("stereo_mode").value)
        self.combined_mode = bool(self.get_parameter("combined_mode").value)
        self.left_topic = self.get_parameter("left_topic").value
        self.right_topic = self.get_parameter("right_topic").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.stereo_max_dt_sec = float(self.get_parameter("stereo_max_dt_sec").value)
        self.stereo_baseline_m = float(self.get_parameter("stereo_baseline_m").value)
        self.stereo_hfov_rad = float(self.get_parameter("stereo_hfov_rad").value)
        self.stereo_focal_px = float(self.get_parameter("stereo_focal_px").value)
        self.stereo_min_disparity = int(self.get_parameter("stereo_min_disparity").value)
        self.stereo_num_disparities = int(self.get_parameter("stereo_num_disparities").value)
        self.stereo_block_size = int(self.get_parameter("stereo_block_size").value)
        self.stereo_uniqueness_ratio = int(self.get_parameter("stereo_uniqueness_ratio").value)
        self.stereo_speckle_window_size = int(self.get_parameter("stereo_speckle_window_size").value)
        self.stereo_speckle_range = int(self.get_parameter("stereo_speckle_range").value)
        self.stereo_disp12_max_diff = int(self.get_parameter("stereo_disp12_max_diff").value)

        self.bridge = CvBridge()
        self.last_frame = None
        self._printed = False

        # Stereo buffers.
        self.left_frame: Optional[np.ndarray] = None
        self.right_frame: Optional[np.ndarray] = None
        self.rgb_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.left_stamp_sec: Optional[float] = None
        self.right_stamp_sec: Optional[float] = None
        self._left_printed = False
        self._right_printed = False
        self._rgb_printed = False
        self._depth_printed = False
        self._auto_focal_px: Optional[float] = None
        self.stereo_matcher = None

        if self.combined_mode and self.stereo_mode:
            self.get_logger().warn("Both combined_mode and stereo_mode are true; using combined_mode.")

        if self.combined_mode:
            self.sub_left = self.create_subscription(
                Image, self.left_topic, self._left_cb, make_sensor_qos()
            )
            self.sub_right = self.create_subscription(
                Image, self.right_topic, self._right_cb, make_sensor_qos()
            )
            self.sub_rgb = self.create_subscription(
                Image, self.rgb_topic, self._rgb_cb, make_sensor_qos()
            )
            self.sub_depth = self.create_subscription(
                Image, self.depth_topic, self._depth_cb, make_sensor_qos()
            )
        elif self.stereo_mode:
            self._init_stereo_matcher()
            self.sub_left = self.create_subscription(
                Image, self.left_topic, self._left_cb, make_sensor_qos()
            )
            self.sub_right = self.create_subscription(
                Image, self.right_topic, self._right_cb, make_sensor_qos()
            )
        else:
            self.sub = self.create_subscription(Image, self.topic, self.cb, make_sensor_qos())

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.combined_mode:
            self.get_logger().info(
                "Combined mode enabled. Stereo scene + RGB + Depth dashboard is active."
            )
            self.get_logger().info(
                f"Left: {self.left_topic}, Right: {self.right_topic}, "
                f"RGB: {self.rgb_topic}, Depth: {self.depth_topic}"
            )
        elif self.stereo_mode:
            self.get_logger().info(
                f"Stereo mode enabled. Left: {self.left_topic}, Right: {self.right_topic}"
            )
            self.get_logger().info(
                f"Stereo baseline={self.stereo_baseline_m:.4f} m, "
                f"hfov={self.stereo_hfov_rad:.4f} rad, focal_px={self.stereo_focal_px:.2f} "
                "(<=0 means auto-calc)"
            )
        else:
            self.get_logger().info(f"Subscribed to: {self.topic} (depth={self.is_depth})")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

    def cb(self, msg: Image):
        if self.stereo_mode or self.combined_mode:
            return

        enc = (msg.encoding or "").lower()

        if self.print_encoding_once and not self._printed:
            self._printed = True
            self.get_logger().info(
                f"First frame encoding: '{msg.encoding}', {msg.width}x{msg.height}, step={msg.step}"
            )

        try:
            # For RGB feeds, directly ask for bgr8 (more reliable for display).
            if self.is_depth or enc in ("16uc1", "32fc1", "64fc1"):
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.last_frame = self._depth_to_view(img)
            else:
                # Many Gazebo cameras publish rgb8/rgba8; bgr8 conversion is safest for OpenCV.
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.last_frame = img
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            self.last_frame = None

    def _init_stereo_matcher(self):
        num_disp = max(16, self.stereo_num_disparities)
        num_disp = int(math.ceil(num_disp / 16.0) * 16)

        block_size = max(5, self.stereo_block_size)
        if block_size % 2 == 0:
            block_size += 1

        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=self.stereo_min_disparity,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=self.stereo_disp12_max_diff,
            uniquenessRatio=self.stereo_uniqueness_ratio,
            speckleWindowSize=self.stereo_speckle_window_size,
            speckleRange=self.stereo_speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.get_logger().info(
            f"Stereo matcher config: min_disp={self.stereo_min_disparity}, "
            f"num_disp={num_disp}, block={block_size}"
        )

    def _left_cb(self, msg: Image):
        try:
            self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.left_stamp_sec = self._stamp_to_sec(msg)
            if self.print_encoding_once and not self._left_printed:
                self._left_printed = True
                self.get_logger().info(
                    f"Left encoding: '{msg.encoding}', {msg.width}x{msg.height}, step={msg.step}"
                )
            if self.combined_mode:
                self._try_update_combined_preview()
            else:
                self._try_update_stereo_preview()
        except Exception as e:
            self.get_logger().warn(f"Left conversion failed: {e}")

    def _right_cb(self, msg: Image):
        try:
            self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.right_stamp_sec = self._stamp_to_sec(msg)
            if self.print_encoding_once and not self._right_printed:
                self._right_printed = True
                self.get_logger().info(
                    f"Right encoding: '{msg.encoding}', {msg.width}x{msg.height}, step={msg.step}"
                )
            if self.combined_mode:
                self._try_update_combined_preview()
            else:
                self._try_update_stereo_preview()
        except Exception as e:
            self.get_logger().warn(f"Right conversion failed: {e}")

    def _rgb_cb(self, msg: Image):
        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.print_encoding_once and not self._rgb_printed:
                self._rgb_printed = True
                self.get_logger().info(
                    f"RGB encoding: '{msg.encoding}', {msg.width}x{msg.height}, step={msg.step}"
                )
            self._try_update_combined_preview()
        except Exception as e:
            self.get_logger().warn(f"RGB conversion failed: {e}")

    def _depth_cb(self, msg: Image):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_frame = self._depth_to_view(depth_raw)
            if self.print_encoding_once and not self._depth_printed:
                self._depth_printed = True
                self.get_logger().info(
                    f"Depth encoding: '{msg.encoding}', {msg.width}x{msg.height}, step={msg.step}"
                )
            self._try_update_combined_preview()
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def _try_update_combined_preview(self):
        panel_w = 640
        panel_h = 480

        left_panel = self._to_panel(self.left_frame, panel_w, panel_h, "Stereo Left")
        right_panel = self._to_panel(self.right_frame, panel_w, panel_h, "Stereo Right")
        rgb_panel = self._to_panel(self.rgb_frame, panel_w, panel_h, "RGB")
        depth_panel = self._to_panel(self.depth_frame, panel_w, panel_h, "Depth")

        stereo_scene = np.hstack([left_panel, right_panel])
        bottom_row = np.hstack([rgb_panel, depth_panel])
        self.last_frame = np.vstack([stereo_scene, bottom_row])

    def _try_update_stereo_preview(self):
        if self.left_frame is None or self.right_frame is None or self.stereo_matcher is None:
            return

        if self.left_stamp_sec is not None and self.right_stamp_sec is not None:
            if abs(self.left_stamp_sec - self.right_stamp_sec) > self.stereo_max_dt_sec:
                return

        left = self.left_frame
        right = self.right_frame

        if left.shape[:2] != right.shape[:2]:
            right = cv2.resize(right, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_LINEAR)

        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        valid = disparity > float(self.stereo_min_disparity)

        focal_px = self.stereo_focal_px
        if focal_px <= 0.0:
            if self._auto_focal_px is None:
                width = float(left.shape[1])
                hfov = min(max(self.stereo_hfov_rad, 1e-3), math.pi - 1e-3)
                self._auto_focal_px = width / (2.0 * math.tan(hfov / 2.0))
                self.get_logger().info(f"Auto focal estimate from hfov+width: {self._auto_focal_px:.2f} px")
            focal_px = self._auto_focal_px

        baseline_m = max(abs(self.stereo_baseline_m), 1e-6)
        depth_m = np.zeros_like(disparity, dtype=np.float32)
        denom = disparity.copy()
        denom[~valid] = 1.0
        depth_m[valid] = (focal_px * baseline_m) / denom[valid]
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < 0.0] = 0.0

        disp_view = self._disparity_to_view(disparity, valid)
        depth_view = self._depth_to_view(depth_m)

        left_view = left.copy()
        right_view = right.copy()
        self._annotate(left_view, "Left")
        self._annotate(right_view, "Right")
        self._annotate(disp_view, "Disparity")
        self._annotate(depth_view, "Depth (m)")

        self.last_frame = np.vstack(
            [
                np.hstack([left_view, right_view]),
                np.hstack([disp_view, depth_view]),
            ]
        )

    @staticmethod
    def _stamp_to_sec(msg: Image) -> Optional[float]:
        if not hasattr(msg, "header") or msg.header is None:
            return None
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _annotate(img, text):
        cv2.putText(
            img,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _to_panel(self, frame: Optional[np.ndarray], width: int, height: int, label: str):
        if frame is None:
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                f"Waiting for {label}...",
                (20, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._annotate(canvas, label)
            return canvas

        panel = frame
        if panel.ndim == 2:
            panel = cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)

        panel = cv2.resize(panel, (width, height), interpolation=cv2.INTER_LINEAR)
        panel = np.ascontiguousarray(panel, dtype=np.uint8)
        self._annotate(panel, label)
        return panel

    def _disparity_to_view(self, disparity, valid_mask):
        if disparity.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        view = np.zeros_like(disparity, dtype=np.float32)
        if np.any(valid_mask):
            valid = disparity[valid_mask]
            vmin, vmax = np.percentile(valid, [5, 95])
            if math.isclose(vmin, vmax):
                vmax = vmin + 1.0
            view[valid_mask] = np.clip((disparity[valid_mask] - vmin) / (vmax - vmin), 0.0, 1.0)

        view8 = (view * 255.0).astype(np.uint8)
        return cv2.applyColorMap(view8, cv2.COLORMAP_TURBO)

    def _depth_to_view(self, depth_img):
        depth = np.array(depth_img, dtype=np.float32, copy=False)
        depth[~np.isfinite(depth)] = 0.0

        if depth.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        valid = depth > 0.0
        if not np.any(valid):
            return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

        vmin, vmax = np.percentile(depth[valid], [5, 95])
        if math.isclose(vmin, vmax):
            vmax = vmin + 1.0

        norm = np.clip((depth - vmin) / (vmax - vmin), 0.0, 1.0)
        view8 = (norm * 255.0).astype(np.uint8)
        return cv2.applyColorMap(view8, cv2.COLORMAP_JET)

    def gui_tick(self, delay_ms=1):
        """Call frequently from the main thread to keep OpenCV responsive."""
        if self.last_frame is None:
            # show a placeholder instead of a frozen/black window
            canvas = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                "Waiting for frames...",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
            )
            if self.combined_mode:
                info = f"L: {self.left_topic}"
                info2 = f"R: {self.right_topic}"
                info3 = f"RGB: {self.rgb_topic}"
                info4 = f"Depth: {self.depth_topic}"
            elif self.stereo_mode:
                info = f"L: {self.left_topic}"
                info2 = f"R: {self.right_topic}"
                info3 = ""
                info4 = ""
            else:
                info = f"Topic: {self.topic}"
                info2 = ""
                info3 = ""
                info4 = ""
            cv2.putText(
                canvas,
                info,
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            if info2:
                cv2.putText(
                    canvas,
                    info2,
                    (20, 145),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
            if info3:
                cv2.putText(
                    canvas,
                    info3,
                    (20, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
            if info4:
                cv2.putText(
                    canvas,
                    info4,
                    (20, 215),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
            cv2.imshow(self.window_name, canvas)
        else:
            cv2.imshow(self.window_name, self.last_frame)

        key = cv2.waitKey(max(1, int(delay_ms))) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraViewer()

    # Manual spin loop so GUI stays on main thread and remains responsive.
    period = 1.0 / max(node.fps, 1.0)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.gui_tick(delay_ms=int(period * 1000))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
