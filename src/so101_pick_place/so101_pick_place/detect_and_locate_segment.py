"""
YOLO + RealSense D435 -> 3D object pose publish.
카메라 좌표를 robot `base_link` 기준으로 직접 변환한다.
RealSense를 별도 스레드에서 구동하여 ROS 콜백을 차단하지 않음.
"""

import threading
from collections import deque

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64


def _rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix → (x, y, z, w) quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class DetectAndLocate(Node):
    def __init__(self):
        super().__init__('detect_and_locate_segment')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('target_classes', ['sports ball'])
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('min_mask_pixels', 80)
        self.declare_parameter('show_debug_view', True)
        self.declare_parameter('show_pointcloud_view', True)
        self.declare_parameter('pointcloud_max_points', 1500)
        self.declare_parameter('pointcloud_update_interval', 5)
        self.declare_parameter('mask_erode_iterations', 2)
        self.declare_parameter('depth_sigma_clip', 2.0)
        self.declare_parameter('geometry_percentile_low', 5.0)
        self.declare_parameter('geometry_percentile_high', 95.0)
        self.declare_parameter('temporal_alpha', 0.25)
        self.declare_parameter('position_history_size', 5)

        # 카메라 위치 파라미터 (robot base_link 기준, m)
        self.declare_parameter('cam_x', 0.05)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', -0.09)
        self.declare_parameter('cam_pitch', 0.0)

        model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        device = self.get_parameter('device').value
        self.min_mask_pixels = self.get_parameter('min_mask_pixels').value
        self.show_debug_view = self.get_parameter('show_debug_view').value
        self.show_pointcloud_view = self.get_parameter('show_pointcloud_view').value
        self.pointcloud_max_points = self.get_parameter('pointcloud_max_points').value
        self.pointcloud_update_interval = self.get_parameter('pointcloud_update_interval').value
        self.mask_erode_iterations = self.get_parameter('mask_erode_iterations').value
        self.depth_sigma_clip = self.get_parameter('depth_sigma_clip').value
        self.geometry_percentile_low = self.get_parameter('geometry_percentile_low').value
        self.geometry_percentile_high = self.get_parameter('geometry_percentile_high').value
        self.temporal_alpha = self.get_parameter('temporal_alpha').value
        self.position_history_size = max(1, int(self.get_parameter('position_history_size').value))

        # target_classes: launch에서 string으로 올 수 있으므로 처리
        raw_classes = self.get_parameter('target_classes').value
        if isinstance(raw_classes, str):
            raw_classes = [c.strip().strip("'\"") for c in raw_classes.strip("[]").split(',') if c.strip()]
        self.target_classes = raw_classes

        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        pitch_deg = self.get_parameter('cam_pitch').value
        self._build_transform(cam_x, cam_y, cam_z, pitch_deg)

        # YOLO
        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info('YOLO model loaded.')

        # RealSense
        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        try:
            profile = self.pipeline.start(rs_config)
        except RuntimeError as e:
            self.get_logger().fatal(f'RealSense failed to start: {e}')
            raise SystemExit(1)

        self.align = rs.align(rs.stream.color)

        intr = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self.fx, self.fy = intr.fx, intr.fy
        self.ppx, self.ppy = intr.ppx, intr.ppy

        self.pub_pose = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.pub_grip = self.create_publisher(Float64, '/detected_grip_angle', 10)
        self.pub_normal = self.create_publisher(Vector3Stamped, '/detected_surface_normal', 10)
        self.pub_size = self.create_publisher(Float64, '/detected_object_size', 10)
        self.pub_thickness = self.create_publisher(Float64, '/detected_object_thickness', 10)

        self.get_logger().info(
            f'Camera at ({cam_x}, {cam_y}, {cam_z})m, pitch={pitch_deg}deg. '
            f'targets={self.target_classes}'
        )

        self._pc_frame_counter = 0
        self._pc_fig = None
        self._pc_ax = None
        self._smoothed_geometry = None
        self._surface_history = deque(maxlen=self.position_history_size)
        self._center_history = deque(maxlen=self.position_history_size)

        self._running = True
        self._thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._thread.start()

    def _build_transform(self, cam_x, cam_y, cam_z, pitch_deg):
        """
        카메라 optical frame -> base_link 변환 행렬

        optical frame: x=오른쪽, y=아래쪽, z=앞(렌즈 방향)
        base_link (ROS): x=앞, y=왼쪽, z=위
        """
        pitch_rad = np.radians(pitch_deg)

        R_opt_to_body = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ])

        R_pitch = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)],
        ])

        self.R = R_pitch @ R_opt_to_body
        self.t = np.array([cam_x, cam_y, cam_z])

    def cam_to_world(self, x_cam, y_cam, z_cam):
        p_cam = np.array([x_cam, y_cam, z_cam])
        return self.R @ p_cam + self.t

    def _pixel_to_camera(self, px, py, z_m):
        x_cam = (px - self.ppx) * z_m / self.fx
        y_cam = (py - self.ppy) * z_m / self.fy
        return np.array([x_cam, y_cam, z_m], dtype=np.float64)

    def _camera_to_pixel(self, point_cam):
        z = float(point_cam[2])
        if z <= 1e-6:
            return None
        px = int(round(self.fx * float(point_cam[0]) / z + self.ppx))
        py = int(round(self.fy * float(point_cam[1]) / z + self.ppy))
        return (px, py)

    def _mask_from_result(self, result, det_index, image_shape):
        if result.masks is None or result.masks.data is None:
            return None
        masks = result.masks.data
        if det_index >= masks.shape[0]:
            return None

        mask = masks[det_index].detach().cpu().numpy()
        mask = (mask > 0.5).astype(np.uint8) * 255
        if mask.shape != image_shape:
            mask = cv2.resize(mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST)

        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.erode(mask, kernel, iterations=max(0, int(self.mask_erode_iterations)))
        if np.count_nonzero(mask) < self.min_mask_pixels:
            return None
        return mask

    def _draw_target_detections(self, image, results):
        annotated = image.copy()
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box_index, box in enumerate(boxes):
                class_name = self.model.names[int(box.cls)]
                conf = float(box.conf)
                if self.target_classes and class_name not in self.target_classes:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f'{class_name} {conf:.2f}',
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                mask = self._mask_from_result(result, box_index, image.shape[:2])
                if mask is not None:
                    overlay = annotated.copy()
                    overlay[mask > 0] = (
                        0.6 * overlay[mask > 0] + 0.4 * np.array([0, 255, 255])
                    ).astype(np.uint8)
                    annotated = overlay
        return annotated

    @staticmethod
    def _grasp_quaternion_from_axes(approach_base, grasp_axis_base):
        approach = approach_base / np.linalg.norm(approach_base)
        grasp_axis = grasp_axis_base - np.dot(grasp_axis_base, approach) * approach
        if np.linalg.norm(grasp_axis) < 1e-6:
            fallback = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(fallback, approach)) > 0.9:
                fallback = np.array([0.0, 1.0, 0.0])
            grasp_axis = fallback - np.dot(fallback, approach) * approach

        grasp_axis /= np.linalg.norm(grasp_axis)
        lateral_axis = np.cross(approach, grasp_axis)
        lateral_axis /= np.linalg.norm(lateral_axis)
        grasp_axis = np.cross(lateral_axis, approach)
        grasp_axis /= np.linalg.norm(grasp_axis)
        R = np.column_stack([grasp_axis, lateral_axis, approach])
        return _rotation_matrix_to_quaternion(R)

    @staticmethod
    def _roll_from_axis(major_axis_base):
        angle = float(np.arctan2(major_axis_base[1], major_axis_base[0]))
        return (angle + np.pi / 2.0) % np.pi - np.pi / 2.0

    def _mask_to_point_cloud(self, depth_image, mask, sample_step=2):
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return None, None

        points_cam = []
        pixels = []
        for idx in range(0, len(xs), sample_step):
            px = int(xs[idx])
            py = int(ys[idx])
            z_m = float(depth_image[py, px]) / 1000.0
            if not (0.05 < z_m < 3.0):
                continue
            points_cam.append(self._pixel_to_camera(px, py, z_m))
            pixels.append((px, py))

        if len(points_cam) < 20:
            return None, None
        return np.array(points_cam), np.array(pixels)

    def _filter_point_cloud(self, points_cam, pixels):
        if points_cam is None or points_cam.shape[0] < 20:
            return None, None

        depths = points_cam[:, 2]
        depth_median = np.median(depths)
        depth_std = np.std(depths)
        if depth_std < 1e-6:
            return points_cam, pixels

        depth_mask = np.abs(depths - depth_median) <= self.depth_sigma_clip * depth_std
        filtered_points = points_cam[depth_mask]
        filtered_pixels = pixels[depth_mask]
        if filtered_points.shape[0] < 20:
            return None, None
        return filtered_points, filtered_pixels

    def _estimate_object_geometry(self, depth_image, mask):
        points_cam, pixels = self._mask_to_point_cloud(depth_image, mask)
        if points_cam is None:
            return None
        points_cam, pixels = self._filter_point_cloud(points_cam, pixels)
        if points_cam is None:
            return None

        valid_depths = points_cam[:, 2]
        centroid_cam = points_cam.mean(axis=0)
        centered = points_cam - centroid_cam
        _, _, vt = np.linalg.svd(centered, full_matrices=False)

        major_cam = vt[0]
        minor_cam = vt[1]
        normal_cam = vt[2]
        if normal_cam[2] > 0:
            normal_cam = -normal_cam
        if major_cam[0] < 0:
            major_cam = -major_cam
        if np.dot(np.cross(major_cam, minor_cam), normal_cam) < 0:
            minor_cam = -minor_cam

        normal_base = self.R @ normal_cam
        approach_base = self.R @ (-normal_cam)
        major_axis_base = self.R @ major_cam
        grasp_axis_base = self.R @ minor_cam

        projections = centered @ vt.T
        low = self.geometry_percentile_low
        high = self.geometry_percentile_high
        projection_center = np.array([
            np.median(projections[:, 0]),
            np.median(projections[:, 1]),
            np.median(projections[:, 2]),
        ])
        size_major = float(np.percentile(projections[:, 0], high) - np.percentile(projections[:, 0], low))
        size_minor = float(np.percentile(projections[:, 1], high) - np.percentile(projections[:, 1], low))
        thickness = float(np.percentile(projections[:, 2], high) - np.percentile(projections[:, 2], low))

        if grasp_axis_base[0] < 0:
            grasp_axis_base = -grasp_axis_base

        grasp_surface_cam = (
            centroid_cam
            + projection_center[0] * vt[0]
            + projection_center[1] * vt[1]
            + projection_center[2] * vt[2]
        )
        object_center_cam = grasp_surface_cam + 0.5 * thickness * (-normal_cam)

        grip_angle = self._roll_from_axis(grasp_axis_base)
        grasp_quaternion = self._grasp_quaternion_from_axes(approach_base, grasp_axis_base)
        grasp_surface_base = self.cam_to_world(*grasp_surface_cam)
        object_center_base = self.cam_to_world(*object_center_cam)

        return {
            'mask': mask,
            'mask_pixels': int(np.count_nonzero(mask)),
            'centroid_cam': centroid_cam,
            'grasp_surface_cam': grasp_surface_cam,
            'object_center_cam': object_center_cam,
            'grasp_surface_base': grasp_surface_base,
            'object_center_base': object_center_base,
            'centroid_base': grasp_surface_base,
            'normal_base': normal_base,
            'approach_base': approach_base,
            'major_axis_base': major_axis_base,
            'grasp_axis_base': grasp_axis_base,
            'grip_angle': grip_angle,
            'grasp_quaternion': grasp_quaternion,
            'size_major': size_major,
            'size_minor': size_minor,
            'thickness': thickness,
            'reference_depth_m': float(np.median(valid_depths)),
            'pixels': pixels,
            'points_cam': points_cam,
        }

    def _smooth_geometry(self, geometry):
        if geometry is None:
            return None

        self._surface_history.append(np.array(geometry['grasp_surface_base'], dtype=np.float64))
        self._center_history.append(np.array(geometry['object_center_base'], dtype=np.float64))
        surface_history_median = np.median(np.stack(self._surface_history, axis=0), axis=0)
        center_history_median = np.median(np.stack(self._center_history, axis=0), axis=0)

        if self._smoothed_geometry is None:
            self._smoothed_geometry = {
                'grasp_surface_base': np.array(surface_history_median, dtype=np.float64),
                'object_center_base': np.array(center_history_median, dtype=np.float64),
                'normal_base': np.array(geometry['normal_base'], dtype=np.float64),
                'approach_base': np.array(geometry['approach_base'], dtype=np.float64),
                'major_axis_base': np.array(geometry['major_axis_base'], dtype=np.float64),
                'grasp_axis_base': np.array(geometry['grasp_axis_base'], dtype=np.float64),
                'size_major': float(geometry['size_major']),
                'size_minor': float(geometry['size_minor']),
                'thickness': float(geometry['thickness']),
                'grip_angle': float(geometry['grip_angle']),
            }
        else:
            alpha = float(np.clip(self.temporal_alpha, 0.0, 1.0))
            self._smoothed_geometry['grasp_surface_base'] = (
                (1.0 - alpha) * self._smoothed_geometry['grasp_surface_base']
                + alpha * surface_history_median
            )
            self._smoothed_geometry['object_center_base'] = (
                (1.0 - alpha) * self._smoothed_geometry['object_center_base']
                + alpha * center_history_median
            )

            prev_normal = self._smoothed_geometry['normal_base']
            new_normal = np.array(geometry['normal_base'], dtype=np.float64)
            if np.dot(prev_normal, new_normal) < 0:
                new_normal = -new_normal
            blended_normal = (1.0 - alpha) * prev_normal + alpha * new_normal
            norm = np.linalg.norm(blended_normal)
            if norm > 1e-6:
                blended_normal = blended_normal / norm
            self._smoothed_geometry['normal_base'] = blended_normal

            for axis_key in ('approach_base', 'major_axis_base', 'grasp_axis_base'):
                prev_axis = self._smoothed_geometry[axis_key]
                new_axis = np.array(geometry[axis_key], dtype=np.float64)
                if np.dot(prev_axis, new_axis) < 0:
                    new_axis = -new_axis
                blended_axis = (1.0 - alpha) * prev_axis + alpha * new_axis
                axis_norm = np.linalg.norm(blended_axis)
                if axis_norm > 1e-6:
                    blended_axis = blended_axis / axis_norm
                self._smoothed_geometry[axis_key] = blended_axis

            for key in ('size_major', 'size_minor', 'thickness', 'grip_angle'):
                self._smoothed_geometry[key] = (
                    (1.0 - alpha) * self._smoothed_geometry[key]
                    + alpha * float(geometry[key])
                )

        smoothed = dict(geometry)
        smoothed['grasp_surface_base'] = self._smoothed_geometry['grasp_surface_base'].copy()
        smoothed['object_center_base'] = self._smoothed_geometry['object_center_base'].copy()
        smoothed['centroid_base'] = smoothed['grasp_surface_base'].copy()
        smoothed['normal_base'] = self._smoothed_geometry['normal_base'].copy()
        smoothed['approach_base'] = self._smoothed_geometry['approach_base'].copy()
        smoothed['major_axis_base'] = self._smoothed_geometry['major_axis_base'].copy()
        smoothed['grasp_axis_base'] = self._smoothed_geometry['grasp_axis_base'].copy()
        smoothed['size_major'] = float(self._smoothed_geometry['size_major'])
        smoothed['size_minor'] = float(self._smoothed_geometry['size_minor'])
        smoothed['thickness'] = float(self._smoothed_geometry['thickness'])
        smoothed['grip_angle'] = float(self._smoothed_geometry['grip_angle'])
        smoothed['grasp_quaternion'] = self._grasp_quaternion_from_axes(
            smoothed['approach_base'],
            smoothed['grasp_axis_base'],
        )
        return smoothed

    def _update_pointcloud_view(self, geometry):
        if not self.show_pointcloud_view or geometry is None:
            return

        self._pc_frame_counter += 1
        if self._pc_frame_counter % max(1, int(self.pointcloud_update_interval)) != 0:
            return

        points = geometry['points_cam']
        if points.shape[0] > self.pointcloud_max_points:
            indices = np.linspace(0, points.shape[0] - 1, self.pointcloud_max_points, dtype=int)
            points = points[indices]

        if self._pc_fig is None or self._pc_ax is None:
            plt.ion()
            self._pc_fig = plt.figure('Segment Point Cloud')
            self._pc_ax = self._pc_fig.add_subplot(111, projection='3d')

        self._pc_ax.cla()
        self._pc_ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=2, c=points[:, 2], cmap='viridis')
        surface_point = geometry['grasp_surface_cam']
        object_center = geometry['object_center_cam']
        self._pc_ax.scatter([surface_point[0]], [surface_point[1]], [surface_point[2]], s=60, c='m')
        self._pc_ax.scatter([object_center[0]], [object_center[1]], [object_center[2]], s=60, c='c')

        normal_cam = self.R.T @ geometry['normal_base']
        normal_norm = np.linalg.norm(normal_cam)
        if normal_norm > 1e-6:
            normal_cam = normal_cam / normal_norm
            self._pc_ax.quiver(
                surface_point[0], surface_point[1], surface_point[2],
                normal_cam[0], normal_cam[1], normal_cam[2],
                length=0.05, color='r'
            )

        major_cam = self.R.T @ geometry['major_axis_base']
        major_norm = np.linalg.norm(major_cam)
        if major_norm > 1e-6:
            major_cam = major_cam / major_norm
            self._pc_ax.quiver(
                surface_point[0], surface_point[1], surface_point[2],
                major_cam[0], major_cam[1], major_cam[2],
                length=0.05, color='g'
            )
        grasp_cam = self.R.T @ geometry['grasp_axis_base']
        grasp_norm = np.linalg.norm(grasp_cam)
        if grasp_norm > 1e-6:
            grasp_cam = grasp_cam / grasp_norm
            self._pc_ax.quiver(
                surface_point[0], surface_point[1], surface_point[2],
                grasp_cam[0], grasp_cam[1], grasp_cam[2],
                length=0.05, color='m'
            )

        self._pc_ax.set_xlabel('X_cam')
        self._pc_ax.set_ylabel('Y_cam')
        self._pc_ax.set_zlabel('Z_cam')
        self._pc_ax.set_title('Segmented Point Cloud')
        self._pc_fig.canvas.draw_idle()
        self._pc_fig.canvas.flush_events()

    def _camera_loop(self):
        """별도 스레드에서 카메라 + YOLO 추론 (ROS 콜백 비차단)"""
        while self._running and rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                continue
            if not frames:
                continue

            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            results = self.model(color_image, conf=self.confidence, verbose=False)
            annotated = self._draw_target_detections(color_image, results)

            best_det = None
            best_result = None
            best_index = None
            best_conf = 0.0

            for result in results:
                for box_index, box in enumerate(result.boxes):
                    class_name = self.model.names[int(box.cls)]
                    conf = float(box.conf)
                    if self.target_classes and class_name not in self.target_classes:
                        continue
                    if conf > best_conf:
                        best_conf = conf
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        best_det = {
                            'class': class_name,
                            'confidence': conf,
                            'cx': int((x1 + x2) / 2),
                            'cy': int((y1 + y2) / 2),
                            'x1': int(x1), 'y1': int(y1),
                            'x2': int(x2), 'y2': int(y2),
                        }
                        best_result = result
                        best_index = box_index

            if best_det and best_result is not None and best_index is not None:
                seg_mask = self._mask_from_result(best_result, best_index, depth_image.shape)
                if seg_mask is None:
                    continue

                raw_geometry = self._estimate_object_geometry(depth_image, seg_mask)
                if raw_geometry is None:
                    continue
                geometry = self._smooth_geometry(raw_geometry)

                self._update_pointcloud_view(geometry)

                surface_x, surface_y, surface_z = geometry['grasp_surface_base']
                center_x, center_y, center_z = geometry['object_center_base']
                normal = geometry['normal_base']
                grip_angle = geometry['grip_angle']
                object_size_m = geometry['size_minor']
                thickness_m = geometry['thickness']
                mask_pixels = geometry['mask_pixels']
                surface_px = self._camera_to_pixel(geometry['grasp_surface_cam'])
                center_px = self._camera_to_pixel(geometry['object_center_cam'])
                if surface_px is None:
                    surface_px = (best_det['cx'], best_det['cy'])
                if center_px is None:
                    center_px = surface_px
                cx, cy = surface_px

                if normal is not None:
                    qx, qy, qz, qw = geometry['grasp_quaternion']
                    normal_str = (
                        f'n=({normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f})'
                        f' grip={np.degrees(grip_angle):.0f}°'
                        f' size=({geometry["size_major"]:.3f},{object_size_m:.3f})m'
                        f' thick={thickness_m:.3f}m'
                    )
                else:
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                    normal_str = (
                        f'n=none size=({geometry["size_major"]:.3f},{object_size_m:.3f})m '
                        f'thick={thickness_m:.3f}m'
                    )

                self.get_logger().info(
                    f'[{best_det["class"]}] '
                    f'surface=({surface_x:.3f},{surface_y:.3f},{surface_z:.3f}) '
                    f'center=({center_x:.3f},{center_y:.3f},{center_z:.3f}) '
                    f'{normal_str} '
                    f'depth={geometry["reference_depth_m"]:.3f}m mask_px={mask_pixels} points={geometry["points_cam"].shape[0]} conf={best_conf:.2f}',
                    throttle_duration_sec=1.0,
                )

                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'base_link'
                pose.pose.position.x = float(surface_x)
                pose.pose.position.y = float(surface_y)
                pose.pose.position.z = float(surface_z)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.pub_pose.publish(pose)

                grip_msg = Float64()
                grip_msg.data = float(grip_angle)
                self.pub_grip.publish(grip_msg)

                size_msg = Float64()
                size_msg.data = float(object_size_m)
                self.pub_size.publish(size_msg)

                thickness_msg = Float64()
                thickness_msg.data = float(thickness_m)
                self.pub_thickness.publish(thickness_msg)

                if normal is not None:
                    normal_msg = Vector3Stamped()
                    normal_msg.header.stamp = pose.header.stamp
                    normal_msg.header.frame_id = 'base_link'
                    normal_msg.vector.x = float(normal[0])
                    normal_msg.vector.y = float(normal[1])
                    normal_msg.vector.z = float(normal[2])
                    self.pub_normal.publish(normal_msg)

                cv2.putText(
                    annotated,
                    f'{best_det["class"]} ({best_conf:.2f})',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
                cv2.putText(annotated, f'Surface X: {surface_x:.3f}m', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(annotated, f'Surface Y: {surface_y:.3f}m', (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(annotated, f'Surface Z: {surface_z:.3f}m', (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(annotated, f'Center Z: {center_z:.3f}m', (10, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(annotated, f'Mask px: {mask_pixels}', (10, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated, f'Points: {geometry["points_cam"].shape[0]}', (10, 210),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated, f'Size: {geometry["size_major"]:.3f}/{object_size_m:.3f}m', (10, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated, f'Thickness: {thickness_m:.3f}m', (10, 270),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(annotated, f'Grasp angle: {np.degrees(grip_angle):.0f} deg', (10, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(annotated, surface_px, 5, (255, 0, 255), -1)
                cv2.circle(annotated, center_px, 5, (255, 255, 0), -1)
                cv2.line(annotated, surface_px, center_px, (255, 255, 0), 2)
                cv2.putText(annotated, 'surface', (surface_px[0] + 6, surface_px[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                cv2.putText(annotated, 'center', (center_px[0] + 6, center_px[1] + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                if normal is not None:
                    n_cam = self.R.T @ normal
                    arrow_len = 60
                    nx2d = n_cam[0]
                    ny2d = n_cam[1]
                    norm2d = np.sqrt(nx2d**2 + ny2d**2)
                    if norm2d > 1e-6:
                        nx2d /= norm2d
                        ny2d /= norm2d
                    ex = int(cx + nx2d * arrow_len)
                    ey = int(cy + ny2d * arrow_len)
                    cv2.arrowedLine(annotated, (cx, cy), (ex, ey),
                                    (255, 100, 0), 2, tipLength=0.3)
                    cv2.putText(annotated, 'normal', (ex + 5, ey),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

                    major_cam = self.R.T @ geometry['major_axis_base']
                    major_2d = major_cam[:2]
                    major_2d_norm = np.linalg.norm(major_2d)
                    if major_2d_norm > 1e-6:
                        major_2d = major_2d / major_2d_norm
                    mex = int(cx + major_2d[0] * arrow_len)
                    mey = int(cy + major_2d[1] * arrow_len)
                    cv2.arrowedLine(annotated, (cx, cy), (mex, mey),
                                    (0, 255, 0), 2, tipLength=0.3)
                    cv2.putText(annotated, 'major', (mex + 5, mey),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                    grasp_cam = self.R.T @ geometry['grasp_axis_base']
                    grasp_2d = grasp_cam[:2]
                    grasp_2d_norm = np.linalg.norm(grasp_2d)
                    if grasp_2d_norm > 1e-6:
                        grasp_2d = grasp_2d / grasp_2d_norm
                    gx = grasp_2d[0]
                    gy = grasp_2d[1]
                    gex = int(cx + gx * arrow_len)
                    gey = int(cy + gy * arrow_len)
                    cv2.arrowedLine(annotated, (cx, cy), (gex, gey),
                                    (0, 0, 255), 2, tipLength=0.3)
                    cv2.putText(annotated, 'grasp', (gex + 5, gey),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                    cv2.putText(annotated, normal_str, (10, 330),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)

            if self.show_debug_view:
                cv2.imshow('YOLO + RealSense', annotated)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.get_logger().info('Shutting down (q pressed)...')
                    self._running = False
                    return

    def destroy_node(self):
        self._running = False
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self._pc_fig is not None:
            plt.close(self._pc_fig)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectAndLocate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
