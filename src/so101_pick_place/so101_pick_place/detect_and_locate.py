"""
YOLO + RealSense D435 -> 3D object pose publish.
기본 노드는 bbox 중심 + depth ROI 기반의 단순하고 안정적인 검출 경로를 유지한다.
세그먼트 기반 실험은 detect_and_locate_segment.py에서 별도로 진행한다.
"""

import threading

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float64


def _rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
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
        super().__init__('detect_and_locate')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('target_classes', ['sports ball'])
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('show_debug_view', True)

        # 카메라 위치 파라미터 (robot base_link 기준, m)
        self.declare_parameter('cam_x', 0.05)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', -0.09)
        self.declare_parameter('cam_pitch', 0.0)

        model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        device = self.get_parameter('device').value
        self.show_debug_view = self.get_parameter('show_debug_view').value

        raw_classes = self.get_parameter('target_classes').value
        if isinstance(raw_classes, str):
            raw_classes = [c.strip().strip("'\"") for c in raw_classes.strip("[]").split(',') if c.strip()]
        self.target_classes = raw_classes

        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        pitch_deg = self.get_parameter('cam_pitch').value
        self._build_transform(cam_x, cam_y, cam_z, pitch_deg)

        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info('YOLO model loaded.')

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
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy = intr.fx, intr.fy
        self.ppx, self.ppy = intr.ppx, intr.ppy

        self.pub_pose = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.pub_grip = self.create_publisher(Float64, '/detected_grip_angle', 10)
        self.pub_normal = self.create_publisher(Vector3Stamped, '/detected_surface_normal', 10)
        self.pub_size = self.create_publisher(Float64, '/detected_object_size', 10)

        self.get_logger().info(
            f'Camera at ({cam_x}, {cam_y}, {cam_z})m, pitch={pitch_deg}deg. '
            f'targets={self.target_classes}'
        )

        self._running = True
        self._thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._thread.start()

    def _build_transform(self, cam_x, cam_y, cam_z, pitch_deg):
        """카메라 optical frame -> base_link 변환 행렬."""
        pitch_rad = np.radians(pitch_deg)
        R_opt_to_body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
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

    def _compute_surface_normal(self, depth_image, cx, cy, patch_size=25):
        """객체 중심 주변 depth patch에서 surface normal을 계산한다."""
        h, w = depth_image.shape
        x1 = max(0, cx - patch_size)
        x2 = min(w, cx + patch_size)
        y1 = max(0, cy - patch_size)
        y2 = min(h, cy + patch_size)

        points = []
        for py in range(y1, y2, 2):
            for px in range(x1, x2, 2):
                d = depth_image[py, px]
                if d <= 0:
                    continue
                z = float(d) / 1000.0
                if not (0.05 < z < 3.0):
                    continue
                x_c = (px - self.ppx) * z / self.fx
                y_c = (py - self.ppy) * z / self.fy
                points.append([x_c, y_c, z])

        if len(points) < 10:
            return None

        points = np.array(points)
        centered = points - points.mean(axis=0)
        _, _, vt = np.linalg.svd(centered)
        normal_cam = vt[2]
        if normal_cam[2] > 0:
            normal_cam = -normal_cam
        return self.R @ normal_cam

    @staticmethod
    def _normal_to_quaternion(normal):
        """법선벡터만으로 approach orientation 결정."""
        approach = -normal / np.linalg.norm(normal)
        world_up = np.array([0.0, 0.0, 1.0])
        world_ref = np.array([1.0, 0.0, 0.0]) if abs(np.dot(approach, world_up)) > 0.99 else world_up

        x_axis = np.cross(world_ref, approach)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(approach, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        R = np.column_stack([x_axis, y_axis, approach])
        return _rotation_matrix_to_quaternion(R)

    def _compute_grip_angle(self, depth_image, det):
        """bbox 내 depth 유효 픽셀에서 PCA로 물체의 2D 주축 각도를 구한다."""
        x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
        h, w = depth_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        roi = depth_image[y1:y2, x1:x2]
        ys, xs = np.where(roi > 0)
        if len(xs) < 20:
            return 0.0

        xs = xs + x1
        ys = ys + y1
        coords = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
        centered = coords - coords.mean(axis=0)
        cov = np.cov(centered.T)
        _, eigenvectors = np.linalg.eigh(cov)
        major_axis = eigenvectors[:, 1]
        angle = np.arctan2(major_axis[1], major_axis[0])
        return (angle + np.pi / 2.0) % np.pi - np.pi / 2.0

    def _estimate_object_size(self, det, z_m):
        """bbox 크기와 depth를 이용해 대략적인 실제 크기(m)를 추정한다."""
        bbox_w_px = max(1, det['x2'] - det['x1'])
        bbox_h_px = max(1, det['y2'] - det['y1'])
        width_m = bbox_w_px * z_m / self.fx
        height_m = bbox_h_px * z_m / self.fy
        return min(width_m, height_m)

    def _camera_loop(self):
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
            annotated = results[0].plot()

            best_det = None
            best_conf = 0.0
            for result in results:
                for box in result.boxes:
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

            if best_det is not None:
                cx, cy = best_det['cx'], best_det['cy']
                roi_size = 10
                h, w = depth_image.shape
                x1r = max(0, cx - roi_size)
                x2r = min(w, cx + roi_size)
                y1r = max(0, cy - roi_size)
                y2r = min(h, cy + roi_size)
                roi = depth_image[y1r:y2r, x1r:x2r].astype(np.float32)
                valid = roi[roi > 0]

                if valid.size > 0:
                    z_m = float(np.median(valid)) / 1000.0
                    if 0.1 < z_m < 3.0:
                        x_cam = (cx - self.ppx) * z_m / self.fx
                        y_cam = (cy - self.ppy) * z_m / self.fy
                        wx, wy, wz = self.cam_to_world(x_cam, y_cam, z_m)

                        normal = self._compute_surface_normal(depth_image, cx, cy)
                        grip_angle = self._compute_grip_angle(depth_image, best_det)
                        object_size_m = self._estimate_object_size(best_det, z_m)

                        if normal is not None:
                            qx, qy, qz, qw = self._normal_to_quaternion(normal)
                            normal_str = (
                                f'n=({normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f}) '
                                f'grip={np.degrees(grip_angle):.0f}° size={object_size_m:.3f}m'
                            )
                        else:
                            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                            normal_str = f'n=none size={object_size_m:.3f}m'

                        self.get_logger().info(
                            f'[{best_det["class"]}] base=({wx:.3f},{wy:.3f},{wz:.3f}) '
                            f'{normal_str} depth={z_m:.3f}m conf={best_conf:.2f}',
                            throttle_duration_sec=1.0,
                        )

                        pose = PoseStamped()
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.header.frame_id = 'base_link'
                        pose.pose.position.x = float(wx)
                        pose.pose.position.y = float(wy)
                        pose.pose.position.z = float(wz)
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

                        if normal is not None:
                            normal_msg = Vector3Stamped()
                            normal_msg.header.stamp = pose.header.stamp
                            normal_msg.header.frame_id = 'base_link'
                            normal_msg.vector.x = float(normal[0])
                            normal_msg.vector.y = float(normal[1])
                            normal_msg.vector.z = float(normal[2])
                            self.pub_normal.publish(normal_msg)

                        cv2.putText(annotated, f'{best_det["class"]} ({best_conf:.2f})', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated, f'X: {wx:.3f}m', (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated, f'Y: {wy:.3f}m', (10, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated, f'Z: {wz:.3f}m', (10, 120),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated, f'Size: {object_size_m:.3f}m', (10, 150),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)

                        if normal is not None:
                            n_cam = self.R.T @ normal
                            arrow_len = 60
                            nx2d, ny2d = n_cam[0], n_cam[1]
                            norm2d = np.sqrt(nx2d ** 2 + ny2d ** 2)
                            if norm2d > 1e-6:
                                nx2d /= norm2d
                                ny2d /= norm2d
                            ex = int(cx + nx2d * arrow_len)
                            ey = int(cy + ny2d * arrow_len)
                            cv2.arrowedLine(annotated, (cx, cy), (ex, ey), (255, 100, 0), 2, tipLength=0.3)
                            cv2.putText(annotated, 'normal', (ex + 5, ey),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

                            gx = np.sin(grip_angle)
                            gy = -np.cos(grip_angle)
                            gex = int(cx + gx * arrow_len)
                            gey = int(cy + gy * arrow_len)
                            cv2.arrowedLine(annotated, (cx, cy), (gex, gey), (0, 0, 255), 2, tipLength=0.3)
                            cv2.putText(annotated, 'grip', (gex + 5, gey),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                            cv2.putText(annotated, normal_str, (10, 180),
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
