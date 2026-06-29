from __future__ import annotations

import copy
from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import tf2_ros


class AlignedRealsenseNode(Node):
    def __init__(self) -> None:
        super().__init__('realsense_sim_driver')

        self._declare_parameters()
        self.bridge = CvBridge()
        self.rgb_info: Optional[CameraInfo] = None
        self.depth_info: Optional[CameraInfo] = None
        self._last_warn_ns = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        input_qos = self._make_qos_profile('input_qos_reliability')
        output_qos = self._make_qos_profile('output_qos_reliability')

        self.color_image_pub = self.create_publisher(
            Image, self._param('output_color_image_topic'), output_qos)
        self.color_info_pub = self.create_publisher(
            CameraInfo, self._param('output_color_info_topic'), output_qos)
        self.depth_image_pub = self.create_publisher(
            Image, self._param('output_depth_image_topic'), output_qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, self._param('output_depth_info_topic'), output_qos)
        self.aligned_depth_image_pub = self.create_publisher(
            Image, self._param('output_aligned_depth_image_topic'), output_qos)
        self.aligned_depth_info_pub = self.create_publisher(
            CameraInfo, self._param('output_aligned_depth_info_topic'), output_qos)
        self.points_pub = self.create_publisher(
            PointCloud2, self._param('output_points_topic'), output_qos)

        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            self._param('input_rgb_info_topic'),
            self._on_rgb_info,
            input_qos,
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self._param('input_depth_info_topic'),
            self._on_depth_info,
            input_qos,
        )

        self.rgb_sub = Subscriber(
            self, Image, self._param('input_rgb_topic'), qos_profile=input_qos)
        self.depth_sub = Subscriber(
            self, Image, self._param('input_depth_topic'), qos_profile=input_qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            int(self._param('queue_size')),
            float(self._param('sync_slop_sec')),
        )
        self.sync.registerCallback(self._on_image_pair)

        self.get_logger().info(
            'realsense_sim_driver started: '
            f"{self._param('input_rgb_topic')}, "
            f"{self._param('input_depth_topic')} -> "
            f"{self._param('output_color_image_topic')}, "
            f"{self._param('output_depth_image_topic')}, "
            f"{self._param('output_aligned_depth_image_topic')}, "
            f"{self._param('output_points_topic')}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('input_rgb_topic', '/rgb')
        self.declare_parameter('input_depth_topic', '/depth')
        self.declare_parameter('input_rgb_info_topic', '/rgb_cam_info')
        self.declare_parameter('input_depth_info_topic', '/depth_cam_info')
        self.declare_parameter('output_color_image_topic', '/color/image_raw')
        self.declare_parameter('output_color_info_topic', '/color/camera_info')
        self.declare_parameter('output_depth_image_topic', '/depth/image_rect_raw')
        self.declare_parameter('output_depth_info_topic', '/depth/camera_info')
        self.declare_parameter(
            'output_aligned_depth_image_topic',
            '/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'output_aligned_depth_info_topic',
            '/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('output_points_topic', '/depth/color/points')
        self.declare_parameter('output_depth_encoding', '16UC1')
        self.declare_parameter('uint16_depth_scale', 0.001)
        self.declare_parameter('float_depth_scale', 1.0)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('input_qos_reliability', 'best_effort')
        self.declare_parameter('output_qos_reliability', 'reliable')
        self.declare_parameter('pointcloud_stride', 1)
        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('publish_camera_info', True)
        self.declare_parameter('use_tf', True)
        self.declare_parameter('use_latest_tf', True)
        self.declare_parameter('lookup_tf_timeout_sec', 0.05)
        self.declare_parameter('assume_identity_tf', True)

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _make_qos_profile(self, reliability_param: str) -> QoSProfile:
        reliability = str(self._param(reliability_param)).lower()
        if reliability in ('best_effort', 'best-effort', 'besteffort'):
            reliability_policy = ReliabilityPolicy.BEST_EFFORT
        elif reliability == 'reliable':
            reliability_policy = ReliabilityPolicy.RELIABLE
        elif reliability in ('system_default', 'default'):
            reliability_policy = ReliabilityPolicy.SYSTEM_DEFAULT
        else:
            raise ValueError(
                f"{reliability_param} must be reliable, best_effort, or system_default")

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self._param('qos_depth')),
            reliability=reliability_policy,
        )

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self.rgb_info = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self.depth_info = msg

    def _on_image_pair(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self.rgb_info is None or self.depth_info is None:
            self._warn_throttled(
                'camera_info',
                'Waiting for rgb_cam_info and depth_cam_info before aligning.',
            )
            return

        try:
            depth_m = self._depth_msg_to_meters(depth_msg)
            aligned_depth_m = self._align_depth_to_rgb(
                depth_m,
                depth_msg,
                self.depth_info,
                self.rgb_info,
            )

            color_header = copy.deepcopy(rgb_msg.header)
            color_frame = self._frame_id(self.rgb_info, rgb_msg)
            if color_frame:
                color_header.frame_id = color_frame

            depth_header = copy.deepcopy(depth_msg.header)
            depth_frame = self._frame_id(self.depth_info, depth_msg)
            if depth_frame:
                depth_header.frame_id = depth_frame

            color_out = copy.deepcopy(rgb_msg)
            color_out.header = color_header
            self.color_image_pub.publish(color_out)

            depth_out = self._make_depth_image(
                depth_m,
                depth_msg.encoding,
                depth_header,
            )
            self.depth_image_pub.publish(depth_out)

            aligned_depth_out = self._make_depth_image(
                aligned_depth_m,
                depth_msg.encoding,
                color_header,
            )
            self.aligned_depth_image_pub.publish(aligned_depth_out)

            if bool(self._param('publish_camera_info')):
                self.color_info_pub.publish(
                    self._make_camera_info(self.rgb_info, color_header))
                self.depth_info_pub.publish(
                    self._make_camera_info(self.depth_info, depth_header))
                self.aligned_depth_info_pub.publish(
                    self._make_camera_info(self.rgb_info, color_header))

            if bool(self._param('publish_pointcloud')):
                rgb_image = self._rgb_image_to_array(rgb_msg)
                points_msg = self._make_point_cloud(
                    aligned_depth_m,
                    rgb_image,
                    self.rgb_info,
                    color_header,
                )
                self.points_pub.publish(points_msg)
        except Exception as exc:
            self._warn_throttled('processing', f'Frame skipped: {exc}')

    def _align_depth_to_rgb(
        self,
        depth_m: np.ndarray,
        depth_msg: Image,
        depth_info: CameraInfo,
        rgb_info: CameraInfo,
    ) -> np.ndarray:
        rgb_height = int(rgb_info.height or depth_msg.height)
        rgb_width = int(rgb_info.width or depth_msg.width)
        if rgb_height <= 0 or rgb_width <= 0:
            raise ValueError('RGB CameraInfo width/height are invalid.')

        dfx, dfy, dcx, dcy = self._intrinsics(depth_info)
        rfx, rfy, rcx, rcy = self._intrinsics(rgb_info)
        min_depth = float(self._param('min_depth_m'))
        max_depth = float(self._param('max_depth_m'))

        valid = (
            np.isfinite(depth_m)
            & (depth_m >= min_depth)
            & (depth_m <= max_depth)
        )
        if not np.any(valid):
            return np.zeros((rgb_height, rgb_width), dtype=np.float32)

        v_depth, u_depth = np.nonzero(valid)
        z_depth = depth_m[v_depth, u_depth].astype(np.float32)
        x_depth = ((u_depth.astype(np.float32) - dcx) / dfx) * z_depth
        y_depth = ((v_depth.astype(np.float32) - dcy) / dfy) * z_depth
        points_depth = np.vstack((x_depth, y_depth, z_depth))

        rgb_frame = self._frame_id(rgb_info)
        depth_frame = self._frame_id(depth_info, depth_msg)
        rotation, translation = self._lookup_depth_to_rgb(
            rgb_frame,
            depth_frame,
            depth_msg.header.stamp,
        )
        self.get_logger().info(f"rotation: {rotation}, translation: {translation}")
        points_rgb = rotation @ points_depth + translation.reshape(3, 1)

        z_rgb = points_rgb[2, :]
        finite_rgb = (
            np.isfinite(z_rgb)
            & (z_rgb >= min_depth)
            & (z_rgb <= max_depth)
        )
        if not np.any(finite_rgb):
            return np.zeros((rgb_height, rgb_width), dtype=np.float32)

        points_rgb = points_rgb[:, finite_rgb]
        z_rgb = points_rgb[2, :]
        u_rgb = np.rint((points_rgb[0, :] * rfx / z_rgb) + rcx).astype(np.int32)
        v_rgb = np.rint((points_rgb[1, :] * rfy / z_rgb) + rcy).astype(np.int32)

        in_bounds = (
            (u_rgb >= 0)
            & (u_rgb < rgb_width)
            & (v_rgb >= 0)
            & (v_rgb < rgb_height)
        )
        if not np.any(in_bounds):
            return np.zeros((rgb_height, rgb_width), dtype=np.float32)

        flat_index = (v_rgb[in_bounds] * rgb_width + u_rgb[in_bounds])
        projected_z = z_rgb[in_bounds].astype(np.float32)
        flat_depth = np.full(rgb_height * rgb_width, np.inf, dtype=np.float32)
        np.minimum.at(flat_depth, flat_index, projected_z)
        aligned = flat_depth.reshape((rgb_height, rgb_width))
        aligned[~np.isfinite(aligned)] = 0.0
        return aligned

    def _depth_msg_to_meters(self, depth_msg: Image) -> np.ndarray:
        depth_raw = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough')
        return self._depth_to_meters(depth_raw, depth_msg.encoding)

    def _depth_to_meters(self, depth_raw: np.ndarray, encoding: str) -> np.ndarray:
        depth = np.asarray(depth_raw)
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]

        normalized_encoding = encoding.upper()
        if normalized_encoding == '16UC1' or depth.dtype == np.uint16:
            return depth.astype(np.float32) * float(self._param('uint16_depth_scale'))
        if normalized_encoding == '32FC1' or depth.dtype == np.float32:
            return depth.astype(np.float32) * float(self._param('float_depth_scale'))
        if depth.dtype == np.float64:
            return depth.astype(np.float32) * float(self._param('float_depth_scale'))
        raise ValueError(f'Unsupported depth encoding: {encoding}')

    def _make_depth_image(
        self,
        depth_m: np.ndarray,
        input_encoding: str,
        header,
    ) -> Image:
        output_encoding = str(self._param('output_depth_encoding'))
        if output_encoding == 'passthrough':
            output_encoding = '16UC1' if input_encoding.upper() == '16UC1' else '32FC1'

        if output_encoding == '16UC1':
            scale = float(self._param('uint16_depth_scale'))
            if scale <= 0.0:
                raise ValueError('uint16_depth_scale must be greater than zero.')
            depth_out = np.zeros_like(depth_m, dtype=np.uint16)
            valid = depth_m > 0.0
            depth_out[valid] = np.clip(
                np.rint(depth_m[valid] / scale),
                1,
                np.iinfo(np.uint16).max,
            ).astype(np.uint16)
        elif output_encoding == '32FC1':
            depth_out = depth_m.astype(np.float32)
        else:
            raise ValueError(
                'output_depth_encoding must be passthrough, 16UC1, or 32FC1.')

        msg = self.bridge.cv2_to_imgmsg(depth_out, encoding=output_encoding)
        msg.header = header
        return msg

    def _make_point_cloud(
        self,
        aligned_depth_m: np.ndarray,
        rgb_image: np.ndarray,
        rgb_info: CameraInfo,
        header,
    ) -> PointCloud2:
        fx, fy, cx, cy = self._intrinsics(rgb_info)
        stride = max(1, int(self._param('pointcloud_stride')))
        depth = aligned_depth_m[::stride, ::stride]
        valid = np.isfinite(depth) & (depth > 0.0)
        v_small, u_small = np.nonzero(valid)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16

        if len(u_small) == 0:
            msg.width = 0
            msg.row_step = 0
            msg.data = b''
            return msg

        u = (u_small * stride).astype(np.float32)
        v = (v_small * stride).astype(np.float32)
        z = depth[v_small, u_small].astype(np.float32)
        x = ((u - cx) / fx) * z
        y = ((v - cy) / fy) * z

        colors = rgb_image[
            np.clip(v.astype(np.int32), 0, rgb_image.shape[0] - 1),
            np.clip(u.astype(np.int32), 0, rgb_image.shape[1] - 1),
        ]
        rgb_uint32 = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )

        cloud = np.empty(
            len(z),
            dtype=[
                ('x', '<f4'),
                ('y', '<f4'),
                ('z', '<f4'),
                ('rgb', '<f4'),
            ],
        )
        cloud['x'] = x
        cloud['y'] = y
        cloud['z'] = z
        cloud['rgb'] = rgb_uint32.view(np.float32)

        msg.width = len(z)
        msg.row_step = msg.point_step * msg.width
        msg.data = cloud.tobytes()
        return msg

    def _rgb_image_to_array(self, rgb_msg: Image) -> np.ndarray:
        try:
            return self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        except CvBridgeError:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
            encoding = rgb_msg.encoding.lower()
            if image.ndim == 2:
                return np.repeat(image[:, :, None], 3, axis=2).astype(np.uint8)
            if image.shape[2] >= 3:
                if encoding in ('bgr8', 'bgra8'):
                    return image[:, :, :3][:, :, ::-1].astype(np.uint8)
                return image[:, :, :3].astype(np.uint8)
            raise ValueError(f'Unsupported RGB encoding: {rgb_msg.encoding}')

    @staticmethod
    def _make_camera_info(info: CameraInfo, header) -> CameraInfo:
        msg = copy.deepcopy(info)
        msg.header = header
        return msg

    def _intrinsics(self, info: CameraInfo) -> Tuple[float, float, float, float]:
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            fx = float(info.p[0])
            fy = float(info.p[5])
            cx = float(info.p[2])
            cy = float(info.p[6])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError('CameraInfo intrinsics are invalid.')
        return fx, fy, cx, cy

    def _lookup_depth_to_rgb(
        self,
        rgb_frame: str,
        depth_frame: str,
        stamp_msg,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if (
            not bool(self._param('use_tf'))
            or not rgb_frame
            or not depth_frame
            or rgb_frame == depth_frame
        ):
            return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)

        lookup_time = Time() if bool(self._param('use_latest_tf')) else Time.from_msg(stamp_msg)
        try:
            transform = self.tf_buffer.lookup_transform(
                rgb_frame,
                depth_frame,
                lookup_time,
                timeout=Duration(seconds=float(self._param('lookup_tf_timeout_sec'))),
            )
            return self._transform_to_matrix(transform)
        except Exception as exc:
            if bool(self._param('assume_identity_tf')):
                self._warn_throttled(
                    'tf',
                    f'TF {depth_frame} -> {rgb_frame} unavailable, using identity: {exc}',
                )
                return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
            raise

    def _transform_to_matrix(self, transform) -> Tuple[np.ndarray, np.ndarray]:
        translation_msg = transform.transform.translation
        rotation_msg = transform.transform.rotation
        rotation = self._quaternion_to_matrix(
            rotation_msg.x,
            rotation_msg.y,
            rotation_msg.z,
            rotation_msg.w,
        )
        translation = np.array(
            [translation_msg.x, translation_msg.y, translation_msg.z],
            dtype=np.float32,
        )
        return rotation, translation

    @staticmethod
    def _quaternion_to_matrix(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> np.ndarray:
        norm = x * x + y * y + z * z + w * w
        if norm < 1e-12:
            return np.eye(3, dtype=np.float32)

        scale = 2.0 / norm
        xx = x * x * scale
        yy = y * y * scale
        zz = z * z * scale
        xy = x * y * scale
        xz = x * z * scale
        yz = y * z * scale
        wx = w * x * scale
        wy = w * y * scale
        wz = w * z * scale

        return np.array([
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ], dtype=np.float32)

    @staticmethod
    def _frame_id(info: CameraInfo, image: Optional[Image] = None) -> str:
        if info.header.frame_id:
            return info.header.frame_id
        if image is not None:
            return image.header.frame_id
        return ''

    def _warn_throttled(self, key: str, message: str, seconds: float = 5.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_warn_ns.get(key, 0)
        if now_ns - last_ns >= int(seconds * 1e9):
            self.get_logger().warn(message)
            self._last_warn_ns[key] = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignedRealsenseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
