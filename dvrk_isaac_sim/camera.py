"""Isaac Sim camera sensors and ROS 2 image publishers for the virtual ECM."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .config import RobotConfig
from .kinematics import Pose


def _quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to Isaac Sim's scalar-first quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12))
            w, x, y, z = (rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = 2.0 * math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12))
            w, x, y, z = (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = 2.0 * math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12))
            w, x, y, z = (rotation[1, 0] - rotation[0, 1]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale
    result = np.asarray([w, x, y, z], dtype=float)
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


class IsaacCameraPublisher:
    """Create an Isaac camera and publish its rendered frames as ROS images.

    ``mono`` publishes ``/<ECM>/image_raw`` and ``/<ECM>/camera_info``.
    ``stereo`` publishes the corresponding ``left`` and ``right`` subtopics.
    The camera pose follows the ECM measured optical pose and is independent of
    any ECM mesh, so the ECM can remain a kinematic-only component.
    """

    def __init__(self, node: Any, config: RobotConfig, mode: str) -> None:
        if mode not in {"mono", "stereo"}:
            raise ValueError(f"unsupported camera mode: {mode}")
        from isaacsim.sensors.camera import Camera
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image

        camera_config = config.raw.get("robot", {}).get("camera", {})
        self.node = node
        self.mode = mode
        self.frame_id = str(camera_config.get("frame", config.tool_frame))
        self.width = int(camera_config.get("width", 1280))
        self.height = int(camera_config.get("height", 720))
        self.encoding = str(camera_config.get("encoding", "rgba8"))
        self.baseline = float(camera_config.get("baseline_m", 0.006))
        self.fov = math.radians(float(camera_config.get("horizontal_fov_deg", 60.0)))
        self.near = float(camera_config.get("near_clip_m", 0.01))
        self.far = float(camera_config.get("far_clip_m", 10.0))
        self.publish_rate = float(camera_config.get("publish_rate_hz", 30.0))
        self.jpeg_quality = int(camera_config.get("jpeg_quality", 85))
        if self.publish_rate <= 0.0:
            raise ValueError("camera.publish_rate_hz must be positive")
        self._last_capture = float("-inf")
        self._Image = Image
        self._CompressedImage = CompressedImage
        self._CameraInfo = CameraInfo
        self._cameras = []
        self._publishers = []
        names = ["mono"] if mode == "mono" else ["left", "right"]
        for name in names:
            suffix = "" if mode == "mono" else f"/{name}"
            camera = Camera(
                prim_path=f"/World/ECM/Camera{name.title() if mode == 'stereo' else ''}",
                name=f"ECM_camera_{name}", frequency=30,
                resolution=(self.width, self.height),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            camera.initialize()
            # Isaac expresses focal length and aperture in the same stage
            # units. Preserve the camera's sensor width while selecting the
            # requested horizontal field of view.
            aperture = camera.get_horizontal_aperture()
            camera.set_focal_length(aperture / (2.0 * math.tan(self.fov / 2.0)))
            camera.set_clipping_range(self.near, self.far)
            self._cameras.append(camera)
            self._publishers.append((
                node.create_publisher(Image, f"image_raw{suffix}", 10),
                node.create_publisher(CompressedImage, f"image_raw{suffix}/compressed", 10),
                node.create_publisher(CameraInfo, f"camera_info{suffix}", 10),
            ))
        node.get_logger().info(f"ECM {mode} camera publishing image_raw and camera_info")

    def _image_message(self, data: np.ndarray, stamp, camera_name: str):
        message = self._Image()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        data = np.asarray(data, dtype=np.uint8)
        if data.ndim == 2:
            data = data[:, :, None]
        if self.encoding == "rgb8" and data.shape[2] >= 3:
            data = data[:, :, :3]
        elif self.encoding == "rgba8" and data.shape[2] == 3:
            alpha = np.full(data.shape[:2] + (1,), 255, dtype=np.uint8)
            data = np.concatenate((data, alpha), axis=2)
        message.height, message.width = data.shape[:2]
        message.encoding = self.encoding
        message.is_bigendian = False
        message.step = int(data.shape[1] * data.shape[2])
        message.data = data.tobytes()
        return message

    def _compressed_message(self, data: np.ndarray, stamp):
        import cv2

        if data.ndim == 2:
            rgb = data
        else:
            rgb = data[:, :, :3]
        # cv2 expects BGR for JPEG encoding; Isaac returns RGBA/RGB.
        bgr = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not success:
            raise RuntimeError("JPEG encoding failed")
        message = self._CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def _camera_info(self, stamp):
        info = self._CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width, info.height = self.width, self.height
        fx = (self.width / 2.0) / math.tan(self.fov / 2.0)
        fy = fx
        cx, cy = (self.width - 1) / 2.0, (self.height - 1) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish(self, seconds: float, pose: Pose) -> None:
        if seconds - self._last_capture < 1.0 / self.publish_rate:
            return
        from builtin_interfaces.msg import Time
        stamp = Time(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))
        orientation = _quaternion_wxyz(pose.orientation)
        for index, (camera, (image_publisher, compressed_publisher, info_publisher)) in enumerate(zip(self._cameras, self._publishers)):
            position = pose.position.copy()
            if self.mode == "stereo":
                position += pose.orientation[:, 0] * (self.baseline / 2.0) * (-1.0 if index == 0 else 1.0)
            # ECM_optical uses local +X as the viewing direction. Isaac's
            # world-axis camera mode also defines +X as forward; using the
            # ROS mode here would incorrectly treat local +Z as forward.
            camera.set_world_pose(position=position, orientation=orientation, camera_axes="world")
            data = camera.get_rgba()
            if data is None:
                continue
            self._last_capture = seconds
            image_publisher.publish(self._image_message(data, stamp, str(index)))
            compressed_publisher.publish(self._compressed_message(data, stamp))
            info_publisher.publish(self._camera_info(stamp))
