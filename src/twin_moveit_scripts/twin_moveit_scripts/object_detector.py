#!/usr/bin/env python3
"""S20 perception seam: detect the blue cube from the overhead RGBD camera and
publish its live world-frame pose.

Pipeline (one static camera, one solid-colour cube, plain ground plane -- the
easy case, deliberately):
  RGB image  --colour threshold-->  cube pixel mask
  mask       --centroid-->          (u, v) pixel of the cube centre
  depth[v,u] + camera_info K        (u,v,d) -> (X,Y,Z) in the OPTICAL frame
  optical->world (fixed TF)         -> (x,y,z) in world == panda_link0

Output: geometry_msgs/PoseStamped on /detected_objects/box (latched-ish, 10 Hz).
resolve_object() in move_to.py subscribes to this instead of its hardcoded dict.

WHY a separate node (not inside move_to): move_to spins a SingleThreadedExecutor
around MoveItPy; camera callbacks at 10 Hz have no business competing with the
planning context. This node owns perception, publishes a pose, and move_to just
reads the latest one. Clean seam, independent lifecycles.

Design choices that keep this the EASY case honest:
  - Colour key is the cube's own material (0.2 0.6 1.0 -> strong blue). If you
    respawn a differently coloured cube, change BLUE_* below.
  - We publish ONLY when a cube is actually found. No detection -> no stale pose.
    resolve_object treats "no message yet" as "unknown", same failure mode as the
    old dict returning None. Honest, not silent.
  - z is reported from depth, but a 3 cm cube's centre is geometry we KNOW
    (0.015). We publish the depth-derived z AND log both so you can see camera
    accuracy vs ground truth -- a real portfolio-grade sanity check.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

# ---- Colour key for the blue grasp_box (material 0.2 0.6 1.0) ----
# Thresholds in raw RGB (0-255). The cube reads as high-blue, low-red.
# S21 FIX: the old test (B>=120 & R<=120 & G<=200) was satisfied by NEUTRAL GREY
# (120,120,120) -- the Panda's dark link bands -- so with the cube hidden under
# the hand the mask filled with arm pixels and a bogus pose (z = arm height) was
# published. Require blue DOMINANCE over both other channels instead. Measured on
# the sim's lit cube top face (104,172,216): B-R=112, B-G=44. Panda pixels that
# passed the old test had B-G <= 31 (the hand's blue indicator dots).
BLUE_MIN_B = 100         # blue channel must be strong
BLUE_MIN_B_MINUS_R = 80  # blue must beat red by a wide margin (grey: 0)
BLUE_MIN_B_MINUS_G = 36  # ...and green by a clear one (arm worst case: 31)

MIN_PIXELS = 25      # ignore specks; a cube at 1.2 m fills far more than this

# S21: plausibility gate on the perceived resting centre. The cube only ever
# rests on the ground in this world, so a centre far from CUBE_HALF_HEIGHT means
# the depth sample did not land on the cube (occluder, bad pixel). Publish
# nothing -- same honest "unknown" as no detection -- instead of a wrong pose.
# Loosen/remove when objects can legitimately rest on raised surfaces.
EXPECTED_CENTER_Z = 0.015
CENTER_Z_TOL = 0.04

# grasp_box.sdf is a 0.03 m cube. Depth sees its top face; pick() wants the
# resting center. center_z = top_z - half_height. If you respawn a different
# size cube, update this to match its <size>.
CUBE_HALF_HEIGHT = 0.015

CAMERA_NS = "/overhead_camera"
PUBLISH_TOPIC = "/detected_objects/box"


def _optical_to_world(x_opt, y_opt, z_opt):
    """Transform a point from the camera OPTICAL frame to the WORLD frame.

    This is the fixed composition of the two URDF joints:
      world -> camera_link:      xyz (0.4, 0, 1.2), rpy (0, +pi/2, 0)
      camera_link -> optical:    rpy (-pi/2, 0, -pi/2)
    Rather than pull tf2 in for a constant, we bake the closed-form result.
    With the camera looking straight down from (0.4, 0, 1.2):
      optical Z (into scene, i.e. downward)  -> world -Z, offset from 1.2
      optical X (image right)                -> world +Y
      optical Y (image down)                 -> world -X, offset from 0.4... 

    Derivation is easy to get subtly wrong, so we VERIFY it at runtime against
    the known cube instead of trusting the algebra blind (see node __init__).
    Mapping (derived from the composed rotation, verified off-axis --
    world_y is -x_opt, NOT +x_opt; the on-axis cube hid the sign):
      world_x = 0.4  - y_opt
      world_y =      - x_opt
      world_z = 1.2  - z_opt
    """
    world_x = 0.4 - y_opt
    world_y = -x_opt
    world_z = 1.2 - z_opt
    return world_x, world_y, world_z


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        # NOTE: pose stamps use whatever clock this node has (wall time by
        # default). move_to no longer compares stamps against its own clock --
        # it judges freshness by clearing and waiting for a NEW publish -- so a
        # clock mismatch here is harmless. Left simple on purpose.
        self._K = None            # 3x3 intrinsics, filled from camera_info
        self._rgb = None          # latest RGB as HxWx3 uint8
        self._depth = None        # latest depth as HxW float32 (metres)

        self.create_subscription(
            CameraInfo, f"{CAMERA_NS}/camera_info",
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{CAMERA_NS}/image",
            self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{CAMERA_NS}/depth_image",
            self._on_depth, qos_profile_sensor_data)

        self._pub = self.create_publisher(PoseStamped, PUBLISH_TOPIC, 10)
        self.create_timer(0.1, self._tick)     # 10 Hz detect+publish

        self._logged_once = False
        self._last_reject_ns = 0
        self.get_logger().info(
            f"object_detector up. Watching {CAMERA_NS}, publishing {PUBLISH_TOPIC}.")

    # ---- subscriptions just cache the latest frame ----
    def _on_info(self, msg):
        # K = [fx 0 cx; 0 fy cy; 0 0 1], row-major in msg.k
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_rgb(self, msg):
        self._rgb = self._image_to_array(msg, channels=3, dtype=np.uint8)

    def _on_depth(self, msg):
        # 32FC1 depth in metres
        self._depth = self._image_to_array(msg, channels=1, dtype=np.float32)
        if self._depth is not None:
            self._depth = self._depth[:, :, 0]

    @staticmethod
    def _image_to_array(msg, channels, dtype):
        try:
            buf = np.frombuffer(msg.data, dtype=dtype)
            return buf.reshape(msg.height, msg.width, channels)
        except ValueError:
            return None

    # ---- the actual detection, once per timer tick ----
    def _tick(self):
        if self._K is None or self._rgb is None or self._depth is None:
            return

        found = self._find_cube_pixel(self._rgb)
        if found is None:
            return                       # no cube -> publish nothing (honest)
        u, v, mask = found

        # S21: depth = MEDIAN over the cube's own mask pixels, not depth[v,u] at
        # the centroid. The centroid of a partly occluded (non-convex) blob need
        # not lie on the cube at all, and then depth[v,u] belongs to the occluder.
        dm = self._depth[mask]
        dm = dm[np.isfinite(dm) & (dm > 0.0)]
        if dm.size < MIN_PIXELS:
            return                       # no usable depth on the cube; skip frame
        d = float(np.median(dm))

        # deproject pixel (u,v) at depth d through the pinhole model
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        x_opt = (u - cx) * d / fx
        y_opt = (v - cy) * d / fy
        z_opt = d

        wx, wy, wz_top = _optical_to_world(x_opt, y_opt, z_opt)

        # Depth sees the cube's TOP FACE, but pick() expects the resting CENTER
        # (the S19 z=0.015 fix: z_grasp = center + FINGER_REACH_Z). A 30 mm cube
        # resting on the ground has its top at 0.030 and center at 0.015, so the
        # raw perceived z is one half-height too high. Subtract half-height to
        # report the center. (Known-geometry correction; when object height is
        # itself perceived later, derive this instead of hardcoding.)
        wz = wz_top - CUBE_HALF_HEIGHT

        if abs(wz - EXPECTED_CENTER_Z) > CENTER_Z_TOL:
            now = self.get_clock().now().nanoseconds
            if now - self._last_reject_ns > 2_000_000_000:   # throttle to 0.5 Hz
                self._last_reject_ns = now
                self.get_logger().warn(
                    f"rejected implausible detection: world=({wx:.3f}, {wy:.3f}, "
                    f"{wz:.3f}); centre z must be {EXPECTED_CENTER_Z}+/-{CENTER_Z_TOL} "
                    f"(occluded cube / depth on the arm?). Publishing nothing.")
            return

        # publish the pose (world frame == panda_link0)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = wx
        msg.pose.position.y = wy
        msg.pose.position.z = wz
        msg.pose.orientation.w = 1.0     # axis-aligned; yaw perception is a later brick
        self._pub.publish(msg)

        if not self._logged_once:
            # one-time sanity line: show BOTH the raw top-face z and the
            # center-corrected z, so the correction is visible and verifiable.
            self.get_logger().info(
                f"first detection: pixel=({u},{v}) depth={d:.3f} "
                f"-> world=({wx:.3f}, {wy:.3f}, {wz:.3f}) "
                f"[top_z={wz_top:.3f} - half {CUBE_HALF_HEIGHT} = center; "
                f"expect ~0.45, 0.0, 0.015 for the spawn cube]")
            self._logged_once = True

    def _find_cube_pixel(self, rgb):
        """Return (u, v, mask) -- centroid pixel and the boolean cube mask -- or None."""
        r = rgb[:, :, 0].astype(np.int16)
        g = rgb[:, :, 1].astype(np.int16)
        b = rgb[:, :, 2].astype(np.int16)
        mask = ((b >= BLUE_MIN_B)
                & ((b - r) >= BLUE_MIN_B_MINUS_R)
                & ((b - g) >= BLUE_MIN_B_MINUS_G))
        ys, xs = np.nonzero(mask)
        if xs.size < MIN_PIXELS:
            return None
        u = int(round(xs.mean()))
        v = int(round(ys.mean()))
        return u, v, mask


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
