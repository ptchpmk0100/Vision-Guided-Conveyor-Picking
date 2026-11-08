#!/usr/bin/env python3
"""One-shot: grab an RGB frame + depth frame, report what the cube ACTUALLY
looks like in pixels, so we key the threshold to reality instead of the SDF's
nominal material. Prints the brightest-blue region and the depth at image
centre. Run while the sim + cube are up."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class Sampler(Node):
    def __init__(self):
        super().__init__("pixel_sampler")
        self.rgb = None; self.depth = None
        self.create_subscription(Image, "/overhead_camera/image",
                                 self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/overhead_camera/depth_image",
                                 self._depth, qos_profile_sensor_data)
    def _rgb(self, m):
        try:
            self.rgb = (np.frombuffer(m.data, np.uint8)
                        .reshape(m.height, m.width, 3), m.encoding)
        except Exception as e:
            self.get_logger().error(f"rgb reshape: {e} (encoding={m.encoding}, "
                                    f"len={len(m.data)}, {m.width}x{m.height})")
    def _depth(self, m):
        try:
            self.depth = (np.frombuffer(m.data, np.float32)
                          .reshape(m.height, m.width), m.encoding)
        except Exception as e:
            self.get_logger().error(f"depth reshape: {e} (encoding={m.encoding})")

def main():
    rclpy.init()
    n = Sampler()
    import time
    t0 = time.time()
    while (n.rgb is None or n.depth is None) and time.time()-t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.rgb is None:
        print("NO RGB received in 5s"); return
    rgb, enc = n.rgb
    print(f"RGB encoding={enc} shape={rgb.shape}")
    r,g,b = rgb[:,:,0].astype(int), rgb[:,:,1].astype(int), rgb[:,:,2].astype(int)
    # find the most blue-dominant pixel (b - max(r,g)) to locate the cube
    blueness = b - np.maximum(r,g)
    idx = np.unravel_index(np.argmax(blueness), blueness.shape)
    print(f"most blue-dominant pixel at (row={idx[0]}, col={idx[1]}): "
          f"RGB=({r[idx]},{g[idx]},{b[idx]}) blueness={blueness[idx]}")
    # also report the darkest region (cube might read dark)
    bright = r+g+b
    didx = np.unravel_index(np.argmin(bright), bright.shape)
    print(f"darkest pixel at (row={didx[0]}, col={didx[1]}): "
          f"RGB=({r[didx]},{g[didx]},{b[didx]})")
    # histogram of blue channel to see the range
    print(f"blue channel: min={b.min()} max={b.max()} mean={b.mean():.0f}")
    print(f"red channel:  min={r.min()} max={r.max()} mean={r.mean():.0f}")
    if n.depth is not None:
        d, denc = n.depth
        print(f"depth encoding={denc} shape={d.shape} "
              f"centre={d[d.shape[0]//2, d.shape[1]//2]:.3f} "
              f"min={np.nanmin(d):.3f} max={np.nanmax(d[np.isfinite(d)]):.3f}")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
