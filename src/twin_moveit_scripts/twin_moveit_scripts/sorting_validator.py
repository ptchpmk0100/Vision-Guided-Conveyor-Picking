#!/usr/bin/env python3
"""Independent FR-2/FR-3/FR-8 monitor for sorting-line detections.

The invariant functions have no ROS dependency and can be imported by tests or other
monitors.  ROS imports occur only in live mode, so the offline check is simply:

  python3 src/twin_moveit_scripts/twin_moveit_scripts/sorting_validator.py --self-test

Live mode observes detections and publishes JSON reports; it never commands the robot or
accesses a mutating Gazebo interface:

  ros2 run twin_moveit_scripts sorting_validator
"""
from collections import Counter
import json
import math
import sys
import time


DETECTION_TOPIC = "/detected_objects/array"
REPORT_TOPIC = "/validation/report"

# FR-2: verified raised-cell perception/reach envelope.
REACH_ENVELOPE = {
    "x_min": 0.30,
    "x_max": 0.62,
    "abs_y_max": 0.27,
}
BELT_Z = 0.320
BELT_Z_TOL = 0.010

# FR-3: the raised sorting line contains exactly these uniform payload classes.
KNOWN_CLASSES = frozenset({"red_box", "green_box", "yellow_box"})
BOX_WIDTH = 0.040

# FR-8: colour-labelled tray footprints, matching the verified sorting-line geometry.
TRAY_CENTRES = {
    "red": (0.450, 0.180),
    "green": (0.450, -0.180),
    "yellow": (0.370, 0.000),
}
TRAY_RADIUS = 0.075
SUMMARY_PERIOD_S = 5.0


def _json_number(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _position(x, y, z):
    return {
        "x": _json_number(x),
        "y": _json_number(y),
        "z": _json_number(z),
    }


def check_detection(cls, x, y, z):
    """Return FR-2/FR-3/FR-8 violations for one world-frame detection."""
    cls = str(cls)
    x, y, z = float(x), float(y), float(z)
    violations = []

    if cls not in KNOWN_CLASSES:
        violations.append({
            "code": "UNKNOWN_CLASS",
            "class_id": cls,
            "position": _position(x, y, z),
            "detail": f"expected one of {sorted(KNOWN_CLASSES)}",
        })

    xy_finite = math.isfinite(x) and math.isfinite(y)
    if not xy_finite or not (
        REACH_ENVELOPE["x_min"] <= x <= REACH_ENVELOPE["x_max"]
        and abs(y) <= REACH_ENVELOPE["abs_y_max"]
    ):
        violations.append({
            "code": "OUT_OF_ENVELOPE",
            "class_id": cls,
            "position": _position(x, y, z),
            "detail": (
                f"require x in [{REACH_ENVELOPE['x_min']:.2f},"
                f"{REACH_ENVELOPE['x_max']:.2f}] and "
                f"|y| <= {REACH_ENVELOPE['abs_y_max']:.2f}"
            ),
        })

    if not math.isfinite(z) or abs(z - BELT_Z) > BELT_Z_TOL:
        violations.append({
            "code": "BAD_Z",
            "class_id": cls,
            "position": _position(x, y, z),
            "detail": f"require z={BELT_Z:.3f} +/- {BELT_Z_TOL:.3f}",
        })

    if xy_finite:
        for tray_colour, (tray_x, tray_y) in TRAY_CENTRES.items():
            if math.hypot(x - tray_x, y - tray_y) <= TRAY_RADIUS:
                actual_colour = cls.rsplit("_box", 1)[0] if cls.endswith("_box") else None
                if actual_colour != tray_colour:
                    violations.append({
                        "code": "WRONG_TRAY",
                        "class_id": cls,
                        "position": _position(x, y, z),
                        "tray_colour": tray_colour,
                        "detail": (
                            f"{cls!r} is inside {tray_colour!r} tray footprint"
                        ),
                    })
                break

    return violations


def check_overlaps(dets):
    """Return one OVERLAP violation for each pair closer than one box width in xy.

    ``dets`` is an iterable of mappings with ``cls``, ``x``, ``y``, and optionally
    ``z``.  Pair indices make each report traceable to its source Detection3DArray.
    """
    detections = list(dets)
    violations = []
    for first in range(len(detections)):
        for second in range(first + 1, len(detections)):
            a, b = detections[first], detections[second]
            ax, ay = float(a["x"]), float(a["y"])
            bx, by = float(b["x"]), float(b["y"])
            if not all(math.isfinite(value) for value in (ax, ay, bx, by)):
                continue
            distance = math.hypot(ax - bx, ay - by)
            if distance < BOX_WIDTH:
                violations.append({
                    "code": "OVERLAP",
                    "detection_indices": [first, second],
                    "class_ids": [str(a.get("cls", "")), str(b.get("cls", ""))],
                    "distance_xy_m": distance,
                    "detail": (
                        f"centres are {distance:.6f} m apart; require >= "
                        f"one box width ({BOX_WIDTH:.3f} m)"
                    ),
                })
    return violations


def validate_detections(dets):
    """Validate a complete detection set and annotate single-object violations."""
    detections = list(dets)
    violations = []
    for index, detection in enumerate(detections):
        found = check_detection(
            detection.get("cls", ""), detection["x"], detection["y"], detection["z"],
        )
        for violation in found:
            violation["detection_index"] = index
        violations.extend(found)
    violations.extend(check_overlaps(detections))
    return violations


def _self_test():
    valid = [
        {"cls": "red_box", "x": 0.560, "y": 0.110, "z": 0.320},
        {"cls": "green_box", "x": 0.560, "y": 0.000, "z": 0.320},
        {"cls": "yellow_box", "x": 0.560, "y": -0.110, "z": 0.320},
    ]
    assert validate_detections(valid) == []
    print("VALIDATOR SELF-TEST: valid 3-box staging set -> OK")

    out_codes = {
        item["code"] for item in check_detection("red_box", 0.80, 0.0, BELT_Z)
    }
    assert "OUT_OF_ENVELOPE" in out_codes
    print("VALIDATOR SELF-TEST: x=0.80 -> OUT_OF_ENVELOPE")

    wrong_codes = {
        item["code"]
        for item in check_detection("red_box", 0.450, -0.180, BELT_Z)
    }
    assert "WRONG_TRAY" in wrong_codes
    print("VALIDATOR SELF-TEST: red in green tray -> WRONG_TRAY")

    unknown_codes = {
        item["code"]
        for item in check_detection("purple_box", 0.560, 0.110, BELT_Z)
    }
    assert "UNKNOWN_CLASS" in unknown_codes
    print("VALIDATOR SELF-TEST: purple_box -> UNKNOWN_CLASS")

    coincident = [
        {"cls": "red_box", "x": 0.560, "y": 0.000, "z": BELT_Z},
        {"cls": "green_box", "x": 0.560, "y": 0.000, "z": BELT_Z},
    ]
    overlap_codes = {item["code"] for item in check_overlaps(coincident)}
    assert "OVERLAP" in overlap_codes
    print("VALIDATOR SELF-TEST: coincident boxes -> OVERLAP")
    print("VALIDATOR SELF-TEST PASS")


def _run_ros():
    # Lazy imports keep pure validation and --self-test usable without a ROS runtime.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from vision_msgs.msg import Detection3DArray

    class SortingValidator(Node):
        def __init__(self):
            super().__init__("sorting_validator")
            self._publisher = self.create_publisher(String, REPORT_TOPIC, 10)
            self.create_subscription(
                Detection3DArray, DETECTION_TOPIC, self._on_detections, 10,
            )
            self._last_summary = float("-inf")
            self.get_logger().info(
                f"sorting_validator observing {DETECTION_TOPIC}; "
                f"publishing {REPORT_TOPIC}"
            )

        def _on_detections(self, message):
            detections = []
            for detection in message.detections:
                cls = ""
                if detection.results:
                    cls = detection.results[0].hypothesis.class_id
                position = detection.bbox.center.position
                detections.append({
                    "cls": cls,
                    "x": float(position.x),
                    "y": float(position.y),
                    "z": float(position.z),
                })

            violations = validate_detections(detections)
            by_code = Counter(item["code"] for item in violations)
            verdict = "OK" if not violations else "VIOLATIONS"
            report = {
                "verdict": verdict,
                "counts": {
                    "detections": len(detections),
                    "violations": len(violations),
                    "by_code": dict(sorted(by_code.items())),
                },
                "violations": violations,
                "source": {
                    "topic": DETECTION_TOPIC,
                    "stamp": {
                        "sec": int(message.header.stamp.sec),
                        "nanosec": int(message.header.stamp.nanosec),
                    },
                    "frame_id": message.header.frame_id,
                },
            }
            output = String()
            output.data = json.dumps(report, sort_keys=True, separators=(",", ":"))
            self._publisher.publish(output)

            now = time.monotonic()
            if now - self._last_summary >= SUMMARY_PERIOD_S:
                summary = (
                    f"validation {verdict}: detections={len(detections)} "
                    f"violations={len(violations)} by_code={dict(by_code)}"
                )
                if violations:
                    self.get_logger().warn(summary)
                else:
                    self.get_logger().info(summary)
                self._last_summary = now

    rclpy.init()
    node = SortingValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    if "--self-test" in sys.argv[1:]:
        _self_test()
        return
    _run_ros()


if __name__ == "__main__":
    main()
