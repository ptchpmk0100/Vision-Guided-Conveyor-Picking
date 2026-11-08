#!/usr/bin/env python3
"""A3: publish the sorting cell's bins into the MoveIt planning scene.

Until now MoveIt knew nothing about the cell -- FR-4 "collision-checked planning" was
technically true but vacuous, because the world it checked against was empty. The arm
cleared the 3 cm bin walls only by luck of trajectory. That stops being acceptable in
A4, where the arm repeatedly descends into bins that already contain objects.

Modelled as HOLLOW bins: four thin wall boxes per bin, never one solid block -- a solid
box would make the bin interior unreachable and every place() would fail to plan.

Deliberately NOT added: the ground plane. The grasp pose puts the fingertips ~3 mm above
the floor (FINGER_REACH_Z is tuned to exactly that, see S21), so a floor collision object
would make every pick unplannable.

Opt-in by design: move_to.setup() only calls this when TWIN_CELL_COLLISION=1, so the
ground-truth-verified A1/A2 paths keep running with the exact planning behaviour they
were validated under.
"""
import os

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

PLANNING_FRAME = "panda_link0"      # == world (identity), confirmed S16 §3

# Mirrors worlds/industrial_cell.sdf exactly. (centre_x, centre_y) per bin.
BINS = {
    "bin_a": (0.45, 0.18),
    "bin_b": (0.45, -0.18),
    "bin_c": (0.37, 0.00),
}
# (dx, dy, dz, sx, sy, sz) for the four walls, relative to the bin centre.
WALLS = [
    (0.068, 0.0, 0.015, 0.004, 0.14, 0.03),
    (-0.068, 0.0, 0.015, 0.004, 0.14, 0.03),
    (0.0, 0.068, 0.015, 0.14, 0.004, 0.03),
    (0.0, -0.068, 0.015, 0.14, 0.004, 0.03),
]


def _bin_collision_object(name, cx, cy):
    co = CollisionObject()
    co.header.frame_id = PLANNING_FRAME
    co.id = name
    for dx, dy, dz, sx, sy, sz in WALLS:
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = cx + dx
        pose.position.y = cy + dy
        pose.position.z = dz
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def add_cell_collision_objects(robot, logger):
    """Add the three bins to the planning scene. Returns the number added."""
    try:
        psm = robot.get_planning_scene_monitor()
        with psm.read_write() as scene:
            for name, (cx, cy) in BINS.items():
                scene.apply_collision_object(_bin_collision_object(name, cx, cy))
            scene.current_state.update()
        logger.info(f"cell_scene: added {len(BINS)} bins as collision objects "
                    f"({', '.join(sorted(BINS))}); ground plane deliberately omitted")
        return len(BINS)
    except Exception as e:
        # Never let scene decoration break a cycle: report honestly and carry on with
        # the previously-verified (collision-unaware) behaviour.
        logger.error(f"cell_scene: could not add collision objects: {e!r}")
        return 0


def maybe_add_cell_collision_objects(robot, logger):
    """Opt-in hook for move_to.setup(). Enabled with TWIN_CELL_COLLISION=1."""
    if os.environ.get("TWIN_CELL_COLLISION", "0") != "1":
        return 0
    return add_cell_collision_objects(robot, logger)
