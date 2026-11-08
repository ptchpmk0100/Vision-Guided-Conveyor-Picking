#!/usr/bin/env python3
"""Stage C: publish the raised sorting line's trays into the MoveIt planning scene.

The belt-height analogue of cell_scene.py. Adds the 3 trays as HOLLOW wall boxes (four thin
walls each) at belt height, so place() is genuinely collision-checked against the tray walls
the open fingers must clear.

Deliberately NOT added (same reasoning as A3's omitted ground plane): the PICK DECK and the
TRAY TABLE. During a grasp the fingertips come down to ~z=0.295 (the deck/floor surface);
modelling that surface as collision would make every descend unplannable, exactly as a floor
object did at ground level. The arm only ever approaches those surfaces vertically from
directly above, so there is no lateral-collision risk in omitting them -- the tray WALLS are
the surfaces the arm can actually foul, and those are modelled.

Call it from the sorting-line task AFTER setup() (it holds the robot handle). Not wired into
move_to.setup(), so the ground-cell paths are entirely unaffected.
"""
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

PLANNING_FRAME = "panda_link0"      # == world (identity)

# Mirrors worlds/industrial_sorting_line.sdf. (centre_x, centre_y) per tray.
TRAYS = {
    "tray_red":    (0.450, 0.180),
    "tray_green":  (0.450, -0.180),
    "tray_yellow": (0.370, 0.000),
}
# (dx, dy, abs_z, sx, sy, sz) per wall: 0.004 thick, 0.040 tall, centred at z=0.315
# (wall band z=0.295..0.335), matching the SDF.
WALLS = [
    (0.072, 0.0, 0.315, 0.004, 0.148, 0.040),
    (-0.072, 0.0, 0.315, 0.004, 0.148, 0.040),
    (0.0, 0.072, 0.315, 0.148, 0.004, 0.040),
    (0.0, -0.072, 0.315, 0.148, 0.004, 0.040),
]


def _tray_collision_object(name, cx, cy):
    co = CollisionObject()
    co.header.frame_id = PLANNING_FRAME
    co.id = name
    for dx, dy, az, sx, sy, sz in WALLS:
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = cx + dx
        pose.position.y = cy + dy
        pose.position.z = az
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


def add_sorting_line_collision(robot, logger):
    """Add the 3 trays (hollow walls) to the planning scene. Returns the number added."""
    try:
        psm = robot.get_planning_scene_monitor()
        with psm.read_write() as scene:
            for name, (cx, cy) in TRAYS.items():
                scene.apply_collision_object(_tray_collision_object(name, cx, cy))
            scene.current_state.update()
        logger.info(f"sorting_line_scene: added {len(TRAYS)} trays as collision objects "
                    f"({', '.join(sorted(TRAYS))}); deck/table omitted (fingers graze them)")
        return len(TRAYS)
    except Exception as e:
        logger.error(f"sorting_line_scene: could not add collision objects: {e!r}")
        return 0
