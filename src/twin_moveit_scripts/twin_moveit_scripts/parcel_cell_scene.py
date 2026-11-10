"""Use every physical static SDF solid as parcel-cell planning collision geometry."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from twin_moveit_scripts.parcel_layout import PARCEL_SIZE


def _pose(values):
    p = Pose()
    p.position.x, p.position.y, p.position.z = values[:3]
    r, pitch, yaw = values[3:]
    cr, sr, cp, sp, cy, sy = (math.cos(r/2), math.sin(r/2), math.cos(pitch/2),
                             math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
    p.orientation.x = sr*cp*cy - cr*sp*sy
    p.orientation.y = cr*sp*cy + sr*cp*sy
    p.orientation.z = cr*cp*sy - sr*sp*cy
    p.orientation.w = cr*cp*cy + sr*sp*sy
    return p


def collision_objects(world_path=None):
    path = world_path or Path(get_package_share_directory('twin_action_demo')) / 'worlds/industrial_parcel_cell.sdf'
    for model in ET.parse(path).findall('world/model'):
        co = CollisionObject()
        co.header.frame_id = 'panda_link0'
        co.id = model.get('name')
        mp = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        if abs(mp[3]) > 1e-6 or abs(mp[4]) > 1e-6:
            if model.findall('.//collision'):
                raise ValueError('Non-yaw model pose requires full transform composition')
        c, s = math.cos(mp[5]), math.sin(mp[5])
        for link in model.findall('link'):
            if link.find('pose') is not None:
                raise ValueError('Nonidentity link poses need explicit composition')
            for collision in link.findall('collision'):
                xyz = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
                primitive = SolidPrimitive()
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = list(map(float, geometry.findtext('box/size').split()))
                elif geometry.find('cylinder') is not None:
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [float(geometry.findtext('cylinder/length')),
                                            float(geometry.findtext('cylinder/radius'))]
                else:
                    raise ValueError('Unsupported physical collision geometry')
                co.primitives.append(primitive)
                co.primitive_poses.append(_pose((mp[0]+c*xyz[0]-s*xyz[1],
                    mp[1]+s*xyz[0]+c*xyz[1], mp[2]+xyz[2], xyz[3], xyz[4], mp[5]+xyz[5])))
        if co.primitives:
            co.operation = CollisionObject.ADD
            yield co


def add_collision_scene(robot, logger, world_path=None):
    objects = list(collision_objects(world_path))
    with robot.get_planning_scene_monitor().read_write() as scene:
        for obj in objects:
            if scene.apply_collision_object(obj) is False:
                raise RuntimeError(f'Failed to apply required obstacle {obj.id}')
        installed = {obj.id for obj in scene.planning_scene_message.world.collision_objects}
        if not {obj.id for obj in objects} <= installed:
            raise RuntimeError('Required static collision geometry is missing after installation')
        # The fixed robot is mounted on the pedestal top at z=0. Permit only
        # that permanent base/fixture contact, never arm/fence/deck contacts.
        scene.allowed_collision_matrix.set_entry('panda_link0', 'robot_pedestal', True)
        scene.current_state.update()
    logger.info(f'parcel scene: {len(objects)} static models, all physical solids included')


def carry_parcel(robot, attached=True):
    msg = AttachedCollisionObject()
    msg.link_name = 'panda_hand'
    msg.touch_links = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']
    msg.object.header.frame_id = 'panda_link0'
    msg.object.id = 'carried_parcel'
    msg.object.operation = CollisionObject.ADD if attached else CollisionObject.REMOVE
    with robot.get_planning_scene_monitor().read_write() as scene:
        if attached:
            # The hand is horizontal and the parcel is below link8 by 0.100 m.
            transform = scene.current_state.get_global_link_transform('panda_link8')
            x, y, z = transform[:3, 3]
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(PARCEL_SIZE)
            msg.object.primitives.append(prim)
            msg.object.primitive_poses.append(_pose((float(x), float(y), float(z)-.100, 0, 0, 0)))
        if scene.process_attached_collision_object(msg) is False:
            raise RuntimeError('Could not update carried parcel collision geometry')
        if not attached:
            # MoveIt detachment reinserts the object into the world at its current
            # hand pose. Remove that transient copy; the occupied-slot solid below
            # represents the released carton after it settles on the tote floor.
            remove = CollisionObject()
            remove.id = 'carried_parcel'
            remove.header.frame_id = 'panda_link0'
            remove.operation = CollisionObject.REMOVE
            scene.apply_collision_object(remove)
        scene.current_state.update()
        attached_ids = {obj.object.id for obj in scene.planning_scene_message.robot_state.attached_collision_objects}
        if ('carried_parcel' in attached_ids) != attached:
            raise RuntimeError('Carried parcel geometry read-back did not match requested state')
        if not attached and 'carried_parcel' in {
                obj.id for obj in scene.planning_scene_message.world.collision_objects}:
            raise RuntimeError('Detached carried parcel left a ghost world obstacle')


def permit_payload_support(robot, allowed):
    """Only the payload may initially touch its belt support during liftoff."""
    with robot.get_planning_scene_monitor().read_write() as scene:
        scene.allowed_collision_matrix.set_entry('carried_parcel','pick_belt',allowed)
