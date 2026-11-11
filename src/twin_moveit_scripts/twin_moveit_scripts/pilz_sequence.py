"""Planning-only Pilz sequence service adapter. Never dispatches to a controller."""
import math
import time

from moveit_msgs.srv import ApplyPlanningScene, GetMotionSequence
from rclpy.serialization import serialize_message
import hashlib


def sequence_errors(items, endpoint_xyz, mandatory_stops):
    errors = []
    if not items or len(items) != len(endpoint_xyz) or len(items) != len(mandatory_stops):
        return ['SEQUENCE_SHAPE']
    groups = {item.req.group_name for item in items}
    if len(groups) != 1 or not next(iter(groups)):
        errors.append('GROUP_MISMATCH')
    for i, item in enumerate(items):
        radius = item.blend_radius
        if not math.isfinite(radius) or radius < 0:
            errors.append(f'INVALID_RADIUS:{i}')
        if mandatory_stops[i] and radius != 0:
            errors.append(f'CONTACT_MUST_STOP:{i}')
        state = item.req.start_state
        has_start = bool(state.joint_state.name or state.multi_dof_joint_state.joint_names
                         or state.attached_collision_objects or state.is_diff)
        if (i == 0 and not has_start) or (i > 0 and has_start):
            errors.append(f'START_STATE_OWNERSHIP:{i}')
        if len(endpoint_xyz[i]) != 3 or not all(math.isfinite(v) for v in endpoint_xyz[i]):
            errors.append(f'INVALID_ENDPOINT:{i}')
    if items[-1].blend_radius != 0:
        errors.append('FINAL_RADIUS_NONZERO')
    if not errors:
        for i, (a, b) in enumerate(zip(items, items[1:])):
            total = a.blend_radius+b.blend_radius
            if total > 0 and total >= math.dist(endpoint_xyz[i], endpoint_xyz[i+1])-1e-9:
                errors.append(f'BLEND_SPHERES_OVERLAP:{i}')
    return errors


class PilzPlanningClient:
    """Caller owns executor and scene. Discover service names by type in a namespace."""
    def __init__(self, node, executor, namespace):
        self.node, self.executor = node, executor
        self.namespace = namespace.rstrip('/')+'/'
        self.revision = None

    def service(self, service_type, type_name, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            names = [name for name, types in self.node.get_service_names_and_types()
                     if name.startswith(self.namespace) and type_name in types]
            if len(names) > 1:
                raise RuntimeError('AMBIGUOUS_SEQUENCE_CAPABILITY')
            if names:
                return self.node.create_client(service_type,names[0])
            self.executor.spin_once(timeout_sec=.05)
        raise RuntimeError(f'CAPABILITY_UNAVAILABLE:{type_name}')

    def call(self, client, request, timeout=20):
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.05)
        if not future.done():
            raise RuntimeError('SEQUENCE_SERVICE_TIMEOUT')
        return future.result()

    def synchronize(self, scene_message):
        client = self.service(ApplyPlanningScene,'moveit_msgs/srv/ApplyPlanningScene')
        request = ApplyPlanningScene.Request(scene=scene_message)
        if not self.call(client,request).success:
            raise RuntimeError('SCENE_SYNC_REJECTED')
        self.revision = hashlib.sha256(serialize_message(scene_message)).hexdigest()
        return self.revision

    def plan(self, items, endpoint_xyz, mandatory_stops, scene_revision):
        errors = sequence_errors(items,endpoint_xyz,mandatory_stops)
        if errors:
            raise ValueError(','.join(errors))
        if not scene_revision or scene_revision != self.revision:
            raise RuntimeError('SCENE_REVISION_MISMATCH')
        client = self.service(GetMotionSequence,'moveit_msgs/srv/GetMotionSequence')
        request = GetMotionSequence.Request()
        request.request.items = items
        response = self.call(client,request).response
        if response.error_code.val != 1 or not response.planned_trajectories:
            raise RuntimeError(f'SEQUENCE_PLAN_FAILED:{response.error_code.val}')
        return response
