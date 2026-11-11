"""Stationary or indexed parcel-to-tote handling with fail-stopped feed control."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import tempfile
import hashlib

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3DArray

from twin_moveit_scripts.move_to import setup, shutdown, move_to_named, pick, place
from twin_moveit_scripts.parcel_cell_scene import add_collision_scene, carry_parcel
from twin_moveit_scripts.parcel_layout import (CONTINUOUS_SPEED, DETECTION_TOPIC, PARCEL_Z,
                                              SLOTS, in_pick_region)
from twin_moveit_scripts.parcel_layout import batch_layout

# Gripper opening time when releasing into the tote in the 7/8/10 profiles. The
# fingers travel ~15 mm (limit 0.2 m/s), so 0.8 s is gentle; the original 2.5 s
# trajectory dominated every release. The three-box path keeps its 2.5 s stroke.
RELEASE_STROKE_S = 0.5


class Observations:
    def __init__(self, node, detection_topic=DETECTION_TOPIC, tote_topic='/parcel_cell/tote_detections',
                 region=in_pick_region):
        self.node = node
        self.region = region
        self.message = None
        self.received = 0.0
        self.tote_message = None
        self.pub = node.create_publisher(Float64, '/parcel_cell/feed_command', 10)
        node.create_subscription(Detection3DArray, detection_topic, self.callback, 10)
        node.create_subscription(Detection3DArray, tote_topic, self.tote_callback, 10)

    def tote_callback(self, message):
        self.tote_message = message

    def callback(self, message):
        self.message = message
        self.received = time.monotonic()

    def poll(self, seconds=.15, speed=0.0):
        deadline = time.monotonic() + seconds
        # Motion helpers temporarily use their own executor on this node. Do not
        # reuse the global executor's partially completed callback iterator after
        # those transfers; keep this poll's subscriptions/clock in one executor.
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            while time.monotonic() < deadline:
                self.pub.publish(Float64(data=speed))
                executor.spin_once(timeout_sec=.05)
        finally:
            executor.remove_node(self.node)
            executor.shutdown(timeout_sec=0)

    def source_stamp(self):
        if self.message is None:
            return None
        return self.message.header.stamp.sec + self.message.header.stamp.nanosec/1e9

    def health(self):
        message = self.message
        stamp_ns = None if message is None else (message.header.stamp.sec*1000000000
                                                 + message.header.stamp.nanosec)
        return {'received_age_wall': time.monotonic()-self.received,
                'source_age_sim': None if stamp_ns is None else
                    (self.node.get_clock().now().nanoseconds-stamp_ns)/1e9}

    def current(self):
        # The wall bound only guards against a frozen stream; freshness for targeting is
        # the sim-time `age` gate below. Sized for GUI real-time factor (~0.3) so detector
        # jitter does not drop otherwise-fresh detections and starve the belt tracker.
        if self.message is None or time.monotonic() - self.received > 1.5:
            return None
        age = self.health()['source_age_sim']
        if self.message.header.frame_id != 'world' or not -.05 <= age <= .7:
            return None
        positions = []
        for detection in self.message.detections:
            if not detection.results or detection.results[0].hypothesis.class_id != 'parcel_box':
                raise RuntimeError('Unrecognized parcel detection')
            p = detection.bbox.center.position
            xyz = (p.x, p.y, p.z)
            if not all(math.isfinite(v) for v in xyz) or not self.region(p.x, p.y) or abs(p.z-PARCEL_Z) > .010:
                raise RuntimeError('Rejected invalid/out-of-region parcel pose')
            positions.append(xyz)
        return sorted(positions, key=lambda p: -p[1])

    def stationary_target(self):
        deadline = time.monotonic() + 8
        previous = None
        stable_since = None
        while time.monotonic() < deadline:
            self.poll()
            positions = self.current()
            if positions:
                target = positions[0]
                now = self.source_stamp()
                if previous is not None and math.dist(previous, target) < .002:
                    if stable_since is not None and now - stable_since >= .6:
                        return target
                else:
                    stable_since = now
                previous = target
        raise RuntimeError('No fresh stable parcel pose after belt stop')

    def feed_one(self, timeout=25.0):
        # Transport depends on simulated physics time; retain a wall-time ceiling
        # and independent perception watchdog if simulation stops progressing.
        deadline = self.node.get_clock().now().nanoseconds/1e9 + timeout
        wall_deadline = time.monotonic() + max(10.0, timeout*4)
        saw_valid = time.monotonic()
        report_at = saw_valid
        try:
            while (self.node.get_clock().now().nanoseconds/1e9 < deadline
                   and time.monotonic() < wall_deadline):
                self.poll(.10, .035)
                positions = self.current()
                if positions is not None:
                    saw_valid = time.monotonic()
                    if len(positions) > 1:
                        raise RuntimeError(
                            'Multiple parcels in the indexed pick region. Use indexed:=true '
                            'in Terminal 1 with --indexed in Terminal 2; omit both for stationary parcels.')
                    if positions and positions[0][1] >= -.025:
                        return
                if time.monotonic() >= report_at:
                    self.node.get_logger().info('FEED_WAIT ' + json.dumps({
                        'sim_time': self.node.get_clock().now().nanoseconds/1e9,
                        'detected': None if positions is None else len(positions),
                        'command_speed_m_s': .035,
                        'feed_guard_subscribers': self.pub.get_subscription_count(),
                        **self.health(),
                    }))
                    report_at = time.monotonic() + 5.0
                if time.monotonic() - saw_valid > 1.0:
                    raise RuntimeError('Perception watchdog stopped the infeed: ' + json.dumps(self.health()))
            raise RuntimeError('Belt arrival timeout')
        finally:
            self.poll(.6, 0.0)

    def _slot_sample(self, x, y):
        """(valid, stamp): does a fresh tote observation show a box at slot (x, y)?"""
        msg = self.tote_message
        now = self.node.get_clock().now().nanoseconds / 1e9
        if msg is None:
            return False, None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        valid = False
        if -.05 <= now-stamp < .7:
            valid = any(math.dist((d.bbox.center.position.x, d.bbox.center.position.y,
                                   d.bbox.center.position.z), (x, y, PARCEL_Z)) < .015
                        for d in msg.detections)
        return valid, stamp

    def confirm_slot(self, x, y):
        deadline = time.monotonic() + 8
        since = None
        while time.monotonic() < deadline:
            self.poll()
            valid, stamp = self._slot_sample(x, y)
            if valid:
                if since is None:
                    since = stamp
                elif stamp-since >= .6:
                    return
            else:
                since = None
        raise RuntimeError('Released parcel did not settle in its tote slot in depth observations')

    def require_empty_tote(self):
        self.tote_message = None
        self.poll(1.0)
        msg = self.tote_message
        if msg is None:
            raise RuntimeError('No tote observation; feed inhibited')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        age = self.node.get_clock().now().nanoseconds/1e9 - stamp
        if not -.05 <= age <= .7 or msg.detections:
            raise RuntimeError('Tote is occupied or observation stale; empty/reset it before a new batch')


def _run(args):
    batch=batch_layout(args.parcel_count,args.parcel_pitch)
    started = time.monotonic()
    result = {'indexed': args.indexed, 'continuous': args.continuous,
              'parcel_count':batch.count,'parcel_pitch':batch.pitch,'slots':batch.slots,
              'started_wall_monotonic': started,
              'grasp_mode': args.grasp_mode,
              'belt_speed': args.belt_speed if args.continuous else None, 'passed': False, 'events': []}
    robot = twin = None
    observed = None
    logger = None
    world_temp=None
    picked_objects=set()
    try:
        robot, arm, logger, params, twin = setup(node_name='parcel_task',
                                               prime_gripper=not args.continuous)
        result['context_ready_wall'] = time.monotonic()-started
        twin.set_parameters([Parameter('use_sim_time', value=True)])
        if batch.extended:
            from ament_index_python.packages import get_package_share_directory
            from twin_moveit_scripts.parcel_batch_world import write_batch_world
            from rclpy.parameter_client import AsyncParameterClient
            handle=tempfile.NamedTemporaryFile(prefix='parcel-ten-plan-',suffix='.sdf',delete=False)
            handle.close();world_temp=Path(handle.name)
            write_batch_world(Path(get_package_share_directory('twin_action_demo')) /
                              'worlds/industrial_parcel_cell_continuous.sdf',world_temp,batch,
                              reject_bin=args.condition_routing)
            client=AsyncParameterClient(twin,'object_detector_parcel_cell')
            if not client.wait_for_services(timeout_sec=10):raise RuntimeError('Batch detector unavailable')
            future=client.get_parameters(['parcel_count','parcel_pitch','batch_scene_hash'])
            executor=SingleThreadedExecutor();executor.add_node(twin)
            try:
                deadline=time.monotonic()+10
                while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
                if not future.done():raise RuntimeError('Batch profile handshake timed out')
                values=future.result().values
                if (len(values)!=3 or values[0].integer_value!=batch.count or
                    abs(values[1].double_value-batch.pitch)>1e-9 or
                    values[2].string_value!=hashlib.sha256(world_temp.read_bytes()).hexdigest()):
                    raise RuntimeError('Launch/task batch profiles differ; no motion authorized')
                result['batch_scene_hash']=values[2].string_value
            finally:
                executor.remove_node(twin);executor.shutdown(timeout_sec=0)
        if args.continuous:
            from ament_index_python.packages import get_package_share_directory
            from twin_moveit_scripts import parcel_continuous as continuous
            observed = continuous.background_observations(Observations, args.belt_speed,
                sensors=args.grasp_sensors or args.grasp_mode=='confirmed')
            add_collision_scene(robot, logger, world_temp or (Path(get_package_share_directory('twin_action_demo')) /
                                'worlds/industrial_parcel_cell_continuous.sdf'))
        else:
            observed = Observations(twin)
            add_collision_scene(robot, logger)
        def event(state, **extra):
            if batch.extended and state=='GRASP_CONFIRMED':
                identity=extra.get('object')
                if identity not in batch.names or identity in picked_objects:
                    raise RuntimeError('Duplicate or foreign confirmed parcel: '+str(identity))
                picked_objects.add(identity)
            now = observed.now() if args.continuous else twin.get_clock().now().nanoseconds/1e9
            row = dict(state=state, sim_time=now, wall_elapsed=time.monotonic()-started, **extra)
            result['events'].append(row)
            logger.info('PARCEL_STATE ' + json.dumps(row))
        if args.continuous:
            continuous.prepare(robot, arm, logger, params, observed, event, startup=True)
            if args.handoff_probe:
                from twin_moveit_scripts.parcel_confirmed import handoff_probe
                result['handoff_probe']=handoff_probe(robot,arm,observed,event)
                continuous.prepare(robot,arm,logger,params,observed,event)
            observed.require_empty_tote()
            observed.start()
            if batch.extended:
                observed.enable_tracking(args.belt_speed)
            event('BELT_RUNNING', speed=args.belt_speed)
        else:
            observed.poll(.7)
            if not move_to_named(robot, arm, logger, params, 'ready').ok:
                raise RuntimeError('Cannot reach the clear observation pose')
            observed.require_empty_tote()
        routing = args.condition_routing
        router = skipped = None
        if routing:
            from twin_moveit_scripts import parcel_reject
            router, skipped = parcel_reject.Router(batch.slots), set()
        turn_numbers = range(batch.count) if routing else range(len(batch.slots))
        for slot_no in turn_numbers:
            turn = slot_no
            reject = False
            x, y = (None, None) if routing else batch.slots[slot_no]
            event('READY', slot=slot_no)
            # Phase 3: the arm is already at 'ready' here -- homed before the loop
            # (slot 0) or by the post-place verification home of the previous slot
            # -- so the old per-slot re-home was a redundant stop. If a direct pick
            # approach ever fails, pick(home_first=False) still homes and retries.
            if args.continuous and batch.extended:
                # Launch as soon as a whole, tracked box is reachable (persistent tracker).
                from twin_moveit_scripts.parcel_confirmed import intercept as confirmed_intercept
                announced = False
                skip_turn = False
                while True:
                    track, velocity = continuous.next_target(
                        observed, args.belt_speed, window=(continuous.EARLY_LAUNCH_Y, -.075),
                        exclude=skipped or frozenset())
                    verdict_info = {}
                    if routing:
                        state, p_damaged, votes = observed.condition_verdict(track.number)
                        action = parcel_reject.next_action(state, observed.track_y(track.number))
                        verdict_info = dict(condition=state, p_damaged=p_damaged, votes=votes)
                        if action == 'wait':
                            observed.poll(.05)
                            continue
                        if action == 'skip':
                            # class unknown or no votes in time: leave it on the belt, never guess
                            skipped.add(track.number)
                            event('SKIPPED', track=track.number, **verdict_info)
                            skip_turn = True
                            break
                    if not announced:
                        event('PICK', track=track.number, **verdict_info)
                        announced = True
                    try:
                        confirmed_intercept(robot, arm, logger, observed, track, velocity, event,
                                            timing=continuous.FAST_TIMING)
                        break
                    except continuous.InterceptNotFeasible as exc:
                        # Nothing was dispatched: wait for the box to come closer, retry.
                        event('INTERCEPT_DEFERRED', track=track.number, reason=str(exc))
                        observed.poll(.15)
                if skip_turn:
                    continue
                if routing:
                    destination, dest_no, (x, y) = router.assign(state)
                    reject = destination == 'reject'
                    if not reject:
                        slot_no = dest_no
            elif args.continuous:
                track, velocity = continuous.next_target(observed, args.belt_speed)
                event('PICK', track=track.number)
                if args.grasp_mode == 'confirmed':
                    from twin_moveit_scripts.parcel_confirmed import intercept as confirmed_intercept
                    confirmed_intercept(robot, arm, logger, observed, track, velocity, event)
                else:
                    continuous.intercept(robot, arm, logger, observed, track, velocity, event)
            elif args.indexed:
                event('FEED')
                observed.feed_one()
                event('STOPPED')
            if not args.continuous:
                target = observed.stationary_target()
                event('PICK', target=target)
            # Retry only an explicitly empty failed grasp. Never home blindly when held.
            for attempt in range(0 if args.continuous else 2):
                # home_first=False: the task already homed to 'ready' before this
                # pick, so skip the redundant re-home (Phase 2). Falls back to
                # home-then-approach internally if the direct plan fails.
                picked = pick(robot, arm, logger, params, twin, target,
                              expect_half_width=.025, width_tol=.006,
                              home_first=False, blend=True)
                if picked.ok:
                    break
                if picked.held or attempt == 1:
                    raise RuntimeError(f'Pick stopped: {picked}')
                ready = move_to_named(robot, arm, logger, params, 'ready')
                if not ready.ok:
                    raise RuntimeError('Failed to recover to ready after empty grasp')
                target = observed.stationary_target()
            if not getattr(observed,'payload_attached',False):
                carry_parcel(robot)
            event('PLACE', target=[x, y, PARCEL_Z], **({'destination': destination, 'slot': dest_no} if routing else {}))
            # home_first=False: route directly from the lift pose to the tote
            # (object->tray) instead of detouring home (Phase 2). Falls back to
            # home-then-approach internally if the direct plan fails.
            from twin_moveit_scripts.move_to import guarded_motion
            confirmed=args.continuous and args.grasp_mode=='confirmed'
            def authorize_release():
                observed.release_grasp()
                event('RELEASE_AUTHORIZED')
            if batch.extended:
                observed.settle_pending_slot()   # never stack onto an unconfirmed slot
            with guarded_motion(observed.check if confirmed else None):
                placed = place(robot, arm, logger, params, twin, (x, y, PARCEL_Z),
                               clearance=.09, home_first=False, blend=True,
                               before_release=authorize_release if confirmed else None,
                               release_seconds=RELEASE_STROKE_S if batch.extended else None,
                               retract=not batch.extended)
            if not placed.ok:
                raise RuntimeError(f'Placement stopped with feed inhibited: {placed}')
            carry_parcel(robot, False)
            if confirmed:observed.payload_attached=False
            # Put occupied slots in the planning scene for later transfers.
            from geometry_msgs.msg import Pose
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive
            from twin_moveit_scripts.parcel_layout import PARCEL_SIZE
            collision = CollisionObject()
            collision.header.frame_id = 'panda_link0'
            collision.id = f'reject_slot_{dest_no}' if reject else f'filled_slot_{slot_no}'
            collision.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(PARCEL_SIZE)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, PARCEL_Z
            pose.orientation.w = 1.0
            collision.primitives.append(prim)
            collision.primitive_poses.append(pose)
            with robot.get_planning_scene_monitor().read_write() as scene:
                if scene.apply_collision_object(collision) is False:
                    raise RuntimeError('Cannot protect occupied tote slot')
                if collision.id not in {obj.id for obj in scene.planning_scene_message.world.collision_objects}:
                    raise RuntimeError('Occupied slot collision read-back failed')
            if batch.extended and reject:
                event('REJECTED', slot=dest_no)     # the bin is outside the tote depth region; the checker confirms it
            elif batch.extended:
                # Confirm the slot from depth while the arm already heads back.
                observed.begin_slot_confirmation(x, y, lambda s=slot_no: event('SETTLED', slot=s))
            direct_return=args.continuous and batch.extended and turn<batch.count-1
            if direct_return:
                continuous.prepare(robot,arm,logger,params,observed,event)
            elif not move_to_named(robot, arm, logger, params, 'ready').ok:
                raise RuntimeError('Cannot clear the tote for release verification')
            if batch.extended:
                if not direct_return:
                    observed.settle_pending_slot()
            else:
                observed.confirm_slot(x, y)
                event('SETTLED', slot=slot_no)
            if args.continuous and slot_no < len(batch.slots)-1 and not direct_return:
                continuous.prepare(robot, arm, logger, params, observed, event)
        if routing:
            observed.settle_pending_slot()
            event('BATCH_ROUTED', intact_in_tote=router.tote_used, rejected=router.bin_used,
                  skipped=len(skipped), skipped_tracks=sorted(skipped))
            result['routing'] = dict(intact_in_tote=router.tote_used, rejected=router.bin_used, skipped=len(skipped))
        if not move_to_named(robot, arm, logger, params, 'ready').ok:
            raise RuntimeError('Final ready failed')
        event('FULL', occupied=router.tote_used if routing else len(batch.slots))
        result['passed'] = True
        print('PARCEL_TASK_COMPLETE', flush=True)
        if routing and skipped:
            print(f'PARCEL_TASK_SKIPPED {len(skipped)} parcel(s) left on the belt: class unknown, never guessed',
                  flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
        if logger is not None:
            logger.error('PARCEL_TASK_FAILED: ' + result['error'])
        print('PARCEL_TASK_FAILED: ' + result['error'], flush=True)
    finally:
        if observed and rclpy.ok():
            if args.continuous:
                observed.close()
            else:
                observed.poll(.5, 0.0)
        if args.output:
            args.output.write_text(json.dumps(result, indent=2) + '\n')
        if world_temp is not None:world_temp.unlink(missing_ok=True)
        sys.stdout.flush()
        sys.stderr.flush()
        # moveit_py's MoveItCpp C++ destructor segfaults at interpreter shutdown under
        # `ros2 run` (exit 245), and that crash interrupts multiprocessing cleanup, which
        # is what prints the resource_tracker semaphore-leak warning. Results and the
        # isolated I/O child are already flushed/stopped above, so finalise with a
        # deterministic exit code instead of letting the crash decide it. This does NOT
        # mask failures: the code comes from result['passed'], already written to disk.
        os._exit(0 if result['passed'] else 1)
    return 0 if result['passed'] else 1


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--indexed', action='store_true')
    mode.add_argument('--continuous', action='store_true')
    parser.add_argument('--belt-speed', type=float, default=CONTINUOUS_SPEED)
    parser.add_argument('--grasp-mode', choices=('scheduled','confirmed'), default='scheduled')
    parser.add_argument('--grasp-sensors', action='store_true')
    parser.add_argument('--handoff-probe', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--parcel-count',type=int,choices=(3,7,8,10),default=3)
    parser.add_argument('--condition-routing', action='store_true',
                        help='damaged parcels -> reject bin, intact -> tote, unknown -> skipped (needs the condition '
                             'node and launch reject_bin:=true); default OFF = depth detector only')
    parser.add_argument('--parcel-pitch',type=float)
    args = parser.parse_args()
    if args.parcel_count!=3 and not args.continuous:parser.error('7/8/10 parcels require --continuous')
    if args.parcel_count!=3 and args.grasp_mode!='confirmed':parser.error('7/8/10 parcels require --grasp-mode confirmed')
    try:batch=batch_layout(args.parcel_count,args.parcel_pitch)
    except ValueError as exc:parser.error(str(exc))
    os.environ['TWIN_PARCEL_COUNT']=str(batch.count)
    os.environ['TWIN_PARCEL_PITCH']=str(batch.pitch)
    if args.grasp_mode=='confirmed' and not args.continuous:
        parser.error('--grasp-mode confirmed requires --continuous')
    if args.condition_routing and not (args.continuous and batch.extended and args.grasp_mode=='confirmed'):
        parser.error('--condition-routing requires --continuous --grasp-mode confirmed and --parcel-count 7/8/10')
    if not math.isfinite(args.belt_speed) or not .02 <= args.belt_speed <= CONTINUOUS_SPEED:
        parser.error('--belt-speed must be finite and in [.02, .06] m/s for this intercept geometry')
    from twin_moveit_scripts.parcel_task_session import task_session
    try:
        with task_session():
            return _run(args)
    except RuntimeError as exc:
        print('PARCEL_TASK_FAILED: ' + str(exc), flush=True)
        if args.output:
            args.output.write_text(json.dumps({'indexed': args.indexed, 'passed': False,
                                              'events': [], 'error': str(exc)}, indent=2) + '\n')
        return 1
