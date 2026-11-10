#!/usr/bin/env python3
"""Verify experimental preflight and existing motion guard inhibit dispatch."""
import unittest
import os
from unittest.mock import Mock,MagicMock, patch
from geometry_msgs.msg import PoseStamped
from twin_moveit_scripts import move_to
from twin_moveit_scripts.arrival_gate import ArrivalGate, ArrivalProfile
from twin_moveit_scripts.gripper_deadline import SimulationDeadline


class DispatchTest(unittest.TestCase):
    def test_simulated_gripper_wait_allows_slow_playback(self):
        deadline=SimulationDeadline(0.,4.5)
        self.assertIsNone(deadline.check(0.,100.))
        for wall in range(1,21):
            self.assertIsNone(deadline.check(wall,100.+wall*.15))
        self.assertIn('simulation deadline',deadline.check(21.,104.5))

    def test_gripper_clock_bootstrap_freeze_reset_and_wall_cap(self):
        deadline=SimulationDeadline(0.,4.5)
        self.assertIsNone(deadline.check(1.,None))
        self.assertIsNone(deadline.check(2.,180.))  # fresh, not stale cached time
        self.assertIsNone(deadline.check(3.,180.1))
        self.assertIn('reset',deadline.check(4.,1.))
        self.assertIn('frozen',SimulationDeadline(0.,4.5).check(3.,None))
        deadline=SimulationDeadline(0.,4.5)
        deadline.check(0.,180.)
        self.assertIn('frozen',deadline.check(3.,180.))
        self.assertIn('hard wall',SimulationDeadline(0.,4.5).check(120.,1.))
        self.assertIn('invalid',SimulationDeadline(0.,4.5).check(1.,float('nan')))

    def test_ten_gripper_timeout_requests_cancellation(self):
        client, executor, twin, logger=(Mock() for _ in range(4))
        handle=client.send_goal_async.return_value.result.return_value
        with patch.dict(os.environ,dict(TWIN_PARCEL_COUNT='10')), \
             patch.object(move_to,'_spin_until'), \
             patch.object(move_to,'_wait_ten_finger_result',return_value='simulation clock frozen'):
            ok,width,error=move_to._send_finger_goal(client,executor,twin,.04,logger)
        self.assertFalse(ok)
        self.assertIn('frozen',error)
        handle.cancel_goal_async.assert_called_once()

    def test_ten_clock_uses_installed_jazzy_qos_api(self):
        # Unmocked import path: GUI02 died on a nonexistent rclpy.qos name here.
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        twin=Mock()
        clock=move_to._TenClock(twin)
        topic_args=twin.create_subscription.call_args.args
        self.assertIs(topic_args[0],Clock)
        self.assertEqual(topic_args[1],'/clock')
        profile=topic_args[3]
        self.assertEqual(profile.depth,1)
        self.assertEqual(profile.reliability,ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(profile.durability,DurabilityPolicy.VOLATILE)
        message=Mock();message.clock.sec=12;message.clock.nanosec=500000000
        clock._on_clock(message)
        self.assertEqual(clock.sample,12.5)
        clock.close()
        twin.destroy_subscription.assert_called_once_with(clock.subscription)

    def test_ten_clock_failure_happens_before_goal_dispatch(self):
        client, executor, twin, logger=(Mock() for _ in range(4))
        twin.create_subscription.side_effect=RuntimeError('no clock')
        with patch.dict(os.environ,dict(TWIN_PARCEL_COUNT='10')), \
             patch.object(move_to,'_spin_until'):
            with self.assertRaisesRegex(RuntimeError,'no clock'):
                move_to._send_finger_goal(client,executor,twin,.04,logger)
        client.send_goal_async.assert_not_called()

    def test_ten_wait_exception_cancels_accepted_goal_and_closes_clock(self):
        client, executor, twin, logger=(Mock() for _ in range(4))
        handle=client.send_goal_async.return_value.result.return_value
        with patch.dict(os.environ,dict(TWIN_PARCEL_COUNT='10')), \
             patch.object(move_to,'_spin_until'), \
             patch.object(move_to,'_wait_ten_finger_result',side_effect=RuntimeError('boom')):
            with self.assertRaisesRegex(RuntimeError,'boom'):
                move_to._send_finger_goal(client,executor,twin,.04,logger)
        handle.cancel_goal_async.assert_called_once()
        twin.destroy_subscription.assert_called_once()

    def test_ten_wait_real_loop_reports_frozen_clock_and_finishes_on_result(self):
        class Future:
            def __init__(self,finish_after=None):
                self.spins=0;self.finish_after=finish_after
            def done(self):
                return self.finish_after is not None and self.spins>=self.finish_after
        executor=Mock();clock=Mock();clock.sample=None
        future=Future()
        executor.spin_once.side_effect=lambda timeout_sec: setattr(future,'spins',future.spins+1)
        ticks=iter([0.,.5,1.,3.5,4.])
        with patch.object(move_to.time,'monotonic',side_effect=lambda:next(ticks)):
            error=move_to._wait_ten_finger_result(executor,Mock(),future,clock)
        self.assertIn('frozen',error)
        done=Future(finish_after=2)
        executor.spin_once.side_effect=lambda timeout_sec: setattr(done,'spins',done.spins+1)
        self.assertIsNone(move_to._wait_ten_finger_result(executor,Mock(),done,clock))

    def test_ten_retime_reserves_margin_inside_unchanged_limits(self):
        with patch.dict(os.environ,dict(TWIN_PARCEL_COUNT='10',TWIN_PLANNER_ID='RRTConnectkConfigDefault',
                TWIN_TRAJECTORY_CHECK='sampled',TWIN_SMOOTHER='off',TWIN_MOTION_METRICS='')):
            cfg=move_to.build_config_dict(Mock())
            self.assertAlmostEqual(cfg['ompl']['plan_request_params']['max_acceleration_scaling_factor'],.495)
            self.assertAlmostEqual(move_to._MOTION_LIMITS['panda_joint1']['acceleration_max'],7.5)

    def test_ten_ik_rejects_colliding_branch_before_planning(self):
        robot=Mock();scene=MagicMock();reference=Mock();first=Mock();second=Mock()
        reference.get_joint_group_positions.return_value=[0.]*7
        for candidate in (first,second):
            candidate.set_from_ik.return_value=True
            candidate.get_joint_group_positions.return_value=[0.]*7
        robot.get_planning_scene_monitor.return_value.read_only.return_value=scene
        scene.__enter__.return_value=scene
        scene.is_state_colliding.side_effect=[True,False]
        with patch.object(move_to,'_MOTION_LIMITS',None), \
             patch.object(move_to.copy,'deepcopy',side_effect=[reference,first,second]), \
             patch.object(move_to.motion_audit,'record'):
            chosen=move_to._ten_profile_goal(robot,PoseStamped(),Mock())
        self.assertIs(chosen,second)
        self.assertEqual(scene.is_state_colliding.call_count,2)

    def test_default_pipeline_and_smoother_gate(self):
        flags = dict(TWIN_SMOOTHER='off', TWIN_PLANNER_ID='RRTConnectkConfigDefault',
                     TWIN_TRAJECTORY_CHECK='off', TWIN_MOTION_METRICS='')
        with patch.dict(os.environ, flags):
            config = move_to.build_config_dict(Mock())
            self.assertEqual(config['planning_pipelines']['pipeline_names'], ['ompl'])
            self.assertEqual(config['ompl']['planning_plugins'], ['ompl_interface/OMPLPlanner'])
        with patch.dict(os.environ, {**flags, 'TWIN_SMOOTHER': 'stomp'}):
            with self.assertRaisesRegex(ValueError, 'requires'):
                move_to.build_config_dict(Mock())
        with patch.dict(os.environ, {**flags, 'TWIN_SMOOTHER': 'stomp',
                                     'TWIN_TRAJECTORY_CHECK': 'sampled'}):
            config = move_to.build_config_dict(Mock())
            self.assertIn('ompl_smooth', config['planning_pipelines']['pipeline_names'])
            self.assertEqual(config['ompl_smooth']['planning_plugins'],
                             ['ompl_interface/OMPLPlanner', 'stomp_moveit/StompPlanner'])

    def test_invalid_candidate_is_not_dispatched(self):
        robot, arm, logger, params = (Mock() for _ in range(4))
        with patch.object(move_to, '_timed_plan', return_value=Mock()), \
             patch.object(move_to, '_audit_candidate', side_effect=ValueError('VALIDATION_FAILED')), \
             patch.object(move_to, '_settle_pose', return_value=(0, 0, 0)):
            result = move_to._plan_execute(robot, arm, logger, params)
        self.assertFalse(result.ok)
        self.assertIn('VALIDATION_FAILED', result.error)
        robot.execute.assert_not_called()

    def test_grasp_loss_prevents_planning(self):
        robot, arm, logger, params = (Mock() for _ in range(4))
        def lost():
            raise RuntimeError('GRASP_LOST')
        with move_to.guarded_motion(lost):
            with self.assertRaisesRegex(RuntimeError, 'GRASP_LOST'):
                move_to._plan_execute(robot, arm, logger, params)
        arm.plan.assert_not_called()
        robot.execute.assert_not_called()

    def test_shadow_hook_records_without_authorizing_motion(self):
        profile = ArrivalProfile((.01,)*7, (.05,)*7, (False,)*7, .2, .1, 2)
        node = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 100000000
        command = dict(command_id='test', target=[0]*7, gate=ArrivalGate(profile),
                       controller_succeeded=True)
        with patch.object(move_to, '_SHADOW_COMMAND', command), \
             patch.object(move_to, '_SHADOW_NODE', node), \
             patch.object(move_to._on_joint_states, 'arm_sample',
                          dict(stamp=.1, positions=[0]*7, velocities=[0]*7)), \
             patch.object(move_to.motion_audit, 'record') as record:
            move_to._shadow_arrival_tick()
            self.assertEqual(command['status'], 'DWELLING')
            move_to._shadow_arrival_end('existing_pose_dwell')
            self.assertFalse(record.call_args.kwargs['execution_authorized'])
            self.assertEqual(record.call_args.kwargs['status'], 'DWELLING')


if __name__ == '__main__':
    unittest.main()
