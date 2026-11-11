"""Isolated ROS/sensor I/O, unaffected by MoveIt Python binding GIL pauses."""
import ctypes
import multiprocessing as mp
import os
import pickle
import queue
import time
from collections import deque

# Wall seconds the simulation clock may stall before the feed is faulted. Sized well
# above the worst GUI frame interval (real-time factor ~0.3) so transient render jitter
# never reads as a freeze, while a genuinely stopped simulator is still caught quickly.
SIM_FREEZE_WALL=3.0

# Sim seconds the detector may go without a published frame before the feed is faulted.
# The detector's image callback jitters under GUI CPU contention (measured gaps up to
# ~0.8 sim-s, confirmed detector-side via an independent subscriber), so this is set
# above that with margin. A genuinely dead perception pipeline still trips it promptly,
# and it is independent of real-time factor (unlike the removed wall-clock budget).
PERCEPTION_STALE_SIM=1.5


def _run(buffer,length,lock,commands,parent_pid,sensors):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.clock import Clock,ClockType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from vision_msgs.msg import Detection3DArray
    from action_msgs.srv import CancelGoal
    from twin_moveit_scripts.parcel_contacts import PadContacts
    from twin_moveit_scripts.parcel_grasp import GripSample,GraspEvaluator,close_command
    from twin_moveit_scripts.parcel_layout import MOVING_DETECTION_TOPIC

    rclpy.init(args=[])
    node=Node('parcel_sensor_io',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(Float64,'/parcel_cell/feed_command',10)
    cancel=node.create_client(CancelGoal,'/arm_controller/follow_joint_trajectory/_action/cancel_goal')
    pads=PadContacts() if sensors else None
    state=dict(message=None,received=0.,tote_message=None,joints=None,joints_at=0.,joint_stamp_ns=0,
               velocities={},fault=None,now_ns=0,packets={})
    widths=deque(maxlen=1000)
    recv_stamps=deque(maxlen=50)  # sim stamps of recent detections (localise receipt gaps)
    recv_gaps=deque(maxlen=50)    # sim-time spacing between consecutive detections
    speed=0.
    monitor=None
    close_start=0.
    last_parent=time.monotonic()
    last_sim_ns=0          # last observed sim clock value (liveness)
    last_sim_wall=time.monotonic()  # wall time at which the sim clock last advanced
    stopping=False

    def detection(msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if recv_stamps and stamp>recv_stamps[-1]:recv_gaps.append((stamp-recv_stamps[-1])/1e9)
        recv_stamps.append(stamp)
        state.update(message=msg,received=time.monotonic())
    def tote(msg):state['tote_message']=msg
    def condition(msg):
        # compact: (sim stamp ns, [((x,y,z), p_damaged), ...]) from the optional condition challenger
        items=[]
        for d in msg.detections:
            pd=next((r.hypothesis.score for r in d.results if r.hypothesis.class_id=='damaged'),None)
            if pd is not None:
                c=d.bbox.center.position;items.append(((c.x,c.y,c.z),pd))
        state['condition']=(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,items)
    def joints(msg):
        q=dict(zip(msg.name,msg.position))
        ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        state.update(joints=q,joints_at=time.monotonic(),joint_stamp_ns=ns,
                     velocities=dict(zip(msg.name,msg.velocity)))
        if 'panda_finger_joint1' in q and 'panda_finger_joint2' in q:
            widths.append((ns/1e9,(q['panda_finger_joint1']+q['panda_finger_joint2'])/2))

    subscriptions=[node.create_subscription(Detection3DArray,MOVING_DETECTION_TOPIC,detection,10),
                   node.create_subscription(Detection3DArray,'/parcel_cell/tote_detections',tote,10),
                   node.create_subscription(JointState,'/joint_states',joints,10),
                   node.create_subscription(Detection3DArray,'/parcel_cell/condition',condition,10)]

    def tick():
        nonlocal speed,monitor,close_start,last_parent,stopping,last_sim_ns,last_sim_wall
        now=node.get_clock().now().nanoseconds
        while True:
            try:kind,value=commands.get_nowait()
            except queue.Empty:break
            last_parent=time.monotonic()
            if kind=='speed':speed=value
            elif kind=='watch':
                monitor=GraspEvaluator();monitor.confirmed=True;monitor.object_id=value[0];close_start=value[1]
            elif kind=='release':monitor=None
            elif kind=='close':stopping=True;speed=0.
        if os.getppid()!=parent_pid or time.monotonic()-last_parent>20:
            state['fault']='Parent controller unavailable';speed=0.;stopping=True
        # Liveness: track wall time at which the SIM clock last advanced. The sim clock
        # ticks every cycle while the simulator runs, so this only grows when sim is
        # genuinely frozen -- unlike a message-cadence budget, it is independent of the
        # GUI real-time factor.
        if now>last_sim_ns:last_sim_ns=now;last_sim_wall=time.monotonic()
        if speed:
            msg=state['message']
            stamp=None if msg is None else msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            age=None if stamp is None else (now-stamp)/1e9
            # Freshness is gated in SIM time (detections must keep up with the sim clock);
            # a separate liveness gate catches a true sim freeze. The old wall-clock
            # receipt budget tripped on low GUI real-time factor and is removed.
            sim_frozen=time.monotonic()-last_sim_wall>SIM_FREEZE_WALL
            if age is None or not -.05<=age<=PERCEPTION_STALE_SIM or sim_frozen:
                state['fault']='Continuous perception watchdog: '+str(dict(
                    source_age_sim=age,sim_frozen=sim_frozen,
                    received_age_wall=time.monotonic()-state['received'],
                    recv_sim_gaps=[round(g,3) for g in list(recv_gaps)[-6:]]))
                speed=0.
        if monitor is not None and state['fault'] is None:
            left,right,age=pads.sample(now)
            q=state['joints'] or {}
            sample=GripSample(state['joint_stamp_ns'],tuple(q.get(n,float('nan')) for n in
                ('panda_finger_joint1','panda_finger_joint2')),
                close_command(state['joint_stamp_ns']/1e9-close_start),left,right,
                (now-state['joint_stamp_ns'])/1e9,age)
            valid,reason=monitor.update(sample)
            if not valid:
                state['fault']='Carried parcel lost: '+reason;speed=0.
        if state['fault'] is not None:
            speed=0.
            if cancel.service_is_ready():cancel.call_async(CancelGoal.Request())
        pub.publish(Float64(data=speed))
        state['now_ns']=now
        state['width_history']=list(widths)
        if pads:
            with pads.lock:state['packets']=dict(pads.packets)
        payload=pickle.dumps(state,protocol=5)
        if len(payload)>len(buffer):raise RuntimeError('Parcel I/O snapshot exceeds capacity')
        with lock:
            ctypes.memmove(ctypes.addressof(buffer),payload,len(payload));length.value=len(payload)

    timer=node.create_timer(.02,tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        while rclpy.ok() and not stopping:rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        pub.publish(Float64(data=0.))
        if pads:pads.close()
        node.destroy_node();rclpy.try_shutdown()


class IsolatedIO:
    def __init__(self,sensors):
        ctx=mp.get_context('spawn')
        self.buffer=ctx.RawArray(ctypes.c_ubyte,262144)
        self.length=ctx.Value('i',0,lock=False)
        self.lock=ctx.Lock()
        self.commands=ctx.Queue()
        self.last_keepalive=0.
        self.process=ctx.Process(target=_run,args=(self.buffer,self.length,self.lock,self.commands,os.getpid(),sensors),
                                 daemon=True)
        self.process.start()

    def send(self,kind,value=None):self.commands.put((kind,value))

    def snapshot(self):
        if time.monotonic()-self.last_keepalive>.2:
            self.send('alive');self.last_keepalive=time.monotonic()
        if not self.process.is_alive():raise RuntimeError('Parcel sensor I/O process stopped')
        with self.lock:
            data=ctypes.string_at(ctypes.addressof(self.buffer),self.length.value)
        return pickle.loads(data) if data else None

    def close(self):
        self.send('close')
        self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.terminate();self.process.join(timeout=2)
        self.commands.close()
        self.commands.join_thread()
        self.process.close()
