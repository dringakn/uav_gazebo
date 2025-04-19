#!/usr/bin/env python3
"""
rucking python bindings installation:
    cd /tmp
    git clone https://github.com/dringakn/ruckig.git
    cd ruckig
    pip3 install .
    TIP:
        if encountering issue: AttributeError: module 'lib' has no attribute 'X509_V_FLAG_NOTIFY_POLICY'
            cd /tmp
            curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
        This will wipe out the system‐broken pip and re‐install a fresh copy
            sudo python3 get-pip.py
        Upgrade setuptools/wheel under the fresh pip
            pip3 install --upgrade setuptools wheel
"""

import time
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from ruckig import InputParameter, Ruckig, Trajectory, Result

class ScurveTrajectoryBuilder:
    def __init__(self, axes=3):
        self.axes = axes
        self.otg = Ruckig(axes)
        self.trajectory = Trajectory(axes)

    def configure(self, current, target, limits):
        inp = InputParameter(self.axes)
        inp.current_position, inp.current_velocity, inp.current_acceleration = current
        inp.target_position,  inp.target_velocity,  inp.target_acceleration  = target
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = limits['max']
        inp.min_velocity, inp.min_acceleration             = limits['min']
        return inp

    def build_path(self, inp, frame_id, dt):
        # measure generation time
        t0 = time.perf_counter()
        result = self.otg.calculate(inp, self.trajectory)
        gen_time = time.perf_counter() - t0

        if result != Result.Working:
            raise RuntimeError(f"Ruckig failed: {result}")

        steps = int(self.trajectory.duration / dt) + 1
        path = Path(header=Header(frame_id=frame_id, stamp=rospy.Time.now()))

        for i in range(steps):
            t = i * dt
            pos, vel, acc = self.trajectory.at_time(t)
            ps = PoseStamped(header=path.header)
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
            path.poses.append(ps)

        # richer logging
        rospy.loginfo(
            "DOF: %d | Duration: %.3fs | Samples: %d | Gen time: %.4fs",
            self.trajectory.degrees_of_freedom,
            self.trajectory.duration,
            len(path.poses),
            gen_time
        )
        rospy.loginfo(
            "Stages: %s | Extrema: %s | Profiles: %s",
            self.trajectory.intermediate_durations,
            self.trajectory.position_extrema,
            self.trajectory.profiles
        )

        return path
    
    
class ScurveTrajectoryNode:
    def __init__(self):
        rospy.init_node('scurve_trajectory_node')
        self.pub      = rospy.Publisher('ruckig_path', Path, latch=True, queue_size=1)
        self.dt       = rospy.get_param('~sample_dt', 0.02)
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.builder  = ScurveTrajectoryBuilder()

        # load static parameters here—or expose a service to reconfigure at runtime
        current = (
            [0.0,  0.0,  0.5],
            [0.0, -2.2, -0.5],
            [0.0,  2.5, -0.5],
        )
        target = (
            [ 5.0, -2.0, -3.5],
            [ 0.0, -0.5, -2.0],
            [ 0.0,  0.0,  0.5],
        )
        # Velocity, acceleration, jerk limits
        # Note: The jerk min is not required.
        limits = {
            'max': ([3.0, 1.0, 3.0], [3.0, 2.0, 1.0], [4.0, 3.0, 2.0]),
            'min': ([-1.0, -0.5, -3.0], [-2.0, -1.0, -2.0]),
        }
        self.inp = self.builder.configure(current, target, limits)

    def run(self):
        try:
            path = self.builder.build_path(self.inp, self.frame_id, self.dt)
            self.pub.publish(path)
            rospy.loginfo("Published /ruckig_path")
            rospy.spin()
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    ScurveTrajectoryNode().run()
