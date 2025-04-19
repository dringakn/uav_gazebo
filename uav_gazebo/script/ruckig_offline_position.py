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

#!/usr/bin/env python3
import json
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

    def build_segment(self, inp, dt, start_time):
        """Plan one segment, return list of (t_offset, pos)."""
        t0 = time.perf_counter()
        res = self.otg.calculate(inp, self.trajectory)
        gen = time.perf_counter() - t0
        if res != Result.Working:
            raise RuntimeError(f"Segment failed: {res}")

        steps = int(self.trajectory.duration / dt) + 1
        samples = []
        for i in range(steps):
            t = i * dt
            pos, vel, acc = self.trajectory.at_time(t)
            samples.append((start_time + t, pos))
        # return end‐velocity & acceleration for chaining
        # query the final velocity & acceleration
        _, end_vel, end_acc = self.trajectory.at_time(self.trajectory.duration)
        
        return samples, self.trajectory.duration, gen, end_vel, end_acc

    def build_from_waypoints(self, json_path, limits, dt, frame_id):
        with open(json_path) as f:
            data = json.load(f)

        wp = [(
            p['pose']['position']['x'],
            p['pose']['position']['y'],
            p['pose']['position']['z']
        ) for p in data['poses']]

        # initialize header
        now = rospy.Time.now()
        path = Path(header=Header(frame_id=frame_id, stamp=now))

        # start from zero-state
        cur_vel = [0.]*self.axes
        cur_acc = [0.]*self.axes
        t_offset = 0.0
        total_gen = 0.0

        for i in range(len(wp)-1):
            current = (wp[i], cur_vel, cur_acc)
            target  = (wp[i+1], [0.]*self.axes, [0.]*self.axes)
            inp     = self.configure(current, target, limits)

            samples, dur, gen, cur_vel, cur_acc = \
                self.build_segment(inp, dt, t_offset)
            total_gen += gen

            # append each sample as PoseStamped
            for t_rel, pos in samples:
                ps = PoseStamped(header=path.header)
                ps.header.stamp = now + rospy.Duration(t_rel)
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
                path.poses.append(ps)

            rospy.loginfo("Segment %d→%d: dur=%.3fs gen=%.4fs", i, i+1, dur, gen)
            t_offset += dur

        rospy.loginfo("Combined: segments=%d total_samples=%d total_gen=%.4fs",
                      len(wp)-1, len(path.poses), total_gen)
        return path

class ScurveTrajectoryNode:
    def __init__(self):
        rospy.init_node('scurve_trajectory_node')
        self.pub      = rospy.Publisher('ruckig_path', Path, latch=True, queue_size=1)
        self.dt       = rospy.get_param('~sample_dt', 0.02)
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.wayfile  = rospy.get_param('~waypoints_file', '/root/catkin_ws/src/uav_gazebo/uav_gazebo/missions/path.json')

        limits = {
            'max': ([3.0,1.0,3.0], [3.0,2.0,1.0], [4.0,3.0,2.0]),
            'min': ([-1.0,-0.5,-3.0], [-2.0,-1.0,-2.0]),
        }
        self.builder = ScurveTrajectoryBuilder()
        self.limits  = limits

    def run(self):
        try:
            path = self.builder.build_from_waypoints(
                self.wayfile, self.limits, self.dt, self.frame_id
            )
            self.pub.publish(path)
            rospy.loginfo("Published /ruckig_path with %d poses", len(path.poses))
            rospy.spin()
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    ScurveTrajectoryNode().run()
