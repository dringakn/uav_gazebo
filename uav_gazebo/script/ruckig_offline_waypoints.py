#!/usr/bin/env python3
import rospy
import json
import time
from copy import copy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ruckig import Ruckig, Trajectory, InputParameter, OutputParameter, Result

class TermColor:
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    BLUE   = '\033[94m'
    BOLD   = '\033[1m'
    END    = '\033[0m'
    
class WaypointLoader:
    @staticmethod
    def load_from_json(path):
        try:
            with open(path, 'r') as f:
                data = json.load(f)
        except (IOError, ValueError) as e:
            rospy.logerr(f"{TermColor.RED}Failed to load waypoints from '{path}': {e}{TermColor.END}")
            return data.get('header', {}).get('frame_id', 'world'), []
        frame = data.get('header', {}).get('frame_id', 'world')
        poses = [
            [p['pose']['position'][axis] for axis in ('x','y','z')]
            for p in data.get('poses', [])
        ]
        return frame, poses

    @staticmethod
    def path_length(poses):
        """
        Compute the total Euclidean length of a sequence of 3‑D poses.
        :param poses: list of [x, y, z]
        :return: float total length
        """
        if not poses or len(poses) < 2:
            return 0.0
        length = 0.0
        prev = poses[0]
        for curr in poses[1:]:
            dx, dy, dz = curr[0] - prev[0], curr[1] - prev[1], curr[2] - prev[2]
            length += (dx*dx + dy*dy + dz*dz) ** 0.5
            prev = curr
        return length
    
class RuckigPlanner:
    def __init__(self, dof, dt, buffer_size, start, target, limits, waypoints):
        self.dof = dof
        self.otg = Ruckig(dof, dt, buffer_size)
        self.trajectory = Trajectory(dof)
        self.inp = InputParameter(dof)
        self.out = OutputParameter(dof, buffer_size)

        # unpack
        (self.inp.current_position,
         self.inp.current_velocity,
         self.inp.current_acceleration) = start

        (self.inp.target_position,
         self.inp.target_velocity,
         self.inp.target_acceleration) = target

        (self.inp.max_velocity,
         self.inp.max_acceleration,
         self.inp.max_jerk) = limits

        self.inp.intermediate_positions = waypoints or []

    def compute(self):
        history = []
        status = Result.Working
        while status == Result.Working:
            status = self.otg.update(self.inp, self.out)
            history.append(copy(self.out))
            self.out.pass_to_input(self.inp)
        return history

class RuckigPathPublisherNode:
    def __init__(self):
        rospy.init_node('ruckig_path_publisher')

        # --- load params ---
        # file & topic
        wp_file        = rospy.get_param('~waypoints_file', '/root/personal_ws/src/uav_gazebo/uav_gazebo/missions/path_lifted.json')
        path_topic     = rospy.get_param('~path_topic', '/ruckig_path')
        buffer_size    = rospy.get_param('~buffer_size', 10)
        dt             = rospy.get_param('~sample_dt', 0.02)

        # start, target, limits: expect lists of lists [[pos], [vel], [acc]]
        start   = rospy.get_param('~start_state',   [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        target  = rospy.get_param('~target_state',  [[0.5, 0.5,  0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        # limits  = rospy.get_param('~limits',        [[1.0, 2.0, 1.0], [3.0, 2.0, 2.0], [6.0,10.0,20.0]])
        self.max_vel = rospy.get_param('~max_velocity', 1.0)
        self.max_acc = rospy.get_param('~max_acceleration', 10.0)
        self.max_jerk = rospy.get_param('~max_jerk', 10.0)        
        limits = (
            [self.max_vel, self.max_vel, self.max_vel], 
            [self.max_acc, self.max_acc, self.max_acc], 
            [self.max_jerk, self.max_jerk, self.max_jerk]
        )

        # load waypoints
        if wp_file:
            frame, waypoints = WaypointLoader.load_from_json(wp_file)
            path_length = WaypointLoader.path_length(waypoints)
            rospy.loginfo(f"{TermColor.BOLD}{TermColor.GREEN}Loaded {len(waypoints)} waypoints (length: {path_length:.2f}m) from '{wp_file}'{TermColor.END}")
        else:
            frame, waypoints = 'world', []

        # infer DOF
        dof = len(start[0])
        rospy.loginfo(f"{TermColor.BLUE}Using DOF={dof}, dt={dt}, buffer_size={buffer_size}{TermColor.END}")

        # setup planner
        planner = RuckigPlanner(
            dof, dt, buffer_size,
            start=start, target=target,
            limits=limits, waypoints=waypoints
        )

        pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)

        # --- timing & compute ---
        t0 = time.perf_counter()
        history = planner.compute()
        gen_time = time.perf_counter() - t0

        # build & publish
        path_msg, traj_exec_time, traj_length = self._make_path_msg(history, frame)
        pub.publish(path_msg)

        rospy.loginfo(f"{TermColor.BOLD}{TermColor.GREEN}Published {len(path_msg.poses)} poses on '{path_topic}' (frame='{frame}'){TermColor.END}")
        rospy.loginfo(f"{TermColor.BLUE}Trajectory generation took {TermColor.YELLOW}{gen_time:0.3f}s{TermColor.END}")
        rospy.loginfo(f"{TermColor.BLUE}Expected Trajectory execution time {TermColor.YELLOW}{traj_exec_time:0.1f}s{TermColor.END}")
        rospy.loginfo(f"{TermColor.BLUE}Total trajectory length: {TermColor.YELLOW}{traj_length:0.2f}m{TermColor.END}")

        # rospy.spin()

    def _make_path_msg(self, history, frame):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = rospy.Time.now()

        traj_exec_time = 0.0        
        traj_total_length = 0.0
        prev = history[0].new_position
        for sample in history:
            p = PoseStamped(header=path.header)
            x, y, z = sample.new_position
            traj_exec_time = sample.time            
            p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
            p.pose.orientation.w = 1.0 # identity quaternion
            path.poses.append(p)
            traj_total_length += math.sqrt(
                (x - prev[0])**2 + (y - prev[1])**2 + (z - prev[2])**2
            )
            prev = sample.new_position
            
        return path, traj_exec_time, traj_total_length
    
if __name__ == '__main__':
    RuckigPathPublisherNode()
