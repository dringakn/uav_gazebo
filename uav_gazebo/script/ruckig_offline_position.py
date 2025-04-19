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

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from ruckig import InputParameter, Ruckig, Trajectory, Result

def build_scurve_path():
    # Define current & target states and limits
    inp = InputParameter(3)
    inp.current_position     = [0.0,  0.0,  0.5]
    inp.current_velocity     = [0.0, -2.2, -0.5]
    inp.current_acceleration = [0.0,  2.5, -0.5]

    inp.target_position     = [ 5.0, -2.0, -3.5]
    inp.target_velocity     = [ 0.0, -0.5, -2.0]
    inp.target_acceleration = [ 0.0,  0.0,  0.5]

    inp.max_velocity     = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk         = [4.0, 3.0, 2.0]
    inp.min_velocity     = [-1.0,  -0.5, -3.0]
    inp.min_acceleration = [-2.0,  -1.0, -2.0]

    # Compute trajectory
    otg        = Ruckig(3)
    trajectory = Trajectory(3)
    result = otg.calculate(inp, trajectory)
    if result != Result.Working:
        rospy.logerr("Trajectory generation failed: %s", result)
        return None

    # 3) Sample at fixed dt
    dt = rospy.get_param('~sample_dt', 0.02)  # 50 Hz by default
    times = [i * dt for i in range(int(trajectory.duration/dt) + 1)]
    path = Path()
    path.header = Header(frame_id=rospy.get_param('~frame_id', 'world'),
                         stamp=rospy.Time.now())

    for t in times:
        pos, _, _ = trajectory.at_time(t)
        pose = PoseStamped()
        pose.header = path.header
        pose.header.stamp = rospy.Time.now()  # or path.header.stamp + rospy.Duration(t)
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        # orientation unused (kept identity)
        path.poses.append(pose)

    rospy.loginfo("Built Path: duration=%.3f s, samples=%d", 
                  trajectory.duration, len(path.poses))
    return path

def main():
    rospy.init_node('scurve_trajectory_node')
    pub = rospy.Publisher('ruckig_path', Path, latch=True, queue_size=1)
    path = build_scurve_path()
    if path:
        pub.publish(path)
        rospy.loginfo("Published /ruckig_path")
    rospy.spin()

if __name__ == '__main__':
    main()
