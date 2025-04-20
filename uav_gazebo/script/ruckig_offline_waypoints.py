#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from copy import copy

def compute_trajectory():
    otg = Ruckig(3, 0.01, 10)
    inp = InputParameter(3)
    out = OutputParameter(3, 10)

    # start state
    inp.current_position     = [0.2,  0.0, -0.3]
    inp.current_velocity     = [0.0,  0.2,  0.0]
    inp.current_acceleration = [0.0,  0.6,  0.0]

    # waypoints
    inp.intermediate_positions = [
        [ 1.4, -1.6,  1.0],
        [-0.6, -0.5,  0.4],
        [-0.4, -0.35, 0.0],
        [ 0.8,  1.8, -0.1],
    ]

    # final target
    inp.target_position     = [0.5, 1.0, 0.0]
    inp.target_velocity     = [0.2, 0.0, 0.3]
    inp.target_acceleration = [0.0, 0.1,-0.1]

    # limits
    inp.max_velocity     = [1.0, 2.0, 1.0]
    inp.max_acceleration = [3.0, 2.0, 2.0]
    inp.max_jerk         = [6.0,10.0,20.0]

    history = []
    status = Result.Working
    while status == Result.Working:
        status = otg.update(inp, out)
        history.append(copy(out))
        out.pass_to_input(inp)

    return history

def make_path_msg(history):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    for sample in history:
        p = PoseStamped()
        p.header = path.header
        # map joint positions to x,y,z
        p.pose.position.x = sample.new_position[0]
        p.pose.position.y = sample.new_position[1]
        p.pose.position.z = sample.new_position[2]
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        path.poses.append(p)
    return path

if __name__ == "__main__":
    rospy.init_node("ruckig_path_publisher", anonymous=True)
    pub = rospy.Publisher("/ruckig_path", Path, queue_size=1, latch=True)
    rospy.loginfo("Computing Ruckig trajectory…")
    traj = compute_trajectory()
    path_msg = make_path_msg(traj)
    pub.publish(path_msg)
    rospy.loginfo("Published %d poses to /ruckig_path", len(path_msg.poses))
    rospy.spin()
