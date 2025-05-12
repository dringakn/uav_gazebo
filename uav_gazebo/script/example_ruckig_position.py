#!/usr/bin/env python3
"""
Conceptual Description:
-----------------------
This ROS 1 node (Python) generates jerk‑limited 3‑DoF trajectories with Ruckig,
computes the required joint torques via KDL inverse dynamics (zero external wrenches),
and publishes both a JointTrajectory (for controllers/simulators)
and a JointState (for RViz) with realistic effort values.

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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF

from ruckig import Ruckig, InputParameter, OutputParameter, Result


def load_vec3(param_name, default):
    val = rospy.get_param(param_name, default)
    if not isinstance(val, list) or len(val) != 3:
        rospy.logwarn("Param '%s' missing or wrong size, using default %s",
                      param_name, default)
        val = default
    return val


def main():
    rospy.init_node('ruckig_dynamics_node')
    # Publishers
    traj_pub = rospy.Publisher('joint_trajectory',
                               JointTrajectory, queue_size=1)
    js_pub   = rospy.Publisher('joint_states',
                               JointState,      queue_size=1)

    # Build KDL chain from URDF on /robot_description
    robot = URDF.from_parameter_server()
    ok, kdl_tree = treeFromUrdfModel(robot)
    if not ok:
        rospy.logfatal("Could not parse URDF")
        return

    root = rospy.get_param('~root_link', 'world')
    tip  = rospy.get_param('~tip_link',  'link3')
    kdl_chain = kdl_tree.getChain(root, tip)

    # Gravity vector
    gravity = kdl.Vector(0.0, 0.0, -9.81)
    # Use ChainDynParam to compute M, C, G
    dyn_solver = kdl.ChainDynParam(kdl_chain, gravity)

    DOF = 3
    # Preallocate arrays/matrices
    q     = kdl.JntArray(DOF)
    qdot  = kdl.JntArray(DOF)
    qddot = kdl.JntArray(DOF)
    tau   = kdl.JntArray(DOF)
    M     = kdl.JntSpaceInertiaMatrix(DOF)
    C     = kdl.JntArray(DOF)
    G     = kdl.JntArray(DOF)

    # Ruckig setup
    control_dt = rospy.get_param('~control_dt', 0.01)
    otg    = Ruckig(DOF, control_dt)
    inp    = InputParameter(DOF)
    outp   = OutputParameter(DOF)

    joint_names = rospy.get_param('~joint_names',
                                  ['joint1', 'joint2', 'joint3'])

    # Load parameters
    inp.max_velocity     = load_vec3('~max_velocity',     [3.0, 1.0, 3.0])
    inp.max_acceleration = load_vec3('~max_acceleration', [3.0, 2.0, 1.0])
    inp.max_jerk         = load_vec3('~max_jerk',         [4.0, 3.0, 2.0])

    inp.current_position     = load_vec3('~initial_position',     [0.0, 0.0, 0.5])
    inp.current_velocity     = load_vec3('~initial_velocity',     [0.0, -2.2, -0.5])
    inp.current_acceleration = load_vec3('~initial_acceleration', [0.0,  2.5, -0.5])

    inp.target_position     = load_vec3('~target_position',     [ 5.0, -2.0, -3.5])
    inp.target_velocity     = load_vec3('~target_velocity',     [ 0.0, -0.5, -2.0])
    inp.target_acceleration = load_vec3('~target_acceleration', [ 0.0,  0.0,  0.5])

    rate = rospy.Rate(1.0 / control_dt)
    while not rospy.is_shutdown():
        if otg.update(inp, outp) == Result.Working:
            # Publish JointTrajectory
            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names  = joint_names

            pt = JointTrajectoryPoint()
            pt.positions       = list(outp.new_position)
            pt.velocities      = list(outp.new_velocity)
            pt.accelerations   = list(outp.new_acceleration)
            pt.time_from_start = rospy.Duration(outp.time)
            traj.points = [pt]
            traj_pub.publish(traj)

            # Fill KDL arrays
            for i in range(DOF):
                q[i]     = outp.new_position[i]
                qdot[i]  = outp.new_velocity[i]
                qddot[i] = outp.new_acceleration[i]

            # Compute M(q), C(q,qd), G(q)
            dyn_solver.JntToMass(q, M)
            dyn_solver.JntToCoriolis(q, qdot, C)
            dyn_solver.JntToGravity(q, G)

            # τ = M(q)·q̈ + C(q, q̇) + G(q)
            for i in range(DOF):
                acc_term = sum(M[i,j] * qddot[j] for j in range(DOF))
                tau[i] = acc_term + C[i] + G[i]

            # Publish JointState with computed effort
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name     = joint_names
            js.position = pt.positions
            js.velocity = pt.velocities
            js.effort   = [tau[i] for i in range(DOF)]
            js_pub.publish(js)

            # Feedback for the next cycle
            outp.pass_to_input(inp)

        rate.sleep()


if __name__ == '__main__':
    main()