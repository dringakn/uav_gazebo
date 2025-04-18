/**
 * Conceptual Description:
 * -----------------------
 * This ROS1 node generates jerk‑limited 3‑DoF trajectories with Ruckig,
 * computes the required joint torques via KDL inverse dynamics (including external wrenches as zero),
 * and publishes both JointTrajectory (for controllers) and JointState (for RViz) with realistic efforts.
 */

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <ruckig/ruckig.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>  // for KDL::Wrench and related frame types
#include <vector>
#include <string>

using namespace ruckig;

int main(int argc, char** argv) {
    // --- ROS initialization ---
    ros::init(argc, argv, "ruckig_dynamics_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Publishers
    auto traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);
    auto js_pub   = nh.advertise<sensor_msgs::JointState>("joint_states",     1);

    // --- Load robot model from parameter server ---
    std::string urdf_string;
    if (!nh.getParam("/robot_description", urdf_string)) {
        ROS_FATAL("Failed to get /robot_description");
        return 1;
    }

    // Parse into KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree)) {
        ROS_FATAL("Failed to construct KDL tree");
        return 1;
    }

    // Extract the chain from 'world' to 'link3'
    KDL::Chain kdl_chain;
    if (!kdl_tree.getChain("world", "link3", kdl_chain)) {
        ROS_FATAL("Failed to extract chain from world to link3");
        return 1;
    }

    // Inverse dynamics solver with gravity along -Z
    KDL::Vector gravity(0.0, 0.0, -9.81);
    KDL::ChainIdSolver_RNE id_solver(kdl_chain, gravity);

    // Prepare a zero external wrench for each segment
    KDL::Wrenches f_ext(kdl_chain.getNrOfSegments(), KDL::Wrench::Zero());

    const unsigned int DOF = 3;
    KDL::JntArray q(DOF), qdot(DOF), qddot(DOF), torques(DOF);

    // --- Ruckig setup ---
    const double control_dt = 0.01;  // 10 ms
    Ruckig<DOF> otg(control_dt);
    InputParameter<DOF>  input;
    OutputParameter<DOF> output;

    std::vector<std::string> joint_names = { "joint1", "joint2", "joint3" };

    // Helper to load a 3‑element vector from params (or default)
    auto load_vec3 = [&](const std::string& name, const std::vector<double>& def, auto& out_array){
        std::vector<double> tmp;
        if (!pnh.getParam(name, tmp) || tmp.size() != DOF) {
            tmp = def;
            ROS_WARN_STREAM("Param '" << name << "' missing or wrong size, using default");
        }
        for (size_t i = 0; i < DOF; ++i) {
            out_array[i] = tmp[i];
        }
    };

    // Load limits
    load_vec3("max_velocity",     {3.0, 1.0, 3.0},     input.max_velocity);
    load_vec3("max_acceleration", {3.0, 2.0, 1.0},     input.max_acceleration);
    load_vec3("max_jerk",         {4.0, 3.0, 2.0},     input.max_jerk);

    // Load initial state
    load_vec3("initial_position",     {0.0, 0.0, 0.5}, input.current_position);
    load_vec3("initial_velocity",     {0.0,-2.2,-0.5}, input.current_velocity);
    load_vec3("initial_acceleration", {0.0, 2.5,-0.5}, input.current_acceleration);

    // Load target state
    load_vec3("target_position",      {5.0,-2.0,-3.5}, input.target_position);
    load_vec3("target_velocity",      {0.0,-0.5,-2.0}, input.target_velocity);
    load_vec3("target_acceleration",  {0.0, 0.0, 0.5}, input.target_acceleration);

    // --- Main loop at 100 Hz ---
    ros::Rate rate(1.0 / control_dt);
    while (ros::ok()) {
        if (otg.update(input, output) == Result::Working) {
            // Publish JointTrajectory
            trajectory_msgs::JointTrajectory traj_msg;
            traj_msg.header.stamp = ros::Time::now();
            traj_msg.joint_names  = joint_names;

            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions      = { output.new_position[0],     output.new_position[1],     output.new_position[2] };
            pt.velocities     = { output.new_velocity[0],     output.new_velocity[1],     output.new_velocity[2] };
            pt.accelerations  = { output.new_acceleration[0], output.new_acceleration[1], output.new_acceleration[2] };
            pt.time_from_start = ros::Duration(output.time);

            traj_msg.points.push_back(pt);
            traj_pub.publish(traj_msg);

            // Compute inverse dynamics (torques), supplying zero external wrenches
            for (unsigned int i = 0; i < DOF; ++i) {
                q(i)     = output.new_position[i];
                qdot(i)  = output.new_velocity[i];
                qddot(i) = output.new_acceleration[i];
            }
            id_solver.CartToJnt(q, qdot, qddot, f_ext, torques);

            // Publish JointState with effort
            sensor_msgs::JointState js_msg;
            js_msg.header.stamp = ros::Time::now();
            js_msg.name         = joint_names;
            js_msg.position     = pt.positions;
            js_msg.velocity     = pt.velocities;
            js_msg.effort       = { torques(0), torques(1), torques(2) };
            js_pub.publish(js_msg);

            // Feedback for next iteration
            output.pass_to_input(input);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
