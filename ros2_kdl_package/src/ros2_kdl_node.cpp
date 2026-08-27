// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <algorithm>
#include <csignal>
#include <thread>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using ExecuteTrajectory =
    ros2_kdl_package::action::ExecuteTrajectory;

using GoalHandleExecuteTrajectory =
    rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;


volatile std::sig_atomic_t shutdown_requested = 0;

void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        shutdown_requested = 1;
    }
}    

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node") 
        // node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // default to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            declare_parameter("ctrl", "velocity_ctrl");
            get_parameter("ctrl", ctrl_);

            
            RCLCPP_INFO(get_logger(), "Current controller is: '%s'", ctrl_.c_str());



            if (!(ctrl_ == "velocity_ctrl" ||
                ctrl_ == "velocity_ctrl_null" ||
                ctrl_ == "vision"))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid ctrl. Use 'velocity_ctrl', "
                    "'velocity_ctrl_null', or 'vision'.");

                return;
            }


            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }


            // =========================================================
            // Declare and get trajectory parameters
            // =========================================================

            declare_parameter("traj_duration", 1.5);
            declare_parameter("acc_duration", 0.5);
            declare_parameter("total_time", 1.5);
            declare_parameter("trajectory_len", 150);
            declare_parameter("Kp", 5.0);
            declare_parameter("lambda", 0.1);

            declare_parameter("end_position_x", 0.5);
            declare_parameter("end_position_y", 0.0);
            declare_parameter("end_position_z", 0.5);

            get_parameter("traj_duration", traj_duration_);
            get_parameter("acc_duration", acc_duration_);
            get_parameter("total_time", total_time_);
            get_parameter("trajectory_len", trajectory_len_);
            get_parameter("Kp", Kp_);
            get_parameter("lambda", lambda_);

            get_parameter("end_position_x", end_position_x_);
            get_parameter("end_position_y", end_position_y_);
            get_parameter("end_position_z", end_position_z_);

            // Print parameters

            RCLCPP_INFO(get_logger(), "traj_duration: %.2f", traj_duration_);
            RCLCPP_INFO(get_logger(), "acc_duration: %.2f", acc_duration_);
            RCLCPP_INFO(get_logger(), "total_time: %.2f", total_time_);
            RCLCPP_INFO(get_logger(), "trajectory_len: %d", trajectory_len_);
            RCLCPP_INFO(get_logger(), "Kp: %.2f", Kp_);
            RCLCPP_INFO(get_logger(), "lambda: %.4f", lambda_);

            RCLCPP_INFO(get_logger(),
                        "end_position: [%.2f, %.2f, %.2f]",
                        end_position_x_,
                        end_position_y_,
                        end_position_z_);


            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            marker_pose_available_ = false;
            trajectory_active_ = false;

            // retrieve robot_description param
            // auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Subscriber to ArUco marker pose
            arucoPoseSubscriber_ =this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));    

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                // rclcpp::spin_some(node_handle_);
                rclcpp::spin_some(this->get_node_base_interface());
            }

            

            // ---------------------------------------------------------
            // Update KDL robot with current joint state
            // ---------------------------------------------------------
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


            // ---------------------------------------------------------
            // Select controlled end-effector
            // ---------------------------------------------------------
            //
            // The KDL chain tip is "tool0".
            //
            // URDF geometry:
            //
            // link_7 -> tool0       = 0.154 m along local Z
            // link_7 -> camera_link = 0.180 m along local Z
            //
            // Therefore:
            //
            // tool0 -> camera origin = 0.026 m along local Z
            //
            // Gazebo optical sensor orientation relative to camera_link:
            //
            // roll  =  0
            // pitch = -0.922
            // yaw   = -0.468
            //
            // For vision control we therefore make the KDL end-effector
            // coincide with the optical camera frame.
            //
            if (ctrl_ == "vision")
            {
                // ---------------------------------------------------------
                // Fixed transform from tool0 to the camera mounting frame
                // ---------------------------------------------------------
                //
                // Translation:
                //   link_7 -> tool0       = 0.154 m
                //   link_7 -> camera      = 0.180 m
                //
                // Therefore:
                //   tool0 -> camera origin = 0.026 m
                //
                // Existing camera mounting rotation:
                //   roll  =  0
                //   pitch = -0.922
                //   yaw   = -0.468
                // ---------------------------------------------------------

                KDL::Rotation R_mount =
                    KDL::Rotation::RPY(
                        0.0,
                        -0.922,
                        -0.468);


                // ---------------------------------------------------------
                // Fixed camera-link -> optical-frame rotation
                //
                // Optical convention:
                //   x = right
                //   y = down
                //   z = forward
                //
                // This is the transformation verified by our
                // RC candidate test.
                // ---------------------------------------------------------

                KDL::Rotation R_link_opt(
                    0.0,  0.0,  1.0,
                    -1.0,  0.0,  0.0,
                    0.0, -1.0,  0.0);


                // ---------------------------------------------------------
                // Complete tool0 -> optical-camera transform
                //
                // R_tool_opt = R_mount * R_link_opt
                // ---------------------------------------------------------

                KDL::Frame tool0_T_camera_optical(
                    R_mount * R_link_opt,
                    KDL::Vector(
                        0.0,
                        0.0,
                        0.026));


                // Tell KDL that the controlled end-effector is now
                // the optical camera frame.
                robot_->addEE(tool0_T_camera_optical);


                RCLCPP_INFO(
                    this->get_logger(),
                    "Vision mode: KDL end-effector set to eye-in-hand optical camera.");
            }
            else
            {
                KDL::Frame f_T_ee =
                    KDL::Frame::Identity();

                robot_->addEE(f_T_ee);
            }
           


            // Recompute pose and Jacobian for the selected end-effector
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));



            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            

            // Initialize controller
            // KDLController controller_(*robot_);
            controller_ = std::make_shared<KDLController>(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));


            // EE's trajectory end position from ROS2 parameters
            Eigen::Vector3d end_position;
            end_position << end_position_x_, end_position_y_, end_position_z_;

            
            // Plan trajectory
            double traj_radius = 0.15;

            // Retrieve the first trajectory point
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner( traj_duration_, acc_duration_, init_position, end_position); 
                
                // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }
            
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            // RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            // Create action server
            using namespace std::placeholders;

            action_server_ =
                rclcpp_action::create_server<ExecuteTrajectory>(
                    this,
                    "execute_trajectory",
                    std::bind(
                        &Iiwa_pub_sub::handle_goal,
                        this,
                        _1,
                        _2),
                    std::bind(
                        &Iiwa_pub_sub::handle_cancel,
                        this,
                        _1),
                    std::bind(
                        &Iiwa_pub_sub::handle_accepted,
                        this,
                        _1));

            RCLCPP_INFO(
                this->get_logger(),
                "Trajectory action server ready. Waiting for a goal...");

            }
            // =========================================================
            // SAFE STOP
            // =========================================================
            void stop_robot()
            {
                // Only needed for velocity control
                if (cmd_interface_ != "velocity")
                {
                    return;
                }

                // ---------------------------------------------------------
                // Stop the periodic command callback FIRST
                // ---------------------------------------------------------
                if (timer_)
                {
                    timer_->cancel();

                    RCLCPP_WARN(
                        this->get_logger(),
                        "Command timer cancelled.");
                }

                // Make sure publisher exists
                if (!cmdPublisher_)
                {
                    return;
                }

                // ---------------------------------------------------------
                // Reset all internal velocity commands
                // ---------------------------------------------------------
                desired_commands_.assign(7, 0.0);

                joint_velocities_cmd_.data.setZero();

                // ---------------------------------------------------------
                // Create explicit zero command
                // ---------------------------------------------------------
                std_msgs::msg::Float64MultiArray stop_msg;
                stop_msg.data.assign(7, 0.0);

                RCLCPP_WARN(
                    this->get_logger(),
                    "Stopping robot: sending ZERO joint velocities.");

                // Publish zero several times
                for (int i = 0; i < 10; ++i)
                {
                    cmdPublisher_->publish(stop_msg);

                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(20));
                }
            }



    private:

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const ExecuteTrajectory::Goal> goal)
        {
            (void)uuid;

            RCLCPP_INFO(
                this->get_logger(),
                "Received trajectory execution goal");

            // Only accept a goal that explicitly requests execution
            if (!goal->start)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Goal rejected because start=false");

                return rclcpp_action::GoalResponse::REJECT;
            }

            // Do not start another trajectory while one is already running
            if (trajectory_active_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Goal rejected because a trajectory is already running");

                return rclcpp_action::GoalResponse::REJECT;
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }


        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
        {
            (void)goal_handle;

            RCLCPP_INFO(
                this->get_logger(),
                "Received request to cancel trajectory");

            return rclcpp_action::CancelResponse::ACCEPT;
        }


        void handle_accepted(
            const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Trajectory goal accepted. Starting execution...");

            active_goal_handle_ = goal_handle;

            // Restart trajectory from its beginning
            iteration_ = 0;
            t_ = 0.0;

            trajectory_active_ = true;
        }



        void cmd_publisher(){

            //
            // ---------------------------------------------------------
            // Shutdown protection
            //
            // If Ctrl+C has been pressed, never calculate or publish
            // another motion command.
            // ---------------------------------------------------------
            if (shutdown_requested)
            {
                return;
            }


            // =========================================================
            // Q2(b): continuous vision-control branch
            // =========================================================
            //
            // Vision control does NOT depend on the Q1(c) trajectory
            // action server and does NOT stop after total_time_.
            //

            if (ctrl_ == "vision")
            {
                // -----------------------------------------------------
                // No marker has been received yet -> send zero velocity
                // -----------------------------------------------------
                if (!marker_pose_available_)
                {
                    std::fill(
                        desired_commands_.begin(),
                        desired_commands_.end(),
                        0.0);

                    std_msgs::msg::Float64MultiArray stop_msg;
                    stop_msg.data = desired_commands_;
                    cmdPublisher_->publish(stop_msg);

                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "Vision controller waiting for ArUco marker pose. Sending zero velocity.");

                    return;
                }

                // -----------------------------------------------------
                // Check whether the latest marker measurement is stale
                // -----------------------------------------------------
                // double marker_age =
                //     (this->now() - last_marker_time_).seconds();

                double marker_age =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() -
                        last_marker_time_).count();
                    

                if (marker_age > 0.5)
                {
                    std::fill(
                        desired_commands_.begin(),
                        desired_commands_.end(),
                        0.0);

                    std_msgs::msg::Float64MultiArray stop_msg;
                    stop_msg.data = desired_commands_;
                    cmdPublisher_->publish(stop_msg);

                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "ArUco marker lost/stale (age %.3f s). Sending zero velocity.",
                        marker_age);

                    return;
                }

                // -----------------------------------------------------
                // Update KDL using latest robot state
                // -----------------------------------------------------
                robot_->update(
                    toStdVector(joint_positions_.data),
                    toStdVector(joint_velocities_.data));

                // Current optical-camera pose
                KDL::Frame camera_frame =
                    robot_->getEEFrame();

                // Camera orientation in world frame
                Eigen::Matrix3d Rc =
                    toEigen(camera_frame.M);
                

                // Camera Jacobian, 6 x 7
                Eigen::MatrixXd Jc =
                    robot_->getEEJacobian().data;

                // Marker distance ||cPo||
                double marker_distance =
                    marker_position_optical_.norm();


                // -----------------------------------------------------
                // Calculate vision controller output
                // -----------------------------------------------------
                joint_velocities_cmd_.data =
                    controller_->vision_ctrl(
                        s_,
                        s_desired_,
                        marker_distance,
                        Rc,
                        Jc,
                        Kp_,
                        lambda_);
                


                // Safety saturation for vision-control joint velocities
                const double max_vision_velocity = 0.05;   // rad/s

                for (long int i = 0;
                    i < joint_velocities_cmd_.data.size();
                    ++i)
                {
                    double qdot_i = joint_velocities_cmd_(i);

                    qdot_i = std::clamp(
                        qdot_i,
                        -max_vision_velocity,
                        max_vision_velocity);

                    desired_commands_[i] = qdot_i;
                }




                // Publish the complete visual-servo velocity command
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                
                
                return;
            }

            // =========================================================
            // Existing Q1 trajectory/action behaviour
            // =========================================================

            // Do nothing until an action client starts the trajectory
            if (!trajectory_active_)
            {
                return;
            }

            // Check whether the action client requested cancellation
            if (active_goal_handle_ &&
                active_goal_handle_->is_canceling())
            {
                // Stop velocity commands
                if (cmd_interface_ == "velocity")
                {
                    for (long int i = 0;
                        i < joint_velocities_.data.size();
                        ++i)
                    {
                        desired_commands_[i] = 0.0;
                    }
                }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                auto result =
                    std::make_shared<ExecuteTrajectory::Result>();

                result->success = false;

                active_goal_handle_->canceled(result);

                trajectory_active_ = false;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Trajectory canceled");

                return;
            }

            iteration_ = iteration_ + 1;

            // Define trajectory timing using ROS2 parameters
            int loop_rate = trajectory_len_ / total_time_;
            double dt = 1.0 / loop_rate;

            t_ += dt;

            if (t_ < total_time_){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }


                // Update KDL robot with the latest measured joint state
                robot_->update(
                    toStdVector(joint_positions_.data),
                    toStdVector(joint_velocities_.data));


    
                
                    
                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                // Publish Cartesian position error as action feedback
                if (active_goal_handle_)
                {
                    auto feedback =
                        std::make_shared<ExecuteTrajectory::Feedback>();

                    feedback->position_error = error.norm();

                    active_goal_handle_->publish_feedback(feedback);
                }

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                


                else if(cmd_interface_ == "velocity")
                {
                    if(ctrl_ == "velocity_ctrl")
                    {
                        // Standard differential IK
                        Vector6d cartvel;
                        cartvel << p_.vel + Kp_*error, o_error;

                        joint_velocities_cmd_.data =
                            pseudoinverse(
                                robot_->getEEJacobian().data) * cartvel;
                    }
                    else if(ctrl_ == "velocity_ctrl_null")
                    {
                        // Differential IK with joint-limit avoidance
                        joint_velocities_cmd_.data =
                            controller_->velocity_ctrl_null(
                                error,
                                Kp_,
                                lambda_);
                    }
                }


                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time_);
                }

                

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                // Complete the action
                if (active_goal_handle_)
                {
                    auto result =
                        std::make_shared<ExecuteTrajectory::Result>();

                    result->success = true;

                    active_goal_handle_->succeed(result);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Action completed successfully");
                }

                trajectory_active_ = false;
                active_goal_handle_.reset();
                

            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){


            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // ArUco marker pose callback


        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped & msg)
        {
            marker_pose_ = msg;
            marker_pose_available_ = true;
            // last_marker_time_ = this->now();
            last_marker_time_ =
                std::chrono::steady_clock::now();

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     1000,
            //     "Fresh ArUco pose received.");    

            // ---------------------------------------------------------
            // Marker position as reported by aruco_ros
            // This is expressed in the eye-in-hand optical/image frame.
            // ---------------------------------------------------------
            marker_position_optical_ <<
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z;
 


            // Desired viewing direction: camera optical +Z axis
            s_desired_ << 0.0, 0.0, 1.0;

            // Compute the current unit viewing direction toward the marker
            if (marker_position_optical_.norm() > 1e-6)
            {
                s_ = marker_position_optical_.normalized();
            }
            else
            {
                s_.setZero();
                return;
            }


                
        }



        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoPoseSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        // rclcpp::Node::SharedPtr node_handle_;

        // Action server
        rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;

        std::shared_ptr<GoalHandleExecuteTrajectory>
            active_goal_handle_;

        bool trajectory_active_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        bool marker_pose_available_;
        geometry_msgs::msg::PoseStamped marker_pose_;
        // rclcpp::Time last_marker_time_;
        std::chrono::steady_clock::time_point last_marker_time_;
        Eigen::Vector3d marker_position_optical_;
        
        Eigen::Vector3d s_;
        Eigen::Vector3d s_desired_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string ctrl_;

        // ROS2 parameters
        double traj_duration_;
        double acc_duration_;
        double total_time_;
        int trajectory_len_;
        double Kp_;
        double lambda_;

        double end_position_x_;
        double end_position_y_;
        double end_position_z_;
        

        KDL::Frame init_cart_pose_;
};

 
// int main( int argc, char** argv )
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
//     rclcpp::shutdown();
//     return 0;
// }


int main(int argc, char** argv)
{
    // Disable ROS2's default SIGINT handler.
    // We handle Ctrl+C ourselves so that we can send
    // zero velocity BEFORE shutting ROS2 down.
    rclcpp::init(
        argc,
        argv,
        rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None);

    // Install our custom handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node =
        std::make_shared<Iiwa_pub_sub>();


    // Keep ROS running until Ctrl+C / SIGTERM is requested
    while (rclcpp::ok() && !shutdown_requested)
    {
        rclcpp::spin_some(node);

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));
    }


    // =====================================================
    // IMPORTANT:
    // ROS is still alive here.
    //
    // Send ZERO velocity BEFORE rclcpp::shutdown().
    // =====================================================
    node->stop_robot();


    // Allow the final DDS message a short time to leave
    // the publisher before destroying the ROS context.
    rclcpp::spin_some(node);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));


    // Now it is safe to shut ROS down
    rclcpp::shutdown();

    return 0;
}
