#include <functional>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_kdl_package/action/execute_trajectory.hpp"


class TrajectoryActionClient : public rclcpp::Node
{
public:

    using ExecuteTrajectory =
        ros2_kdl_package::action::ExecuteTrajectory;

    using GoalHandleExecuteTrajectory =
        rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;


    TrajectoryActionClient()
    : Node("trajectory_action_client")
    {
        using namespace std::placeholders;

        client_ptr_ =
            rclcpp_action::create_client<ExecuteTrajectory>(
                this,
                "execute_trajectory");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(
                &TrajectoryActionClient::send_goal,
                this));
    }


    void send_goal()
    {
        using namespace std::placeholders;

        // Send the goal only once
        timer_->cancel();

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for trajectory action server...");

        if (!client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trajectory action server not available");

            rclcpp::shutdown();
            return;
        }


        ExecuteTrajectory::Goal goal_msg;

        goal_msg.start = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending trajectory execution goal");


        auto send_goal_options =
            rclcpp_action::Client<
                ExecuteTrajectory>::SendGoalOptions();


        send_goal_options.goal_response_callback =
            std::bind(
                &TrajectoryActionClient::goal_response_callback,
                this,
                _1);


        send_goal_options.feedback_callback =
            std::bind(
                &TrajectoryActionClient::feedback_callback,
                this,
                _1,
                _2);


        send_goal_options.result_callback =
            std::bind(
                &TrajectoryActionClient::result_callback,
                this,
                _1);


        client_ptr_->async_send_goal(
            goal_msg,
            send_goal_options);
    }


private:

    rclcpp_action::Client<
        ExecuteTrajectory>::SharedPtr client_ptr_;

    rclcpp::TimerBase::SharedPtr timer_;


    void goal_response_callback(
        const GoalHandleExecuteTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trajectory goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Trajectory goal accepted by server");
        }
    }


    void feedback_callback(
        GoalHandleExecuteTrajectory::SharedPtr,
        const std::shared_ptr<
            const ExecuteTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Position error: %.6f",
            feedback->position_error);
    }


    void result_callback(
        const GoalHandleExecuteTrajectory::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:

                if (result.result->success)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Trajectory executed successfully");
                }
                else
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Trajectory finished without success");
                }

                break;


            case rclcpp_action::ResultCode::ABORTED:

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Trajectory goal was aborted");

                break;


            case rclcpp_action::ResultCode::CANCELED:

                RCLCPP_WARN(
                    this->get_logger(),
                    "Trajectory goal was canceled");

                break;


            default:

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unknown action result");

                break;
        }


        rclcpp::shutdown();
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TrajectoryActionClient>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}