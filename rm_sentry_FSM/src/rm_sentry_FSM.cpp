#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "rm_decision_interfaces/msg/cv_decision.hpp"  // 自定义数据类型

namespace rm_sentry_FSM
{

class SentryStateMachine
{
public:
    enum State
    {
        NOT_STARTED = 0,  // 0:未开始比赛
        PREPARATION = 1,  // 1:准备阶段
        SELF_CHECK = 2,   // 2:十五秒裁判系统自检阶段
        COUNTDOWN = 3,    // 3:五秒倒计时
        IN_PROGRESS = 4,  // 4:比赛中
        SETTLEMENT = 5    // 5:比赛结算中
    };

    SentryStateMachine() : currentState(NOT_STARTED) {}

    void handleInput(int game_progress)
    {
        switch (currentState)
        {
            case NOT_STARTED:
                std::cout << "未开始比赛" << std::endl;
                handleInvalidInput(game_progress);
                break;
            case PREPARATION:
                std::cout << "准备阶段" << std::endl;
                handleInvalidInput(game_progress);
                break;
            case SELF_CHECK:
                std::cout << "十五秒裁判系统自检阶段" << std::endl;
                handleInvalidInput(game_progress);
                break;
            case COUNTDOWN:
                std::cout << "五秒倒计时" << std::endl;
                handleInvalidInput(game_progress);
                break;
            case IN_PROGRESS:
                std::cout << "比赛中" << std::endl;
                handleInvalidInput(game_progress);
                break;
            case SETTLEMENT:
                std::cout << "比赛结算中" << std::endl;
                handleInvalidInput(game_progress);
                break;
        }
    }

    void handleInvalidInput(int game_progress)
    {
        if (game_progress > 5 || !game_progress)
        {
            std::cout << "missing required input [game_status]..." << std::endl;
        }
    }

private:
    State currentState;
};

class GameStatusNode : public rclcpp::Node
{
public:
    GameStatusNode() : Node("game_status_node")
    {
        subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/game_status", 10, std::bind(&GameStatusNode::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)
    {
        sm_.handleInput(msg->game_progress);
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr subscription_;
    rm_sentry_FSM::SentryStateMachine sm_;
};

}  // namespace rm_sentry_FSM

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_sentry_FSM::GameStatusNode>());
    rclcpp::shutdown();
    return 0;
}
