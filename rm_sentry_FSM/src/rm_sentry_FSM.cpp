#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "rm_decision_interfaces/msg/cv_decision.hpp"  // 自定义数据类型

namespace rm_sentry_FSM
{

class GameStatusStateMachine  // 比赛进程状态机
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

    GameStatusStateMachine() : currentState(NOT_STARTED) {}  // 将状态机的初始状态设置为 0:未开始比赛

    void handleInput(int game_progress)
    {
        switch (currentState)  // 检测比赛状态的状态机
        {
            case NOT_STARTED:
                std::cout << "未开始比赛" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 1) currentState = PREPARATION;
                break;
            case PREPARATION:
                std::cout << "准备阶段" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 2) currentState = SELF_CHECK; 
                break;
            case SELF_CHECK:
                std::cout << "十五秒裁判系统自检阶段" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 3) currentState = COUNTDOWN; 
                break;
            case COUNTDOWN:
                std::cout << "五秒倒计时" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 4) currentState = IN_PROGRESS; 
                break;
            case IN_PROGRESS:
                std::cout << "比赛中" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 5) currentState = SETTLEMENT;
                break;
            case SETTLEMENT:
                std::cout << "比赛结算中" << std::endl;
                handleInvalidInput(game_progress);
                if (game_progress == 0) currentState = NOT_STARTED;
                break;
        }
    }

    State getCurrentState() const  // 获取当前比赛状态(前哨站判断读取比赛)
    {
        return currentState;
    }

    void handleInvalidInput(int game_progress)  // 检测比赛状态非法输入的函数
    {
        if (game_progress < 0 || game_progress > 5)
        {
            std::cout << "missing required input [game_status]..." << std::endl;
        }
    }

private:
    State currentState;  // 当前状态
};

class OutpostStateMachine  // 前哨站血量状态机
{
public:
    enum Color
    {
        RED,  // 0:红色
        BLUE  // 1:蓝色
    };

    OutpostStateMachine() : color(RED), my_outpost_hp(0) {}  // 将状态机的初始状态设置为 红色,前哨站血量:0

    void setColor(bool team_color)  // 检测己方颜色
    {
        if (team_color)
        {
            this->color = BLUE;
        }
        else 
        {
            this->color = RED;
        }
    }

    void updateHp(int red_outpost_hp, int blue_outpost_hp)  // 更新己方前哨站血量
    {
        if (color == RED)
        {
            my_outpost_hp = red_outpost_hp;
        } 
        else if (color == BLUE)
        {
            my_outpost_hp = blue_outpost_hp;
        }
        checkHp();
    }

    void checkHp()  // 检测是否到己方前哨站斩杀线
    {
        if(my_outpost_hp > 500)
        {
            std::cout << "己方前哨站处于安全血量" << std::endl;
        }
        else
        {
            std::cout << "己方前哨站处于斩杀线" << std::endl;
        }
    }

private:
    Color color;  // 己方颜色
    int my_outpost_hp;  // 己方前哨站血量
};

class RobotStatusStateMachine 
{
public:
    RobotStatusStateMachine() : current_hp(0), remaining_bullet(0) {}

    void updateStatus(int current_hp, int remaining_bullet)
    {
        this->current_hp = current_hp;
        this->remaining_bullet = remaining_bullet;
        checkHp();
        checkBullet();
    }

    void checkHp()
    {
        if (current_hp <= 200)
        {
            std::cout << "哨兵血量达到危险值" << std::endl;
        }
    }

    void checkBullet()
    {
        if (remaining_bullet == 0)
        {
            std::cout << "剩余发弹量为0" << std::endl;
        }
    }

private:
    int current_hp;
    int remaining_bullet;
};

class GameStatusNode : public rclcpp::Node  // 比赛进程节点
{
public:
    GameStatusNode() : Node("game_status_node")  // 订阅比赛进程
    {
        game_status_subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/cv_decision", 10, std::bind(&GameStatusNode::gameStatusCallback, this, std::placeholders::_1)
        );
    }

    GameStatusStateMachine& getGameStateMachine()
    {
        return gsm_;
    }

private:
    void gameStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)  // 比赛进程的回调函数
    {
        if (msg->game_progress < 0 || msg->game_progress > 5)
        {
            std::cout << "missing required input [game_status]..." << std::endl;
        }
        else
        {
            gsm_.handleInput(msg->game_progress);
        }
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr game_status_subscription_;
    rm_sentry_FSM::GameStatusStateMachine gsm_;
};

class OutpostHpNode : public rclcpp::Node  // 前哨站状态节点
{
public:
    OutpostHpNode(OutpostStateMachine& osm) : Node("robot_hp_node"), osm_(osm)  // 订阅比赛状态
    {
        robot_hp_subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/cv_decision", 10, std::bind(&OutpostHpNode::robotHpCallback, this, std::placeholders::_1)
        );
    }

private:
    void robotHpCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)  // 前哨站状态的回调函数
    {
        std::cout << "Red Outpost HP:" << msg->red_outpost_hp << std::endl;
        std::cout << "Blue Outpost HP:" << msg->blue_outpost_hp << std::endl;

        osm_.setColor(msg->team_color);
        osm_.updateHp(msg->red_outpost_hp, msg->blue_outpost_hp);
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr robot_hp_subscription_;
    OutpostStateMachine& osm_;
};

class RobotStatusNode : public rclcpp::Node
{
public:
    RobotStatusNode(RobotStatusStateMachine& rssm) : Node("robot_status_node"), rssm_(rssm)
    {
        robot_status_subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/cv_decision", 10, std::bind(&RobotStatusNode::robotStatusCallback, this, std::placeholders::_1)
        );
    }

private:
    void robotStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)
    {
        std::cout << "current_hp:" << msg->current_hp << std::endl;
        std::cout << "remaining_bullet" << msg->remaining_bullet << std::endl;

        rssm_.updateStatus(msg->current_hp, msg->remaining_bullet);
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr robot_status_subscription_;
    RobotStatusStateMachine& rssm_;
};

}  // namespace rm_sentry_FSM

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto game_status_node = std::make_shared<rm_sentry_FSM::GameStatusNode>();

    rm_sentry_FSM::OutpostStateMachine outpost_sm;
    auto robot_hp_node = std::make_shared<rm_sentry_FSM::OutpostHpNode>(outpost_sm);

    rm_sentry_FSM::RobotStatusStateMachine robot_status_sm;
    auto robot_status_node = std::make_shared<rm_sentry_FSM::RobotStatusNode>(robot_status_sm);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(game_status_node);
    executor.add_node(robot_hp_node);
    executor.add_node(robot_status_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
