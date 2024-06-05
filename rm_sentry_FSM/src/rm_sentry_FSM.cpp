#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

    State getCurrentState() const  // 获取当前比赛状态
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

    enum HpState
    {
        OUTPOST_SAFE,   // 前哨站处于安全血量
        OUTPOST_DANGER  // 前哨站处于斩杀线
    };

    OutpostStateMachine() : color(RED), my_outpost_hp(1500), current_hp_state(OUTPOST_SAFE) {}  // 将状态机的初始状态设置为 红色,前哨站血量:1500,当前血量状态(SAFE)

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

    bool isOutpostSafe() const
    {
        return current_hp_state == OUTPOST_SAFE;
    }

    Color color;  // 己方颜色
    int my_outpost_hp;  // 己方前哨站血量
    HpState current_hp_state;  // 当前血量状态

    void checkHp()  // 检测是否到己方前哨站斩杀线
    {
        if(my_outpost_hp > 500)
        {
            if (current_hp_state !=  OUTPOST_SAFE)
            {
                current_hp_state =  OUTPOST_SAFE;
                std::cout << "己方前哨站处于安全血量" << std::endl;
            }
            
        }
        else
        {
            if (current_hp_state != OUTPOST_DANGER)
            {
                current_hp_state = OUTPOST_DANGER;
                std::cout << "己方前哨站处于斩杀线" << std::endl;
            }
            
        }
    }
};

class RobotStatusStateMachine  // 哨兵自身状态状态机
{
public:
    enum HpState
    {
        SENTRY_SAFE,  // 哨兵血量安全
        SENTRY_DANGER // 哨兵血量危险
    };

    enum BulletState
    {
        REMAINING,  // 仍有剩余发弹量
        EMPTY       // 没有剩余发弹量
    };

    RobotStatusStateMachine() : current_hp(400), remaining_bullet(400), current_hp_state(SENTRY_SAFE), current_bullet_state(REMAINING) {}

    void updateStatus(int current_hp, int remaining_bullet)
    {
        this->current_hp = current_hp;
        this->remaining_bullet = remaining_bullet;
        checkHp();
        checkBullet();
    }

    bool isSentrySafe() const
    {
        return current_hp_state == SENTRY_SAFE;
    }

    int current_hp;  // 哨兵当前生命值
    int remaining_bullet;  // 哨兵剩余发弹量
    HpState current_hp_state;  // 哨兵当前生命值状态
    BulletState current_bullet_state; // 哨兵当前剩余发弹量状态

    void checkHp()
    {
        if(current_hp > 200)
        {
            if (current_hp_state !=  SENTRY_SAFE)
            {
                current_hp_state =  SENTRY_SAFE;
                std::cout << "哨兵处于安全血量" << std::endl;
            }
            
        }
        else
        {
            if (current_hp_state != SENTRY_DANGER)
            {
                current_hp_state = SENTRY_DANGER;
                std::cout << "哨兵处于危险血量" << std::endl;
            }
            
        }
    }

    void checkBullet()
    {
        if(remaining_bullet > 0)
        {
            if (current_bullet_state !=  REMAINING)
            {
                current_bullet_state =  EMPTY;
            }
            
        }
        else
        {
            if (current_bullet_state != EMPTY)
            {
                current_bullet_state = REMAINING;
                std::cout << "哨兵剩余发弹量为0" << std::endl;
            }
            
        }
    }
};

class DecisionStateMachine
{
public:

    enum DecisionState
    {
        INVALID = 0,
        ATTACK_POINT1 = 1,
        DEFEND_POINT1 = 2,
        SUPPLY_POINT = 3,
    };

    DecisionStateMachine(GameStatusStateMachine& gsm, OutpostStateMachine& osm, RobotStatusStateMachine& rssm, rclcpp::Node::SharedPtr node)
        : gsm_(gsm), osm_(osm), rssm_(rssm)
    {
    }

    DecisionState determineDecisionState() const
    {
        // std::cout << "GameStatusStateMachine Address:" << &gsm_ <<std::endl;
        // std::cout << "OutpostStateMachine Address:" << &osm_ <<std::endl;
        // std::cout << "RobotStatusStateMachine Address:" << &rssm_ <<std::endl;

        if (gsm_.getCurrentState() == GameStatusStateMachine::IN_PROGRESS &&
            osm_.isOutpostSafe() && rssm_.isSentrySafe())
        {
            std::cout << "ATTACK_POINT1" << std::endl;
            return ATTACK_POINT1;
        }
        else if (gsm_.getCurrentState() == GameStatusStateMachine::IN_PROGRESS &&
            !osm_.isOutpostSafe() && rssm_.isSentrySafe())
        {
            return DEFEND_POINT1;
        }
        else if (gsm_.getCurrentState() == GameStatusStateMachine::IN_PROGRESS &&
            osm_.isOutpostSafe() && !rssm_.isSentrySafe())
        {
            return SUPPLY_POINT;
        }
        else
        {
            return INVALID;
        }
    }

    void makeDecision()
    {
        DecisionState decision_state = determineDecisionState();

        switch (decision_state)
        {
            case ATTACK_POINT1:
                std::cout << "正在前往预设进攻点位1" << std::endl;
                break;

            case DEFEND_POINT1:
                std::cout << "正在前往预设防守点位1" << std::endl;
                break;

            case SUPPLY_POINT:
                std::cout << "正在前往预设补给点位" << std::endl;
                break;

            case INVALID:
                std::cout << "无效的决策状态" << std::endl;
                break;
        }   
    }

    GameStatusStateMachine& gsm_;
    OutpostStateMachine& osm_;
    RobotStatusStateMachine& rssm_;

};


class GameStatusNode : public rclcpp::Node  // 比赛进程节点
{
public:
    GameStatusNode(GameStatusStateMachine& gsm) : Node("game_status_node"), gsm_(gsm)  // 订阅比赛进程
    {
        game_status_subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/cv_decision", 10, std::bind(&GameStatusNode::gameStatusCallback, this, std::placeholders::_1)
        );
    }

    GameStatusStateMachine& getGameStateMachine()
    {
        return gsm_;
    }

    void gameStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)  // 比赛进程的回调函数
    {
        // std::cout << "GameStatusStateMachine in callback Address:" << &gsm_ << std::endl;

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
    GameStatusStateMachine& gsm_;
};

class OutpostHpNode : public rclcpp::Node  // 前哨站状态节点
{
public:
    OutpostHpNode(OutpostStateMachine& osm) : Node("outpost_hp_node"), osm_(osm)  // 订阅比赛状态
    {
        outpost_hp_subscription_ = this->create_subscription<rm_decision_interfaces::msg::CvDecision>(
            "/cv_decision", 10, std::bind(&OutpostHpNode::OutpostHpCallback, this, std::placeholders::_1)
        );
    }

    OutpostStateMachine& getOutpostStateMachine()
    {
        return osm_;
    }

    void OutpostHpCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)  // 前哨站状态的回调函数
    {
        // std::cout << "Red Outpost HP:" << msg->red_outpost_hp << std::endl;
        // std::cout << "Blue Outpost HP:" << msg->blue_outpost_hp << std::endl;
        // std::cout << "OutpostStateMachine in callback Address:" << &osm_ << std::endl;

        osm_.setColor(msg->team_color);
        osm_.updateHp(msg->red_outpost_hp, msg->blue_outpost_hp);
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr outpost_hp_subscription_;
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

    RobotStatusStateMachine& getRobotStatusStateMachine()
    {
        return rssm_;
    }

    void robotStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg)
    {
        // std::cout << "current_hp:" << msg->current_hp << std::endl;
        // std::cout << "remaining_bullet:" << msg->remaining_bullet << std::endl;
        // std::cout << "RobotStatusStateMachine in callback Address:" << &rssm_ << std::endl;

        rssm_.updateStatus(msg->current_hp, msg->remaining_bullet);
    }

    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr robot_status_subscription_;
    RobotStatusStateMachine& rssm_;
};

class DecisionNode : public rclcpp::Node
{
public:
    DecisionNode(DecisionStateMachine& dsm) : Node("decision_node"), dsm_(dsm)
    {
        decision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DecisionNode::decisionCallback, this)
        );
    }

    void decisionCallback()
    {
        dsm_.makeDecision();
    }

    DecisionStateMachine& dsm_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
};

}  // namespace rm_sentry_FSM

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // 创建 ROS 节点
    auto node = rclcpp::Node::make_shared("decision_navigation_node");

    rm_sentry_FSM::GameStatusStateMachine game_status_sm;
    // std::cout << "Main GameStatusStateMachine address:" << &game_status_sm << std::endl;
    auto game_status_node = std::make_shared<rm_sentry_FSM::GameStatusNode>(game_status_sm);

    rm_sentry_FSM::OutpostStateMachine outpost_sm;
    // std::cout << "Main OutpostStateMachine address:" << &outpost_sm << std::endl;
    auto robot_hp_node = std::make_shared<rm_sentry_FSM::OutpostHpNode>(outpost_sm);

    rm_sentry_FSM::RobotStatusStateMachine robot_status_sm;
    // std::cout << "Main RobotStatusStateMachine address:" << &robot_status_sm << std::endl;
    auto robot_status_node = std::make_shared<rm_sentry_FSM::RobotStatusNode>(robot_status_sm);

    rm_sentry_FSM::DecisionStateMachine decision_sm(game_status_sm, outpost_sm, robot_status_sm, node);
    auto decision_node = std::make_shared<rm_sentry_FSM::DecisionNode>(decision_sm);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(game_status_node);
    executor.add_node(robot_hp_node);
    executor.add_node(robot_status_node);
    executor.add_node(decision_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
