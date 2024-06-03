#ifndef RM_SENTRY_FSM_HPP_
#define RM_SENTRY_FSM_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/cv_decision.hpp"  // 自定义数据类型

namespace rm_sentry_FSM
{

class GameStatusStateMachine
{
public:
    enum State
    {
        NOT_STARTED = 0,
        PREPARATION = 1,
        SELF_CHECK = 2,
        COUNTDOWN = 3,
        IN_PROGRESS = 4,
        SETTLEMENT = 5
    };

    GameStatusStateMachine();
    void handleInput(int game_progress);
    State getCurrentState() const;

private:
    State currentState;
    void handleInvalidInput(int game_progress);
};

class OutpostStateMachine
{
public:
    enum Color
    {
        RED,
        BLUE
    };

    enum HpState
    {
        OUTPOST_SAFE,
        OUTPOST_DANGER
    };

    OutpostStateMachine();
    void setColor(bool team_color);
    void updateHp(int red_outpost_hp, int blue_outpost_hp);
    bool isOutpostSafe() const;

private:
    Color color;
    int my_outpost_hp;
    HpState current_hp_state;
    void checkHp();
};

class RobotStatusStateMachine
{
public:
    enum HpState
    {
        SENTRY_SAFE,
        SENTRY_DANGER
    };

    enum BulletState
    {
        REMAINING,
        EMPTY
    };

    RobotStatusStateMachine();
    void updateStatus(int current_hp, int remaining_bullet);
    bool isSentrySafe() const;

private:
    int current_hp;
    int remaining_bullet;
    HpState current_hp_state;
    BulletState current_bullet_state;
    void checkHp();
    void checkBullet();
};

class DecisionStateMachine
{
public:
    enum DecisionState
    {
        INVALID = 0,
        ATTACK_POINT1 = 1,
        DEFEND_POINT1 = 2,
    };

    DecisionStateMachine(GameStatusStateMachine& gsm, OutpostStateMachine& osm, RobotStatusStateMachine& rssm);
    DecisionState determineDecisionState() const;
    void makeDecision();

private:
    GameStatusStateMachine& gsm_;
    OutpostStateMachine& osm_;
    RobotStatusStateMachine& rssm_;
};

class GameStatusNode : public rclcpp::Node
{
public:
    GameStatusNode();
    void setDecisionNode(std::shared_ptr<DecisionNode> decision_node);
    GameStatusStateMachine& getGameStateMachine();

private:
    void gameStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg);
    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr game_status_subscription_;
    GameStatusStateMachine gsm_;
    std::shared_ptr<DecisionNode> decision_node_;
};

class OutpostHpNode : public rclcpp::Node
{
public:
    OutpostHpNode(OutpostStateMachine& osm);
    void setDecisionNode(std::shared_ptr<DecisionNode> decision_node);

private:
    void robotHpCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg);
    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr robot_hp_subscription_;
    OutpostStateMachine& osm_;
    std::shared_ptr<DecisionNode> decision_node_;
};

class RobotStatusNode : public rclcpp::Node
{
public:
    RobotStatusNode(RobotStatusStateMachine& rssm);
    void setDecisionNode(std::shared_ptr<DecisionNode> decision_node);

private:
    void robotStatusCallback(const rm_decision_interfaces::msg::CvDecision::SharedPtr msg);
    rclcpp::Subscription<rm_decision_interfaces::msg::CvDecision>::SharedPtr robot_status_subscription_;
    RobotStatusStateMachine& rssm_;
    std::shared_ptr<DecisionNode> decision_node_;
};

class DecisionNode : public rclcpp::Node
{
public:
    DecisionNode(DecisionStateMachine& dsm);

private:
    void decisionCallback();
    DecisionStateMachine& dsm_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
};

}  // namespace rm_sentry_FSM

#endif // RM_SENTRY_FSM_HPP_
