source install/setup.sh

ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/CvDecision "{
    robot_id: 7,
    current_hp: 600,
    shooter_heat: 100,
    team_color: 0,
    is_attacked: 1,
    remaining_bullet: 200,
}" &

ros2 topic pub -r 10 /game_status rm_decision_interfaces/msg/CvDecision "{
    game_progress: 4,
    stage_remain_time: 5,
}" &

ros2 topic pub -r 10 /robot_hp rm_decision_interfaces/msg/CvDecision "{
    red_outpost_hp: 100,
    blue_outpost_hp: 35,
}" &

ros2 topic pub -r 10 /decision_num rm_decision_interfaces/msg/CvDecision "{
    decision_num: 1,
}" &

wait
