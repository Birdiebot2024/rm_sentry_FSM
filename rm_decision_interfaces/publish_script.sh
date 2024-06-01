source install/setup.sh

ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/CvDecision "{
    game_progress: 1,
}" &

ros2 topic pub -r 1 /robot_hp rm_decision_interfaces/msg/CvDecision "{
    team_color: 1,
    red_outpost_hp: 100,
    blue_outpost_hp: 100,
}" &

wait
