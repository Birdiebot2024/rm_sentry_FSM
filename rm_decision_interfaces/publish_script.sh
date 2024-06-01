source install/setup.sh

ros2 topic pub -r 1 /cv_decision rm_decision_interfaces/msg/CvDecision "{
    game_progress: 1,      

    team_color: 1,
    red_outpost_hp: 100,
    blue_outpost_hp: 100,

    current_hp: 200,
    remaining_bullet: 0,
}" &

wait
