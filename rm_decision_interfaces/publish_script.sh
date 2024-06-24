source install/setup.sh

ros2 topic pub -r 10 /cv_decision rm_decision_interfaces/msg/CvDecision "{
    game_progress: 1,      

    team_color: 0,
    red_outpost_hp: 400,
    blue_outpost_hp: 1500,

    current_hp: 400,
    remaining_bullet: 400,
}" &

wait
