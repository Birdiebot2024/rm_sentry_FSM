source install/setup.sh

ros2 topic pub -r 10 /game_status rm_decision_interfaces/msg/CvDecision "{
    game_progress: 0,
}" &

wait
