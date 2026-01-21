ros2 run lsa_nav_controller lsa_nav_controller \\
    --ros-args -r cmd_vel/lsa_nav:=/cmd_vel \\
    --params-file $(ros2 pkg prefix --share lsa_nav_controller)/param/param.yaml
