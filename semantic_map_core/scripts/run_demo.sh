#!/bin/bash
# 干净全套演示: 每个进程都重启, 删档重建, 完整播包(rate 2)
export DISPLAY=:0 XDG_RUNTIME_DIR=/run/user/1000
export XAUTHORITY=$(ps -o args= -C Xwayland | grep -oE '\-auth [^ ]+' | awk '{print $2}')
unset WAYLAND_DISPLAY
source /opt/ros/jazzy/setup.bash
source /home/james/lightning_simple_ws/install/setup.bash
export ROS_DOMAIN_ID=45
pkill -9 -f run_slam_online; pkill -9 -f "bag play"; pkill -9 -f smc_live_node
pkill -9 -f static_transform; pkill -9 -f "rvi[z]2"; pkill -9 -f "path_pub.p[y]"
pkill -9 -f "livox2pc2.p[y]"; sleep 2
rm -rf /home/james/semantic_map_core/maps
CFG=/home/james/lightning_simple_ws/src/lightning/config/default_tg_house.yaml
nohup rviz2 -d /home/james/live.rviz >/tmp/fn_rviz.log 2>&1 & disown
sleep 5
nohup ros2 run lightning run_slam_online --config=$CFG >/tmp/fn_slam.log 2>&1 & disown
sleep 5
nohup ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id base --child-frame-id cam_optical >/tmp/fn_tf.log 2>&1 & disown
nohup python3 /home/james/path_pub.py >/tmp/fn_path.log 2>&1 & disown
nohup python3 /home/james/livox2pc2.py >/tmp/fn_pc2.log 2>&1 & disown
nohup /home/james/semantic_map_core/build/smc_live_node >/tmp/fn_node.log 2>&1 & disown
sleep 3
nohup ros2 bag play --rate 2.0 /home/james/dataset/house_sim_bag >/tmp/fn_bag.log 2>&1 & disown
echo LAUNCHED
