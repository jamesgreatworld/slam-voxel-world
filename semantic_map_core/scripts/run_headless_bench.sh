#!/bin/bash
# 自研 C++ 在线节点 跑完整 house_sim_bag(rate 2.0=原始传感器速度)
# 测: 总墙钟、积分帧数/丢帧、每帧积分ms、DSG cycle ms、峰值RSS、DSG 规模、merge CRUD 统计
O=/mnt/hgfs/Shared/claude_jobs
source /opt/ros/jazzy/setup.bash
source /home/james/lightning_simple_ws/install/setup.bash
export ROS_DOMAIN_ID=45
pkill -9 -f run_slam_online; pkill -9 -f live_ros_stream; pkill -9 -f "bag play"
pkill -9 -f "gvd_live.p[y]"; pkill -9 -f smc_live_node; pkill -9 -f static_transform; sleep 2
CFG=/home/james/lightning_simple_ws/src/lightning/config/default_tg_house.yaml
rm -f $O/cpp_node_stats.txt $O/cpp_dsg.txt
: > /tmp/cf_rss.log

( for i in $(seq 1 400); do
    r=$(ps -eo rss,cmd | grep -E "smc_live_node" | grep -v grep | awk '{s+=$1} END{print s+0}')
    echo "$r" >> /tmp/cf_rss.log; sleep 0.5
  done ) & SAMP=$!

nohup ros2 run lightning run_slam_online --config=$CFG >/tmp/cf_slam.log 2>&1 & disown
sleep 6
nohup ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id base --child-frame-id cam_optical >/tmp/cf_tf.log 2>&1 & disown
nohup /home/james/semantic_map_core/build/smc_live_node >/tmp/cf_node.log 2>&1 & disown
sleep 4
T0=$(date +%s)
ros2 bag play --rate 2.0 /home/james/dataset/house_sim_bag >/tmp/cf_bag.log 2>&1
T1=$(date +%s)
sleep 12   # 收尾积分 + 最后一个 cycle
T2=$(date +%s)
kill $SAMP 2>/dev/null
# SIGINT 节点让它 dump stats
pkill -INT -f smc_live_node; sleep 3

PEAK=$(sort -n /tmp/cf_rss.log | tail -1)
{
echo "=== 自研 C++ 节点 完整 house_sim_bag(rate 2.0=原速)==="
echo "bag 播放墙钟: $((T1-T0))s   总(含收尾): $((T2-T0))s"
echo "峰值 RSS(smc_live_node): ${PEAK} KB = $(awk "BEGIN{printf \"%.0f\", ${PEAK:-0}/1024}") MB"
echo "--- 节点统计(cpp_node_stats.txt)---"
cat $O/cpp_node_stats.txt 2>/dev/null || echo "(无 stats 文件)"
echo "--- 节点日志尾(integrated/cycle)---"
grep -E "integrated=|cycle " /tmp/cf_node.log | tail -12
echo "--- 节点日志头(grid/caminfo/异常)---"
head -8 /tmp/cf_node.log
echo "--- DSG dump 头 12 行 ---"
head -12 $O/cpp_dsg.txt 2>/dev/null
} > $O/cpp_full.txt
pkill -9 -f run_slam_online; pkill -9 -f smc_live_node; pkill -9 -f static_transform
echo DONE
