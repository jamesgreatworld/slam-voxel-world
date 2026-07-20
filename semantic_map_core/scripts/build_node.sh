#!/bin/bash
# 构建 rclcpp 在线节点(source ROS 后 cmake -DBUILD_ROS_NODE=ON)
O=/mnt/hgfs/Shared/claude_jobs
DST=/home/james/semantic_map_core
source /opt/ros/jazzy/setup.bash
{
rsync -a --delete $O/smc/ $DST/
mkdir -p $DST/build
cd $DST/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS_NODE=ON 2>&1 | grep -iE "error|warn" | head
make smc_live_node -j4 2>&1 | grep -E "error|warning|Built target" | head -30
ls -la smc_live_node 2>/dev/null && echo BUILD_OK
} > $O/node_build.txt 2>&1
cat $O/node_build.txt
