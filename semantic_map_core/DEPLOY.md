# 真机部署指南(雷达 + IMU + RGBD)

目标形态:

```
雷达 + IMU ──► lightning(SLAM) ──► TF: map_frame ← camera_frame   [已跑通]
                                        │
RGBD 相机 ──► /rgb  ──► seg_node ──► /seg/labels (mono8 类别id)    [唯一要新建的]
         └──► /depth (32FC1 米) ─┐    │
                                 ▼    ▼
                        smc_live_node(C++ 自研 Hydra)
                          · 每帧: 反投影 + 射线积分 → 语义占据图
                          · 周期: ESDF→GVD→骨架→房间→物体→merge → 三层 DSG
                          · 发布: /semantic_voxels /dsg/* /nav/surface_map_L*
                          · 存档: map_dir(重启自动恢复)
                          · 查询: TCP 18080
                                 │
                        MCP server ──stdio──► agent
```

## 0. 依赖

| 组件 | 依赖 |
|---|---|
| smc_live_node | ROS 2(Jazzy 验证过)、rclcpp、sensor_msgs、nav_msgs、visualization_msgs、message_filters、tf2_ros;C++17;可选 OpenMP |
| MCP 服务 | Python 3(标准库,**零第三方依赖**) |
| seg_node | Python 3 + numpy + opencv;后端二选一:onnxruntime 或 torch |

## 1. 编译

```bash
source /opt/ros/<distro>/setup.bash
cd semantic_map_core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS_NODE=ON
make -j4            # 产出 build/smc_live_node
```
纯 cmake,不需要 colcon/ament(要放进 ROS 工作区也可以,加个 package.xml 即可)。

## 2. 三个必须对齐的接口

### 2.1 输入话题(全部可配)
| 话题 | 类型 | 要求 |
|---|---|---|
| depth | `sensor_msgs/Image` | **32FC1,单位米**(16UC1 毫米需先转换) |
| seg | `sensor_msgs/Image` | **mono8,像素值 = 类别 id**;`header.stamp` 必须与 depth 同源 |
| rgb | `sensor_msgs/Image` | rgb8/bgr8 |
| camera_info | `sensor_msgs/CameraInfo` | 提供 fx/fy/cx/cy |

三路图像走 ApproximateTime 同步,**stamp 必须一致**——分割节点务必原样复制 RGB 的 header。

### 2.2 TF(位姿唯一来源)
必须能查到 `map_frame ← depth.header.frame_id`。lightning 发 `odom→base`,再加一条静态 `base→camera` 外参:
```bash
ros2 run tf2_ros static_transform_publisher --x .. --y .. --z .. \
  --qx .. --qy .. --qz .. --qw .. --frame-id base --child-frame-id camera_link
```
**外参不准 = 语义地图和点云对不齐**,这是最常见的坑。

### 2.3 labelspace(类别表)
yaml 里每行 `- {label: 31, name: floor}`。三处必须一致:分割模型输出 → label_map 映射 → labelspace。
名字含 `wall/ceiling/floor/ground/carpet/stair/…` 的类被当作**结构**(不提取为物体,渲染半透明);
`surface_places_labels: [..]` 列出**地面类**(供 2D 代价图判定可走)。

## 3. 启动(真机)

```bash
# 1) SLAM(你已跑通)
ros2 run lightning run_slam_online --config=<your.yaml>
# 2) 相机外参
ros2 run tf2_ros static_transform_publisher ... --frame-id base --child-frame-id camera_link
# 3) 实时语义分割
python3 seg/seg_node.py --backend onnx --model seg.onnx --label-map map.json \
    --rgb-topic /camera/color/image_raw --seg-topic /seg/labels
# 4) 自研 Hydra 建图 + 查询服务
./build/smc_live_node --ros-args --params-file config/real_robot.yaml
# 5) 可视化(可选)
rviz2 -d config/smc_live.rviz
```

参数全在 `config/real_robot.yaml`,**改参数不改代码**。必调三项:
- `bounds`:你的场地范围(内存 ≈ 体素数 × 7 字节)
- `labelspace`:与分割模型类别一致
- 话题名 / `map_frame`

单层场地把 `level_gap` 设成 100.0 即禁用楼层分层。

## 4. MCP 查询服务

```bash
python3 mcp/smc_mcp_server.py --port 18080     # 节点在跑 → 直查内存
python3 mcp/smc_mcp_server.py --offline        # 节点没跑 → 读存档
```
agent 端(如 Claude Desktop)配置:
```json
{"mcpServers": {"semantic-map": {"command": "python3",
  "args": ["/opt/smc/mcp/smc_mcp_server.py"]}}}
```
工具:`get_robot_pose` `get_robot_room` `list_rooms` `room_contents` `find_object`
`get_object` `get_relations` `scene_summary`。每个回答带 `_source`(live_rpc / archive)。

## 5. 算力调参(VM 2 核实测:积分 8ms/帧、场景图 181ms/周期、RSS 111MB)

| 现象 | 调法 |
|---|---|
| 积分跟不上帧率 | `pixel_stride` 6→8/10;`depth_max` 调小 |
| 场景图周期太久 | `cycle_interval` 调大;`bounds` 收紧(体素数是主因);编译开 OpenMP |
| 内存吃紧 | `voxel_size` 0.1→0.15;`bounds` 收紧;`heavy_save_every` 调大 |
| 查询要更新鲜 | `cycle_interval` 调小(位姿本来就是每帧更新的) |

## 6. 先用 House 数据集验证全流程(强烈建议)

`deliver/house_sim_bag.tar` 里有雷达/IMU/RGB/depth/**semantic**/camera_info 全套。
因为它自带 GT 分割,**可以先不接 seg_node**,直接验证 lightning + 建图 + 查询链路:

```bash
tar -xf house_sim_bag.tar -C ~/dataset
ros2 run lightning run_slam_online --config=default_tg_house.yaml
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id base --child-frame-id cam_optical
./build/smc_live_node        # 默认参数即对应这个数据集
ros2 bag play --rate 2.0 ~/dataset/house_sim_bag
python3 mcp/agent_demo.py    # 看 agent 查询是否正常
```
跑通后再把 `seg_topic` 指向你自己的 seg_node,即完成真机迁移。

## 7. 已知边界

- 位姿用的是相机(传感器)原点近似机器人;需要严格 base 位姿可再查一次 `map_frame←base`。
- 场景图是**周期全图重算**(室内规模 170ms 够用);超大场景需要窗口化增量(见 OPTIMIZATION.md)。
- 语义质量完全取决于分割模型;类别抖动会让物体反复增删(靠 `merge_observation` 的 misses 机制吸收)。
