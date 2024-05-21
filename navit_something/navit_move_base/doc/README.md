## Build

```
catkin_make install -DCATKIN_WHITELIST_PACKAGES="navit_auto_dock;navit_msgs;navit_costmap;navit_collision_checker;navit_move_base;teb_local_planner;nem_local_planner;m100_dock_local_planner;nem_global_planner;costmap_converter;navfn;navit_velocity_smoother;navit_core;navit_routing;tsp;path_follower;path_smoother"
```
## navit move base 仿真运行
- 运行节点
```
roslaunch navit_move_base movebase_sim.launch
```

- 加载点边文件
```
rosservice call /load_nem_data "node_file_path: '~/navit_ws/src/navit_stack/navit_move_base/cfg/actual_nodes.json' \
edge_file_path: '~/navit_ws/src/navit_stack/navit_move_base/cfg/actual_edges.json'"
```

- 选择地图id
```
rostopic pub -1 /car_loc_mapid_cmd std_msgs/String "data: '1478773979551608833'"
```

- 给定初始位置 2D Pose Estimate

- 给定Goal 2DNavGoal（给定目标点在选择的地图上，并且在点边上的点上）

- 回充插件作为一个控制器，回充前需要切换控制器

```
rosservice call /navit_move_base/controller_switch "plugin_name: 'dock_local_planner'"
```
