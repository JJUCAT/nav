###### Navit rviz panel 主要用于辅助导航模块进行编辑地图、编辑任务，让开发人员在rviz 终端中进行地图编辑、任务编辑。该模块需要配合Navit mapserver 使用。

###### 地图接口

```
上传地图 <std_msgs::String> "/navit/map_info_update";

拉取地图 <std_msgs::String> "/navit/map_info"

保存地图 <std_srvs::Empty> "/navit/map_info_save"
```

###### 地图格式：

```
proto/message_navit_map.proto
```

###### 任务接口：

```
推送任务 <std_msgs::String> "/navit/task_control"
```

###### 任务格式：

```
proto/message_navit_task.proto
```

###### 目前具备的功能

* [X] 添加/删除多类型多边形区域
* [X] 添加/删除多类型站点
* [X] 添加/删除直线连线
* [X] 上传地图
* [X] 拉取地图
* [X] 保存/清除地图
* [X] 高亮选中多边形区域并发送任务
* [X] 选中自由点点并发送任务
* [X] 选中多站点发送任务

##### 需要持续迭代具备的功能

* [ ] 多边形取消选中(取消高亮)
* [ ] 多站点取消选中(取消高亮)
* [ ] 可编辑区域
* [ ] 可编辑站点
* [ ] 增加移动功能
