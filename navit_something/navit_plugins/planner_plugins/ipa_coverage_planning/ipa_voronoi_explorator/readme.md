# ipa_voronoi_explorator

### 使用方法

1. 启动测试程序

```shell
# 规划器选择插件的方式启动
roslaunch ipa_voronoi_explorator voronoi_explorator_planner_test.launch
```

或者

```shell
# 只启动测试用例
roslaunch ipa_voronoi_explorator voronoi_explorator_test.launch
```

2. 打开rviz 添加 PolygonSelectionTool 工具编辑要进行全覆盖的区域。  
   未指定覆盖区域时将从地图中选择最大区域进行全覆盖规划。

3. 使用rviz PublishPoint 选择规划起始点。将从起始点开始进行全覆盖规划。 

