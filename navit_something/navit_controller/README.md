# 控制器功能包（navit_controller）

## 1. 该模块实现在给定路径下的跟随功能：

给定地图，控制器会根据目前的位置，局部障碍物，和给定的路线进行下发速度的规划。

## 2. ros重要接口说明

提供ros ActionServer /following_path

```yaml
ActionGoal:
# 指定跟踪路径
nav_msgs/Path path

# 选用控制器
string controller_plugin

ActionResult:

std_msgs/Empty result

ActionFeedback:
# 反馈
float32 distance_to_goal #离目标点的距离

float32 current_vel #当前速度

float32 completion_percentage #完成度，[0-100]

```


## 3. 重要插件说明


| nav_controller插件类型   | 说明 |
| :----- | :--: | 
| teb_local_planner/TebLocalPlannerROS|  teb局部规划器 | 
| mpc_local_planner/MpcLocalPlannerROS|  teb局部规划器 | 
| fg100_mpc_local_planner/MpcLocalPlannerROS| mpc局部规划器，用于高喷FG100项目|
| m100_mpc_local_planner/MpcLocalPlannerROS| mpc局部规划器，用于M100项目|
| PurePursuitController| 纯跟随局部规划器 |


## 4. 单元测试

[navit_controller单元测试](test/README.md)
