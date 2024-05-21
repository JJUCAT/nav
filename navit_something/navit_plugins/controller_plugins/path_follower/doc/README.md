# 该功能包作为用于机器人的局部规划控制器模块

## Dependencies
- protobuf 3.0
- gsl (GNU Scientific Library) https://www.gnu.org/software/gsl/

##

## 模块局部避障方法


## 该模块具备以下功能
- [ ] 支持差速底盘和阿克曼底盘低速无人车/机器人
- [ ] 具备绕障和停障功能
- [ ] 支持多种特殊区域
- [ ] ...


## 接口

- 模块类继承于 navit_core::BaseLocalPlanner

## 效果


## 关于配置文件


## 说明

- 功能包中增加了proto格式的配置文件和参数，方便去 ros

## 关于碰撞检测的说明
 碰撞检测功能需要在controller中实现：
 分为

 1. 遇障减速不停止
 2. 遇障减速停止
 3. 遇障急停 遇障急停需要考虑到机器人的刹车距离、机器人的减速距离等，需要增加配置参数。


 ## 关于costmap重构的问题

 ## Todo

- [ ] 碰撞检测可以抽象出来一个类，放在controller中代码臃肿。