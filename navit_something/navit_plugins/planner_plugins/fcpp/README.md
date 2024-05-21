Full Coverage path Planning
===
# Build
1. 进入程序包根目录, run .ci/build.sh
2. 在根目录下的build文件夹中查找 .deb文件
3. 使用dpkg工具安装.deb文件
# Example
1. 执行单弓型示例： test_singlebow + 地图文件
2. 执行双弓型示例： test_doublebow + 地图文件
# TODO 
1. 优化全覆盖转向路径
2. 优化遍历覆盖方法
3. 根目录下提供相关的ROS包及接口
