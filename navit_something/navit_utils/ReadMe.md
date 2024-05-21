### PolygonSelectionTool 使用

1. source 编译的目录打卡rviz 添加PolygonSelectionTool 插件，点击该插件可以画图
2. 按下左键拖动，可以画外轮廓（通行区域），按下右键拖动可以画内轮廓（禁行区域），中键清除上一个区域（通行区域/禁行区域）
3. 通过 rosservice call /get_selection "{}" 获取区域边界
   ```
   ---
   PolygonArrayStamped[] selection
   ```
