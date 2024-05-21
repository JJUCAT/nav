# slic3r全覆盖路径规划算法插件 
slic3r_coverage_planner是基于slic3r的全覆盖路径规划算法插件，可以在slic3r的基础上实现全覆盖路径规划

提供了弓字形和回字形两种全覆盖路径规划算法

# example config
```yaml
is_contour: true # 是否回字形全覆盖，默认为true，否则为弓字形全覆盖
outer_offset: 0.5 # 外轮廓偏移量，单位m, 默认0.5m
distance: 0.5 # 路径间距，单位m, 默认0.5m
angle: 0.0 # 路径角度，单位度，默认0度
```


