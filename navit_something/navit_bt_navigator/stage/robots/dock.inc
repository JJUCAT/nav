define dock model
(
    block
      (
            points 6
            point[0] [0.75 0]
            point[1] [1 0.25]
            point[2] [1 0.75]
            point[3] [0.75 1]
            point[4] [0 1]
            point[5] [0 0]
            z [0 1]
      )
      
)

define dock_v_shape model
(
    block
    (
        points 7
        point[0] [0 0]
        point[1] [0 1.2]
        point[2] [0.75 1.2]
        point[3] [0.75 1.1]
        point[4] [0.6 0.6]
        point[5] [0.75 0.1]
        point[6] [0.75 0.0]
        z [0 1]
    )
)

define static_obstacle model
(
    block
    (
        points 4
        point[0] [0 0]
        point[1] [0 1]
        point[2] [1 1]
        point[3] [1 0]
        z [0 1]
    )
)
