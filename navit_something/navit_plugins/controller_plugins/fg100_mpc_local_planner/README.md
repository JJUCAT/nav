mpc_local_planner ROS Package for FG100, add a simple_car_delay_model which depend on state_feedback of steer_angle and linear_x. 
state_feedback topic: /move_base_node/MpcLocalPlannerROS/state_feedback
state_feedback struct: [empty, empty, empty, steer_angle, linear_x]

Params example of "simple_car_delay":

MpcLocalPlannerROS:

  odom_topic: odom

  ## Robot settings
  robot:
    type: "simple_car_delay"
    simple_car_delay:
      wheelbase: 2.0
      time_constant:
        T_1: 0.779  #前轮状态方程时间常数
        T_2: 0.385  #后轮状态方程时间常数
      front_wheel_driving: False
      max_vel_x: 0.7
      max_vel_x_backwards: 0.6
      max_steering_angle: 0.9
      acc_lim_x: 0.25 # deactive bounds with zero
      dec_lim_x: 0.25 # deactive bounds with zero
      max_steering_rate: 0.2 # deactive bounds with zero


  ## Footprint model for collision avoidance
  footprint_model: # types: "point", "circular", "two_circles", "line", "polygon"
    type: "point"
    radius: 0.2 # for type "circular"
    line_start: [0.0, 0.0] # for type "line"
    line_end: [0.4, 0.0] # for type "line"
    front_offset: 0.2 # for type "two_circles"
    front_radius: 0.2 # for type "two_circles"
    rear_offset: 0.2 # for type "two_circles"
    rear_radius: 0.2 # for type "two_circles"
    vertices: [ [-0.5,-0.8], [2.5,-0.8], [2.5,0.8], [-0.5,0.8] ] # for type "polygon"
    is_footprint_dynamic: False


  ## Collision avoidance
  collision_avoidance:
    min_obstacle_dist: 0.27 # Note, this parameter must be chosen w.r.t. the footprint_model
    enable_dynamic_obstacles: False
    force_inclusion_dist: 0.5
    cutoff_dist: 2.5
    include_costmap_obstacles: False
    costmap_obstacles_behind_robot_dist: 1.0
    collision_check_no_poses: 30
    collision_check_min_resolution_angular: 0.01


  ## Planning grid
  grid:
    type: "fd_grid"
    grid_size_ref: 20
    dt_ref: 1.0
    xf_fixed: [True, True, True, True, True]
    warm_start: True
    collocation_method: "forward_differences"
    cost_integration_method: "left_sum"
    variable_grid:
      enable: True
      min_dt: 0.0;
      max_dt: 10.0;
      grid_adaptation:
        enable: True
        dt_hyst_ratio: 0.1
        min_grid_size: 2
        max_grid_size: 50

  ## Planning options
  planning:
    objective:
      type: "minimum_time_via_points" # minimum_time requires grid/variable_grid/enable=True and grid/xf_fixed set properly
      quadratic_form:
        state_weights: [2.0, 2.0, 2.0]
        control_weights: [1.0, 1.0]
        integral_form: False
      minimum_time_via_points:
        position_weight: 5
        orientation_weight: 0.0
        via_points_ordered: True
    terminal_cost:
      type: "none" # can be "none"
      quadratic:
        final_state_weights: [2.0, 2.0, 2.0]
    terminal_constraint:
      type: "none" # can be "none"
      l2_ball:
        weight_matrix: [1.0, 1.0, 1.0]
        radius: 5

  ## Controller options
  controller:
    outer_ocp_iterations: 1
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.2
    global_plan_overwrite_orientation: False
    global_plan_prune_distance: 1.0
    global_plan_viapoint_sep: 2.0
    allow_init_with_backward_motion: True
    max_global_plan_lookahead_dist: 3.0
    force_reinit_new_goal_dist: 1.0
    force_reinit_new_goal_angular: 1.57
    prefer_x_feedback: False
    publish_ocp_results: True

  ## Solver settings
  solver:
    type: "ipopt"
    ipopt:
      iterations: 70
      max_cpu_time: -1.0
      ipopt_numeric_options:
        tol: 1e-4
      ipopt_string_options:
        linear_solver: "mumps"
        hessian_approximation: "limited-memory" # exact/limited-memory, WARNING 'exact' does currently not work well with the carlike model
    lsq_lm:
      iterations: 10
      weight_init_eq: 2
      weight_init_ineq: 2
      weight_init_bounds: 2
      weight_adapt_factor_eq: 1.5
      weight_adapt_factor_ineq: 1.5
      weight_adapt_factor_bounds: 1.5
      weight_adapt_max_eq: 500
      weight_adapt_max_ineq: 500
      weight_adapt_max_bounds: 500

