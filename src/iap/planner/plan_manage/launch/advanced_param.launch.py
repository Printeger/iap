import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchConfigurations
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
    
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
    
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)
    point5_x = LaunchConfiguration('point5_x', default=0.0)
    point5_y = LaunchConfiguration('point5_y', default=0.0)
    point5_z = LaunchConfiguration('point5_z', default=1.0)
    point6_x = LaunchConfiguration('point6_x', default=0.0)
    point6_y = LaunchConfiguration('point6_y', default=0.0)
    point6_z = LaunchConfiguration('point6_z', default=1.0)

    flight_type = LaunchConfiguration('flight_type', default=2)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
    use_integrity_cost = LaunchConfiguration('use_integrity_cost', default=False)
    integrity_debug_csv_path = LaunchConfiguration(
        'integrity_debug_csv_path',
        default='/home/dev/ws_iap/src/iap/log/latest/export/planner_integrity_cost_debug.csv')
    lambda_integrity = LaunchConfiguration('lambda_integrity', default=0.00001)
    p1_use_integrity_cost = LaunchConfiguration('p1_use_integrity_cost', default=False)
    p1_metrics_only = LaunchConfiguration('p1_metrics_only', default=True)
    p1_lambda_integrity = LaunchConfiguration('p1_lambda_integrity', default=0.0)
    p1_sample_dt_min_s = LaunchConfiguration('p1_sample_dt_min_s', default=0.1)
    p1_sample_dt_scale = LaunchConfiguration('p1_sample_dt_scale', default=1.0)
    p1_max_samples_per_eval = LaunchConfiguration('p1_max_samples_per_eval', default=30)
    p1_integrity_cost_max = LaunchConfiguration('p1_integrity_cost_max', default=100.0)
    p1_integrity_grad_norm_max = LaunchConfiguration('p1_integrity_grad_norm_max', default=0.1)
    p1_unknown_policy = LaunchConfiguration('p1_unknown_policy', default='skip')
    p1_unknown_soft_penalty = LaunchConfiguration('p1_unknown_soft_penalty', default=1.0)
    p1_debug_csv_enable = LaunchConfiguration('p1_debug_csv_enable', default=False)
    p1_debug_csv_path = LaunchConfiguration('p1_debug_csv_path', default='')
    p2_enable_candidate_ranking = LaunchConfiguration('p2_enable_candidate_ranking', default=False)
    p2_metrics_only = LaunchConfiguration('p2_metrics_only', default=True)
    p2_sample_dt_s = LaunchConfiguration('p2_sample_dt_s', default=0.2)
    p2_lambda_candidate_integrity = LaunchConfiguration('p2_lambda_candidate_integrity', default=1.0)
    p2_w_max_cost = LaunchConfiguration('p2_w_max_cost', default=0.25)
    p2_w_unknown = LaunchConfiguration('p2_w_unknown', default=5.0)
    p2_w_stale = LaunchConfiguration('p2_w_stale', default=2.0)
    p2_min_valid_ratio = LaunchConfiguration('p2_min_valid_ratio', default=0.3)
    p2_debug_csv_enable = LaunchConfiguration('p2_debug_csv_enable', default=False)
    p2_debug_csv_path = LaunchConfiguration('p2_debug_csv_path', default='')
    p3_enable_local_reference_bias = LaunchConfiguration('p3_enable_local_reference_bias', default=False)
    p3_enable_global_reference_bias = LaunchConfiguration('p3_enable_global_reference_bias', default=False)
    p3_local_bias_radius_m = LaunchConfiguration('p3_local_bias_radius_m', default=1.5)
    p3_min_improvement_ratio = LaunchConfiguration('p3_min_improvement_ratio', default=0.05)
    p3_w_risk = LaunchConfiguration('p3_w_risk', default=1.0)
    p3_w_detour = LaunchConfiguration('p3_w_detour', default=0.25)
    p3_w_unknown = LaunchConfiguration('p3_w_unknown', default=5.0)
    p3_min_corridor_valid_ratio = LaunchConfiguration('p3_min_corridor_valid_ratio', default=0.8)
    p3_station_spacing_m = LaunchConfiguration('p3_station_spacing_m', default=2.0)
    p3_lateral_sample_step_m = LaunchConfiguration('p3_lateral_sample_step_m', default=1.0)
    p3_lateral_sample_count_each_side = LaunchConfiguration('p3_lateral_sample_count_each_side', default=3)
    p3_beam_width = LaunchConfiguration('p3_beam_width', default=5)
    p3_max_detour_ratio = LaunchConfiguration('p3_max_detour_ratio', default=1.5)
    p3_debug_csv_enable = LaunchConfiguration('p3_debug_csv_enable', default=False)
    p3_debug_csv_path = LaunchConfiguration('p3_debug_csv_path', default='')
    p4_enable_risk_aware_astar = LaunchConfiguration('p4_enable_risk_aware_astar', default=False)
    p4_lambda_p4_risk = LaunchConfiguration('p4_lambda_p4_risk', default=0.05)
    p4_risk_cost_max = LaunchConfiguration('p4_risk_cost_max', default=100.0)
    p4_unknown_edge_penalty = LaunchConfiguration('p4_unknown_edge_penalty', default=1.0)
    p4_max_extra_path_ratio = LaunchConfiguration('p4_max_extra_path_ratio', default=1.3)
    p4_fallback_to_original_when_risk_not_ready = LaunchConfiguration('p4_fallback_to_original_when_risk_not_ready', default=True)
    p4_debug_csv_enable = LaunchConfiguration('p4_debug_csv_enable', default=False)
    p4_debug_csv_path = LaunchConfiguration('p4_debug_csv_path', default='')
    integrity_field_stale_timeout_s = LaunchConfiguration('integrity_field_stale_timeout_s', default=0.5)
    integrity_nearest_radius_m = LaunchConfiguration('integrity_nearest_radius_m', default=1.0)
    integrity_cost_max = LaunchConfiguration('integrity_cost_max', default=1000.0)
    integrity_grad_norm_max = LaunchConfiguration('integrity_grad_norm_max', default=0.1)
    use_integrity_front_search = LaunchConfiguration('use_integrity_front_search', default=False)
    use_integrity_global_search = LaunchConfiguration('use_integrity_global_search', default=False)
    lambda_integrity_front = LaunchConfiguration('lambda_integrity_front', default=2.0)
    integrity_front_cost_topic = LaunchConfiguration('integrity_front_cost_topic', default='/iap/integrity_front_cost_field')
    integrity_front_nearest_radius_m = LaunchConfiguration('integrity_front_nearest_radius_m', default=1.5)
    integrity_front_stale_timeout_s = LaunchConfiguration('integrity_front_stale_timeout_s', default=1.0)
    integrity_front_cost_max = LaunchConfiguration('integrity_front_cost_max', default=10.0)
    risk_overlay_enable = LaunchConfiguration('risk_overlay_enable', default=True)
    risk_overlay_use_for_astar = LaunchConfiguration('risk_overlay_use_for_astar', default=False)
    risk_overlay_use_for_bspline = LaunchConfiguration('risk_overlay_use_for_bspline', default=False)
    risk_overlay_topic = LaunchConfiguration('risk_overlay_topic', default='/iap/integrity_front_cost_field')
    risk_overlay_lambda_unknown = LaunchConfiguration('risk_overlay_lambda_unknown', default=10.0)
    risk_overlay_lambda_stale = LaunchConfiguration('risk_overlay_lambda_stale', default=1.0)
    risk_overlay_stale_timeout_s = LaunchConfiguration('risk_overlay_stale_timeout_s', default=1.0)
    risk_overlay_stale_tau_s = LaunchConfiguration('risk_overlay_stale_tau_s', default=1.0)
    risk_overlay_r_soft = LaunchConfiguration('risk_overlay_r_soft', default=0.75)
    risk_overlay_w_soft = LaunchConfiguration('risk_overlay_w_soft', default=1.0)
    risk_overlay_w_hard = LaunchConfiguration('risk_overlay_w_hard', default=10.0)
    risk_overlay_c_unsafe = LaunchConfiguration('risk_overlay_c_unsafe', default=10.0)
    risk_overlay_eps_al_m = LaunchConfiguration('risk_overlay_eps_al_m', default=1.0e-3)
    risk_overlay_gamma_h = LaunchConfiguration('risk_overlay_gamma_h', default=0.8)
    risk_overlay_gamma_v = LaunchConfiguration('risk_overlay_gamma_v', default=0.8)
    risk_overlay_drone_radius_m = LaunchConfiguration('risk_overlay_drone_radius_m', default=0.35)
    risk_overlay_safety_buffer_m = LaunchConfiguration('risk_overlay_safety_buffer_m', default=0.20)
    risk_overlay_clearance_max_m = LaunchConfiguration('risk_overlay_clearance_max_m', default=5.0)
    risk_overlay_clearance_unknown_m = LaunchConfiguration('risk_overlay_clearance_unknown_m', default=1.0)
    risk_overlay_edge_sample_alpha = LaunchConfiguration('risk_overlay_edge_sample_alpha', default=0.75)
    risk_overlay_debug_publish = LaunchConfiguration('risk_overlay_debug_publish', default=True)
    risk_overlay_debug_topic = LaunchConfiguration('risk_overlay_debug_topic', default='/grid_map/risk_overlay_debug')
    risk_overlay_debug_publish_hz = LaunchConfiguration('risk_overlay_debug_publish_hz', default=2.0)
    risk_overlay_debug_color_mode = LaunchConfiguration('risk_overlay_debug_color_mode', default='cost')
    risk_overlay_debug_cost_max = LaunchConfiguration('risk_overlay_debug_cost_max', default=50.0)
    risk_overlay_bspline_samples_per_segment = LaunchConfiguration('risk_overlay_bspline_samples_per_segment', default=3)
    integrity_global_astar_step_m = LaunchConfiguration('integrity_global_astar_step_m', default=0.5)
    integrity_global_max_waypoints = LaunchConfiguration('integrity_global_max_waypoints', default=80)
    p0_enable_risk_grid = LaunchConfiguration('p0_enable_risk_grid', default=False)
    p0_resolution_m = LaunchConfiguration('p0_resolution_m', default=0.75)
    p0_size_x_m = LaunchConfiguration('p0_size_x_m', default=30.0)
    p0_size_y_m = LaunchConfiguration('p0_size_y_m', default=30.0)
    p0_size_z_m = LaunchConfiguration('p0_size_z_m', default=6.0)
    p0_refresh_period_s = LaunchConfiguration('p0_refresh_period_s', default=0.5)
    p0_stale_timeout_s = LaunchConfiguration('p0_stale_timeout_s', default=1.0)
    p0_skip_occupied_voxels = LaunchConfiguration('p0_skip_occupied_voxels', default=True)
    p0_debug_metrics_enable = LaunchConfiguration('p0_debug_metrics_enable', default=False)
    p0_health_topic = LaunchConfiguration('p0_health_topic', default='planning/risk_grid_health')
    p5_enable_runtime_gate = LaunchConfiguration('p5_enable_runtime_gate', default=False)
    p5_enable_final_gate = LaunchConfiguration('p5_enable_final_gate', default=False)
    p5_horizon_s = LaunchConfiguration('p5_horizon_s', default=2.0)
    p5_sample_dt_s = LaunchConfiguration('p5_sample_dt_s', default=0.2)
    p5_current_stale_to_replan_s = LaunchConfiguration('p5_current_stale_to_replan_s', default=0.5)
    p5_current_stale_to_emergency_s = LaunchConfiguration('p5_current_stale_to_emergency_s', default=2.0)
    p5_current_low_margin_to_emergency_s = LaunchConfiguration('p5_current_low_margin_to_emergency_s', default=2.0)
    p5_future_unknown_to_emergency_s = LaunchConfiguration('p5_future_unknown_to_emergency_s', default=2.0)
    p5_final_gate_max_consecutive_failures = LaunchConfiguration('p5_final_gate_max_consecutive_failures', default=3)
    p5_final_gate_max_failure_duration_s = LaunchConfiguration('p5_final_gate_max_failure_duration_s', default=1.0)
    p5_current_replan_margin_m = LaunchConfiguration('p5_current_replan_margin_m', default=0.3)
    p5_current_emergency_margin_m = LaunchConfiguration('p5_current_emergency_margin_m', default=-0.2)
    p5_future_replan_margin_m = LaunchConfiguration('p5_future_replan_margin_m', default=0.3)
    p5_future_emergency_margin_m = LaunchConfiguration('p5_future_emergency_margin_m', default=-0.5)
    p5_max_bad_ratio = LaunchConfiguration('p5_max_bad_ratio', default=0.25)
    p5_max_unknown_ratio = LaunchConfiguration('p5_max_unknown_ratio', default=0.30)
    p5_bad_tick_to_replan = LaunchConfiguration('p5_bad_tick_to_replan', default=2)
    p5_good_tick_to_clear = LaunchConfiguration('p5_good_tick_to_clear', default=2)
    p5_pred_alert_limit_mode = LaunchConfiguration('p5_pred_alert_limit_mode', default='current_msg_constant')
    p5_pred_alert_limit_constant_hal_m = LaunchConfiguration('p5_pred_alert_limit_constant_hal_m', default=10.0)
    p5_pred_alert_limit_constant_val_m = LaunchConfiguration('p5_pred_alert_limit_constant_val_m', default=10.0)
    p5_pred_alert_limit_min_hal_m = LaunchConfiguration('p5_pred_alert_limit_min_hal_m', default=0.1)
    p5_pred_alert_limit_max_hal_m = LaunchConfiguration('p5_pred_alert_limit_max_hal_m', default=50.0)
    p5_pred_alert_limit_min_val_m = LaunchConfiguration('p5_pred_alert_limit_min_val_m', default=0.1)
    p5_pred_alert_limit_max_val_m = LaunchConfiguration('p5_pred_alert_limit_max_val_m', default=50.0)
    p5_pred_alert_limit_clearance_search_radius_m = LaunchConfiguration('p5_pred_alert_limit_clearance_search_radius_m', default=5.0)
    p5_pred_alert_limit_clearance_step_m = LaunchConfiguration('p5_pred_alert_limit_clearance_step_m', default=0.25)
    p5_pred_alert_limit_drone_radius_m = LaunchConfiguration('p5_pred_alert_limit_drone_radius_m', default=0.35)
    p5_pred_alert_limit_clearance_scale = LaunchConfiguration('p5_pred_alert_limit_clearance_scale', default=1.0)
    p5_pred_alert_limit_vertical_scale = LaunchConfiguration('p5_pred_alert_limit_vertical_scale', default=1.0)
    p5_debug_metrics_enable = LaunchConfiguration('p5_debug_metrics_enable', default=False)
    p5_status_topic = LaunchConfiguration('p5_status_topic', default='planning/integrity_gate_status')
    
    obj_num_set = LaunchConfiguration('obj_num_set', default=10)
    
    drone_id = LaunchConfiguration('drone_id', default=0)

    # DeclareLaunchArguments
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
    
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    point5_x_arg = DeclareLaunchArgument('point5_x', default_value=point5_x, description='Waypoint 5 X coordinate')
    point5_y_arg = DeclareLaunchArgument('point5_y', default_value=point5_y, description='Waypoint 5 Y coordinate')
    point5_z_arg = DeclareLaunchArgument('point5_z', default_value=point5_z, description='Waypoint 5 Z coordinate')
    point6_x_arg = DeclareLaunchArgument('point6_x', default_value=point6_x, description='Waypoint 6 X coordinate')
    point6_y_arg = DeclareLaunchArgument('point6_y', default_value=point6_y, description='Waypoint 6 Y coordinate')
    point6_z_arg = DeclareLaunchArgument('point6_z', default_value=point6_z, description='Waypoint 6 Z coordinate')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    use_integrity_cost_arg = DeclareLaunchArgument('use_integrity_cost', default_value=use_integrity_cost, description='Enable optional integrity soft cost')
    integrity_debug_csv_path_arg = DeclareLaunchArgument('integrity_debug_csv_path', default_value=integrity_debug_csv_path, description='Planner integrity cost debug CSV path')
    lambda_integrity_arg = DeclareLaunchArgument('lambda_integrity', default_value=lambda_integrity, description='Planner integrity cost weight')
    p1_use_integrity_cost_arg = DeclareLaunchArgument('p1_use_integrity_cost', default_value=p1_use_integrity_cost, description='Enable P1 backend integrity soft cost')
    p1_metrics_only_arg = DeclareLaunchArgument('p1_metrics_only', default_value=p1_metrics_only, description='Compute P1 metrics without adding objective cost')
    p1_lambda_integrity_arg = DeclareLaunchArgument('p1_lambda_integrity', default_value=p1_lambda_integrity, description='P1 integrity weight tuned by gradient ratio')
    p1_sample_dt_min_s_arg = DeclareLaunchArgument('p1_sample_dt_min_s', default_value=p1_sample_dt_min_s, description='P1 minimum trajectory sample dt')
    p1_sample_dt_scale_arg = DeclareLaunchArgument('p1_sample_dt_scale', default_value=p1_sample_dt_scale, description='P1 sample dt scale relative to B-spline interval')
    p1_max_samples_per_eval_arg = DeclareLaunchArgument('p1_max_samples_per_eval', default_value=p1_max_samples_per_eval, description='P1 sample cap per L-BFGS evaluation')
    p1_integrity_cost_max_arg = DeclareLaunchArgument('p1_integrity_cost_max', default_value=p1_integrity_cost_max, description='P1 per-sample integrity cost clip')
    p1_integrity_grad_norm_max_arg = DeclareLaunchArgument('p1_integrity_grad_norm_max', default_value=p1_integrity_grad_norm_max, description='P1 per-sample gradient norm clip')
    p1_unknown_policy_arg = DeclareLaunchArgument('p1_unknown_policy', default_value=p1_unknown_policy, description='P1 unknown policy: skip or small_penalty')
    p1_unknown_soft_penalty_arg = DeclareLaunchArgument('p1_unknown_soft_penalty', default_value=p1_unknown_soft_penalty, description='P1 debug penalty for unknown samples')
    p1_debug_csv_enable_arg = DeclareLaunchArgument('p1_debug_csv_enable', default_value=p1_debug_csv_enable, description='Enable P1 debug CSV output')
    p1_debug_csv_path_arg = DeclareLaunchArgument('p1_debug_csv_path', default_value=p1_debug_csv_path, description='P1 debug CSV path')
    p2_enable_candidate_ranking_arg = DeclareLaunchArgument('p2_enable_candidate_ranking', default_value=p2_enable_candidate_ranking, description='Enable P2 candidate ranking or metrics')
    p2_metrics_only_arg = DeclareLaunchArgument('p2_metrics_only', default_value=p2_metrics_only, description='Compute P2 candidate ranking metrics without changing winner')
    p2_sample_dt_s_arg = DeclareLaunchArgument('p2_sample_dt_s', default_value=p2_sample_dt_s, description='P2 trajectory sample dt')
    p2_lambda_candidate_integrity_arg = DeclareLaunchArgument('p2_lambda_candidate_integrity', default_value=p2_lambda_candidate_integrity, description='P2 integrity score weight')
    p2_w_max_cost_arg = DeclareLaunchArgument('p2_w_max_cost', default_value=p2_w_max_cost, description='P2 max cost score weight')
    p2_w_unknown_arg = DeclareLaunchArgument('p2_w_unknown', default_value=p2_w_unknown, description='P2 unknown ratio score weight')
    p2_w_stale_arg = DeclareLaunchArgument('p2_w_stale', default_value=p2_w_stale, description='P2 stale ratio score weight')
    p2_min_valid_ratio_arg = DeclareLaunchArgument('p2_min_valid_ratio', default_value=p2_min_valid_ratio, description='P2 minimum valid sample ratio before ranking')
    p2_debug_csv_enable_arg = DeclareLaunchArgument('p2_debug_csv_enable', default_value=p2_debug_csv_enable, description='Enable P2 debug CSV output')
    p2_debug_csv_path_arg = DeclareLaunchArgument('p2_debug_csv_path', default_value=p2_debug_csv_path, description='P2 debug CSV path')
    p3_enable_local_reference_bias_arg = DeclareLaunchArgument('p3_enable_local_reference_bias', default_value=p3_enable_local_reference_bias, description='Enable P3 local reference bias')
    p3_enable_global_reference_bias_arg = DeclareLaunchArgument('p3_enable_global_reference_bias', default_value=p3_enable_global_reference_bias, description='Enable P3 global reference bias')
    p3_local_bias_radius_m_arg = DeclareLaunchArgument('p3_local_bias_radius_m', default_value=p3_local_bias_radius_m, description='P3 local target search radius')
    p3_min_improvement_ratio_arg = DeclareLaunchArgument('p3_min_improvement_ratio', default_value=p3_min_improvement_ratio, description='P3 minimum improvement ratio before applying bias')
    p3_w_risk_arg = DeclareLaunchArgument('p3_w_risk', default_value=p3_w_risk, description='P3 risk score weight')
    p3_w_detour_arg = DeclareLaunchArgument('p3_w_detour', default_value=p3_w_detour, description='P3 detour score weight')
    p3_w_unknown_arg = DeclareLaunchArgument('p3_w_unknown', default_value=p3_w_unknown, description='P3 unknown sample penalty weight')
    p3_min_corridor_valid_ratio_arg = DeclareLaunchArgument('p3_min_corridor_valid_ratio', default_value=p3_min_corridor_valid_ratio, description='P3 minimum corridor coverage before global bias')
    p3_station_spacing_m_arg = DeclareLaunchArgument('p3_station_spacing_m', default_value=p3_station_spacing_m, description='P3 global corridor station spacing')
    p3_lateral_sample_step_m_arg = DeclareLaunchArgument('p3_lateral_sample_step_m', default_value=p3_lateral_sample_step_m, description='P3 lateral sample spacing')
    p3_lateral_sample_count_each_side_arg = DeclareLaunchArgument('p3_lateral_sample_count_each_side', default_value=p3_lateral_sample_count_each_side, description='P3 lateral samples on each side of corridor')
    p3_beam_width_arg = DeclareLaunchArgument('p3_beam_width', default_value=p3_beam_width, description='P3 global beam width')
    p3_max_detour_ratio_arg = DeclareLaunchArgument('p3_max_detour_ratio', default_value=p3_max_detour_ratio, description='P3 maximum biased path detour ratio')
    p3_debug_csv_enable_arg = DeclareLaunchArgument('p3_debug_csv_enable', default_value=p3_debug_csv_enable, description='Enable P3 debug CSV output')
    p3_debug_csv_path_arg = DeclareLaunchArgument('p3_debug_csv_path', default_value=p3_debug_csv_path, description='P3 debug CSV path')
    p4_enable_risk_aware_astar_arg = DeclareLaunchArgument('p4_enable_risk_aware_astar', default_value=p4_enable_risk_aware_astar, description='Enable P4 collision-segment risk-aware A* guide fallback')
    p4_lambda_p4_risk_arg = DeclareLaunchArgument('p4_lambda_p4_risk', default_value=p4_lambda_p4_risk, description='P4 risk edge cost weight')
    p4_risk_cost_max_arg = DeclareLaunchArgument('p4_risk_cost_max', default_value=p4_risk_cost_max, description='P4 per-edge risk cost clamp')
    p4_unknown_edge_penalty_arg = DeclareLaunchArgument('p4_unknown_edge_penalty', default_value=p4_unknown_edge_penalty, description='P4 unknown risk edge penalty')
    p4_max_extra_path_ratio_arg = DeclareLaunchArgument('p4_max_extra_path_ratio', default_value=p4_max_extra_path_ratio, description='P4 maximum risk-aware path length ratio before fallback')
    p4_fallback_to_original_when_risk_not_ready_arg = DeclareLaunchArgument('p4_fallback_to_original_when_risk_not_ready', default_value=p4_fallback_to_original_when_risk_not_ready, description='Fallback to original A* when P4 risk snapshot is unavailable')
    p4_debug_csv_enable_arg = DeclareLaunchArgument('p4_debug_csv_enable', default_value=p4_debug_csv_enable, description='Enable P4 debug CSV output')
    p4_debug_csv_path_arg = DeclareLaunchArgument('p4_debug_csv_path', default_value=p4_debug_csv_path, description='P4 debug CSV path')
    integrity_field_stale_timeout_s_arg = DeclareLaunchArgument('integrity_field_stale_timeout_s', default_value=integrity_field_stale_timeout_s, description='Planner integrity field stale timeout in seconds')
    integrity_nearest_radius_m_arg = DeclareLaunchArgument('integrity_nearest_radius_m', default_value=integrity_nearest_radius_m, description='Planner integrity nearest sample search radius')
    integrity_cost_max_arg = DeclareLaunchArgument('integrity_cost_max', default_value=integrity_cost_max, description='Planner integrity sample cost clamp')
    integrity_grad_norm_max_arg = DeclareLaunchArgument('integrity_grad_norm_max', default_value=integrity_grad_norm_max, description='Planner integrity gradient norm clamp')
    use_integrity_front_search_arg = DeclareLaunchArgument('use_integrity_front_search', default_value=use_integrity_front_search, description='Enable integrity-aware rebound A*')
    use_integrity_global_search_arg = DeclareLaunchArgument('use_integrity_global_search', default_value=use_integrity_global_search, description='Enable integrity-aware global A*')
    lambda_integrity_front_arg = DeclareLaunchArgument('lambda_integrity_front', default_value=lambda_integrity_front, description='Front-end A* integrity ratio cost weight')
    integrity_front_cost_topic_arg = DeclareLaunchArgument('integrity_front_cost_topic', default_value=integrity_front_cost_topic, description='Front-end integrity cost field topic')
    integrity_front_nearest_radius_m_arg = DeclareLaunchArgument('integrity_front_nearest_radius_m', default_value=integrity_front_nearest_radius_m, description='Front-end integrity nearest sample radius')
    integrity_front_stale_timeout_s_arg = DeclareLaunchArgument('integrity_front_stale_timeout_s', default_value=integrity_front_stale_timeout_s, description='Front-end integrity field stale timeout')
    integrity_front_cost_max_arg = DeclareLaunchArgument('integrity_front_cost_max', default_value=integrity_front_cost_max, description='Front-end integrity normalized cost clamp')
    risk_overlay_enable_arg = DeclareLaunchArgument('risk_overlay_enable', default_value=risk_overlay_enable)
    risk_overlay_use_for_astar_arg = DeclareLaunchArgument('risk_overlay_use_for_astar', default_value=risk_overlay_use_for_astar)
    risk_overlay_use_for_bspline_arg = DeclareLaunchArgument('risk_overlay_use_for_bspline', default_value=risk_overlay_use_for_bspline)
    risk_overlay_topic_arg = DeclareLaunchArgument('risk_overlay_topic', default_value=risk_overlay_topic)
    risk_overlay_lambda_unknown_arg = DeclareLaunchArgument('risk_overlay_lambda_unknown', default_value=risk_overlay_lambda_unknown)
    risk_overlay_lambda_stale_arg = DeclareLaunchArgument('risk_overlay_lambda_stale', default_value=risk_overlay_lambda_stale)
    risk_overlay_stale_timeout_s_arg = DeclareLaunchArgument('risk_overlay_stale_timeout_s', default_value=risk_overlay_stale_timeout_s)
    risk_overlay_stale_tau_s_arg = DeclareLaunchArgument('risk_overlay_stale_tau_s', default_value=risk_overlay_stale_tau_s)
    risk_overlay_r_soft_arg = DeclareLaunchArgument('risk_overlay_r_soft', default_value=risk_overlay_r_soft)
    risk_overlay_w_soft_arg = DeclareLaunchArgument('risk_overlay_w_soft', default_value=risk_overlay_w_soft)
    risk_overlay_w_hard_arg = DeclareLaunchArgument('risk_overlay_w_hard', default_value=risk_overlay_w_hard)
    risk_overlay_c_unsafe_arg = DeclareLaunchArgument('risk_overlay_c_unsafe', default_value=risk_overlay_c_unsafe)
    risk_overlay_eps_al_m_arg = DeclareLaunchArgument('risk_overlay_eps_al_m', default_value=risk_overlay_eps_al_m)
    risk_overlay_gamma_h_arg = DeclareLaunchArgument('risk_overlay_gamma_h', default_value=risk_overlay_gamma_h)
    risk_overlay_gamma_v_arg = DeclareLaunchArgument('risk_overlay_gamma_v', default_value=risk_overlay_gamma_v)
    risk_overlay_drone_radius_m_arg = DeclareLaunchArgument('risk_overlay_drone_radius_m', default_value=risk_overlay_drone_radius_m)
    risk_overlay_safety_buffer_m_arg = DeclareLaunchArgument('risk_overlay_safety_buffer_m', default_value=risk_overlay_safety_buffer_m)
    risk_overlay_clearance_max_m_arg = DeclareLaunchArgument('risk_overlay_clearance_max_m', default_value=risk_overlay_clearance_max_m)
    risk_overlay_clearance_unknown_m_arg = DeclareLaunchArgument('risk_overlay_clearance_unknown_m', default_value=risk_overlay_clearance_unknown_m)
    risk_overlay_edge_sample_alpha_arg = DeclareLaunchArgument('risk_overlay_edge_sample_alpha', default_value=risk_overlay_edge_sample_alpha)
    risk_overlay_bspline_samples_per_segment_arg = DeclareLaunchArgument('risk_overlay_bspline_samples_per_segment', default_value=risk_overlay_bspline_samples_per_segment)
    risk_overlay_debug_publish_arg = DeclareLaunchArgument('risk_overlay_debug_publish', default_value=risk_overlay_debug_publish)
    risk_overlay_debug_topic_arg = DeclareLaunchArgument('risk_overlay_debug_topic', default_value=risk_overlay_debug_topic)
    risk_overlay_debug_publish_hz_arg = DeclareLaunchArgument('risk_overlay_debug_publish_hz', default_value=risk_overlay_debug_publish_hz)
    risk_overlay_debug_color_mode_arg = DeclareLaunchArgument('risk_overlay_debug_color_mode', default_value=risk_overlay_debug_color_mode)
    risk_overlay_debug_cost_max_arg = DeclareLaunchArgument('risk_overlay_debug_cost_max', default_value=risk_overlay_debug_cost_max)
    integrity_global_astar_step_m_arg = DeclareLaunchArgument('integrity_global_astar_step_m', default_value=integrity_global_astar_step_m, description='Integrity-aware global A* step size')
    integrity_global_max_waypoints_arg = DeclareLaunchArgument('integrity_global_max_waypoints', default_value=integrity_global_max_waypoints, description='Maximum global waypoints generated by integrity-aware A*')
    p0_enable_risk_grid_arg = DeclareLaunchArgument('p0_enable_risk_grid', default_value=p0_enable_risk_grid)
    p0_resolution_m_arg = DeclareLaunchArgument('p0_resolution_m', default_value=p0_resolution_m)
    p0_size_x_m_arg = DeclareLaunchArgument('p0_size_x_m', default_value=p0_size_x_m)
    p0_size_y_m_arg = DeclareLaunchArgument('p0_size_y_m', default_value=p0_size_y_m)
    p0_size_z_m_arg = DeclareLaunchArgument('p0_size_z_m', default_value=p0_size_z_m)
    p0_refresh_period_s_arg = DeclareLaunchArgument('p0_refresh_period_s', default_value=p0_refresh_period_s)
    p0_stale_timeout_s_arg = DeclareLaunchArgument('p0_stale_timeout_s', default_value=p0_stale_timeout_s)
    p0_skip_occupied_voxels_arg = DeclareLaunchArgument('p0_skip_occupied_voxels', default_value=p0_skip_occupied_voxels)
    p0_debug_metrics_enable_arg = DeclareLaunchArgument('p0_debug_metrics_enable', default_value=p0_debug_metrics_enable)
    p0_health_topic_arg = DeclareLaunchArgument('p0_health_topic', default_value=p0_health_topic)
    p5_enable_runtime_gate_arg = DeclareLaunchArgument('p5_enable_runtime_gate', default_value=p5_enable_runtime_gate)
    p5_enable_final_gate_arg = DeclareLaunchArgument('p5_enable_final_gate', default_value=p5_enable_final_gate)
    p5_horizon_s_arg = DeclareLaunchArgument('p5_horizon_s', default_value=p5_horizon_s)
    p5_sample_dt_s_arg = DeclareLaunchArgument('p5_sample_dt_s', default_value=p5_sample_dt_s)
    p5_current_stale_to_replan_s_arg = DeclareLaunchArgument('p5_current_stale_to_replan_s', default_value=p5_current_stale_to_replan_s)
    p5_current_stale_to_emergency_s_arg = DeclareLaunchArgument('p5_current_stale_to_emergency_s', default_value=p5_current_stale_to_emergency_s)
    p5_current_low_margin_to_emergency_s_arg = DeclareLaunchArgument('p5_current_low_margin_to_emergency_s', default_value=p5_current_low_margin_to_emergency_s)
    p5_future_unknown_to_emergency_s_arg = DeclareLaunchArgument('p5_future_unknown_to_emergency_s', default_value=p5_future_unknown_to_emergency_s)
    p5_final_gate_max_consecutive_failures_arg = DeclareLaunchArgument('p5_final_gate_max_consecutive_failures', default_value=p5_final_gate_max_consecutive_failures)
    p5_final_gate_max_failure_duration_s_arg = DeclareLaunchArgument('p5_final_gate_max_failure_duration_s', default_value=p5_final_gate_max_failure_duration_s)
    p5_current_replan_margin_m_arg = DeclareLaunchArgument('p5_current_replan_margin_m', default_value=p5_current_replan_margin_m)
    p5_current_emergency_margin_m_arg = DeclareLaunchArgument('p5_current_emergency_margin_m', default_value=p5_current_emergency_margin_m)
    p5_future_replan_margin_m_arg = DeclareLaunchArgument('p5_future_replan_margin_m', default_value=p5_future_replan_margin_m)
    p5_future_emergency_margin_m_arg = DeclareLaunchArgument('p5_future_emergency_margin_m', default_value=p5_future_emergency_margin_m)
    p5_max_bad_ratio_arg = DeclareLaunchArgument('p5_max_bad_ratio', default_value=p5_max_bad_ratio)
    p5_max_unknown_ratio_arg = DeclareLaunchArgument('p5_max_unknown_ratio', default_value=p5_max_unknown_ratio)
    p5_bad_tick_to_replan_arg = DeclareLaunchArgument('p5_bad_tick_to_replan', default_value=p5_bad_tick_to_replan)
    p5_good_tick_to_clear_arg = DeclareLaunchArgument('p5_good_tick_to_clear', default_value=p5_good_tick_to_clear)
    p5_pred_alert_limit_mode_arg = DeclareLaunchArgument('p5_pred_alert_limit_mode', default_value=p5_pred_alert_limit_mode)
    p5_pred_alert_limit_constant_hal_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_constant_hal_m', default_value=p5_pred_alert_limit_constant_hal_m)
    p5_pred_alert_limit_constant_val_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_constant_val_m', default_value=p5_pred_alert_limit_constant_val_m)
    p5_pred_alert_limit_min_hal_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_min_hal_m', default_value=p5_pred_alert_limit_min_hal_m)
    p5_pred_alert_limit_max_hal_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_max_hal_m', default_value=p5_pred_alert_limit_max_hal_m)
    p5_pred_alert_limit_min_val_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_min_val_m', default_value=p5_pred_alert_limit_min_val_m)
    p5_pred_alert_limit_max_val_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_max_val_m', default_value=p5_pred_alert_limit_max_val_m)
    p5_pred_alert_limit_clearance_search_radius_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_clearance_search_radius_m', default_value=p5_pred_alert_limit_clearance_search_radius_m)
    p5_pred_alert_limit_clearance_step_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_clearance_step_m', default_value=p5_pred_alert_limit_clearance_step_m)
    p5_pred_alert_limit_drone_radius_m_arg = DeclareLaunchArgument('p5_pred_alert_limit_drone_radius_m', default_value=p5_pred_alert_limit_drone_radius_m)
    p5_pred_alert_limit_clearance_scale_arg = DeclareLaunchArgument('p5_pred_alert_limit_clearance_scale', default_value=p5_pred_alert_limit_clearance_scale)
    p5_pred_alert_limit_vertical_scale_arg = DeclareLaunchArgument('p5_pred_alert_limit_vertical_scale', default_value=p5_pred_alert_limit_vertical_scale)
    p5_debug_metrics_enable_arg = DeclareLaunchArgument('p5_debug_metrics_enable', default_value=p5_debug_metrics_enable)
    p5_status_topic_arg = DeclareLaunchArgument('p5_status_topic', default_value=p5_status_topic)
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    # Ego Planner Node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', odometry_topic),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            
            ('grid_map/odom', odometry_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/pose', camera_pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
        parameters=[
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            
            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},
            {'fsm/waypoint5_x': point5_x},
            {'fsm/waypoint5_y': point5_y},
            {'fsm/waypoint5_z': point5_z},
            {'fsm/waypoint6_x': point6_x},
            {'fsm/waypoint6_y': point6_y},
            {'fsm/waypoint6_z': point6_z},
            
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.099},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            # camera parameter
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            # depth filter
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # local fusion
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},
            
            {'grid_map/virtual_ceil_height': 2.9},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': "map"},
            {'p0.enable_risk_grid': p0_enable_risk_grid},
            {'p0.resolution_m': p0_resolution_m},
            {'p0.size_x_m': p0_size_x_m},
            {'p0.size_y_m': p0_size_y_m},
            {'p0.size_z_m': p0_size_z_m},
            {'p0.horizons_s': [0.0, 0.5, 1.0, 1.5, 2.0]},
            {'p0.refresh_period_s': p0_refresh_period_s},
            {'p0.stale_timeout_s': p0_stale_timeout_s},
            {'p0.skip_occupied_voxels': p0_skip_occupied_voxels},
            {'p0.debug_metrics_enable': p0_debug_metrics_enable},
            {'p0.odom_topic': odometry_topic},
            {'p0.integrity_topic': '/iap/integrity'},
            {'p0.range_meas_topic': '/ublox_driver/range_meas'},
            {'p0.ephem_topic': '/ublox_driver/ephem'},
            {'p0.glo_ephem_topic': '/ublox_driver/glo_ephem'},
            {'p0.receiver_lla_topic': '/ublox_driver/receiver_lla'},
            {'p0.iono_topic': '/ublox_driver/iono_params'},
            {'p0.map_topic': '/map_generator/global_cloud'},
            {'p0.health_topic': p0_health_topic},
            {'p5.enable_runtime_gate': p5_enable_runtime_gate},
            {'p5.enable_final_gate': p5_enable_final_gate},
            {'p5.horizon_s': p5_horizon_s},
            {'p5.sample_dt_s': p5_sample_dt_s},
            {'p5.current_stale_to_replan_s': p5_current_stale_to_replan_s},
            {'p5.current_stale_to_emergency_s': p5_current_stale_to_emergency_s},
            {'p5.current_low_margin_to_emergency_s': p5_current_low_margin_to_emergency_s},
            {'p5.future_unknown_to_emergency_s': p5_future_unknown_to_emergency_s},
            {'p5.final_gate_max_consecutive_failures': p5_final_gate_max_consecutive_failures},
            {'p5.final_gate_max_failure_duration_s': p5_final_gate_max_failure_duration_s},
            {'p5.current_replan_margin_m': p5_current_replan_margin_m},
            {'p5.current_emergency_margin_m': p5_current_emergency_margin_m},
            {'p5.future_replan_margin_m': p5_future_replan_margin_m},
            {'p5.future_emergency_margin_m': p5_future_emergency_margin_m},
            {'p5.max_bad_ratio': p5_max_bad_ratio},
            {'p5.max_unknown_ratio': p5_max_unknown_ratio},
            {'p5.bad_tick_to_replan': p5_bad_tick_to_replan},
            {'p5.good_tick_to_clear': p5_good_tick_to_clear},
            {'p5.pred_alert_limit_mode': p5_pred_alert_limit_mode},
            {'p5.pred_alert_limit_constant_hal_m': p5_pred_alert_limit_constant_hal_m},
            {'p5.pred_alert_limit_constant_val_m': p5_pred_alert_limit_constant_val_m},
            {'p5.pred_alert_limit_min_hal_m': p5_pred_alert_limit_min_hal_m},
            {'p5.pred_alert_limit_max_hal_m': p5_pred_alert_limit_max_hal_m},
            {'p5.pred_alert_limit_min_val_m': p5_pred_alert_limit_min_val_m},
            {'p5.pred_alert_limit_max_val_m': p5_pred_alert_limit_max_val_m},
            {'p5.pred_alert_limit_clearance_search_radius_m': p5_pred_alert_limit_clearance_search_radius_m},
            {'p5.pred_alert_limit_clearance_step_m': p5_pred_alert_limit_clearance_step_m},
            {'p5.pred_alert_limit_drone_radius_m': p5_pred_alert_limit_drone_radius_m},
            {'p5.pred_alert_limit_clearance_scale': p5_pred_alert_limit_clearance_scale},
            {'p5.pred_alert_limit_vertical_scale': p5_pred_alert_limit_vertical_scale},
            {'p5.integrity_topic': '/iap/integrity'},
            {'p5.status_topic': p5_status_topic},
            {'p5.debug_metrics_enable': p5_debug_metrics_enable},
            {'p1.use_integrity_cost': p1_use_integrity_cost},
            {'p1.metrics_only': p1_metrics_only},
            {'p1.lambda_integrity': p1_lambda_integrity},
            {'p1.sample_dt_min_s': p1_sample_dt_min_s},
            {'p1.sample_dt_scale': p1_sample_dt_scale},
            {'p1.max_samples_per_eval': p1_max_samples_per_eval},
            {'p1.integrity_cost_max': p1_integrity_cost_max},
            {'p1.integrity_grad_norm_max': p1_integrity_grad_norm_max},
            {'p1.unknown_policy': p1_unknown_policy},
            {'p1.unknown_soft_penalty': p1_unknown_soft_penalty},
            {'p1.debug_csv_enable': p1_debug_csv_enable},
            {'p1.debug_csv_path': p1_debug_csv_path},
            {'p2.enable_candidate_ranking': p2_enable_candidate_ranking},
            {'p2.metrics_only': p2_metrics_only},
            {'p2.sample_dt_s': p2_sample_dt_s},
            {'p2.lambda_candidate_integrity': p2_lambda_candidate_integrity},
            {'p2.w_max_cost': p2_w_max_cost},
            {'p2.w_unknown': p2_w_unknown},
            {'p2.w_stale': p2_w_stale},
            {'p2.min_valid_ratio': p2_min_valid_ratio},
            {'p2.debug_csv_enable': p2_debug_csv_enable},
            {'p2.debug_csv_path': p2_debug_csv_path},
            {'p3.enable_local_reference_bias': p3_enable_local_reference_bias},
            {'p3.enable_global_reference_bias': p3_enable_global_reference_bias},
            {'p3.local_bias_radius_m': p3_local_bias_radius_m},
            {'p3.min_improvement_ratio': p3_min_improvement_ratio},
            {'p3.w_risk': p3_w_risk},
            {'p3.w_detour': p3_w_detour},
            {'p3.w_unknown': p3_w_unknown},
            {'p3.min_corridor_valid_ratio': p3_min_corridor_valid_ratio},
            {'p3.station_spacing_m': p3_station_spacing_m},
            {'p3.lateral_sample_step_m': p3_lateral_sample_step_m},
            {'p3.lateral_sample_count_each_side': p3_lateral_sample_count_each_side},
            {'p3.beam_width': p3_beam_width},
            {'p3.max_detour_ratio': p3_max_detour_ratio},
            {'p3.debug_csv_enable': p3_debug_csv_enable},
            {'p3.debug_csv_path': p3_debug_csv_path},
            {'p4.enable_risk_aware_astar': p4_enable_risk_aware_astar},
            {'p4.lambda_p4_risk': p4_lambda_p4_risk},
            {'p4.risk_cost_max': p4_risk_cost_max},
            {'p4.unknown_edge_penalty': p4_unknown_edge_penalty},
            {'p4.max_extra_path_ratio': p4_max_extra_path_ratio},
            {'p4.fallback_to_original_when_risk_not_ready': p4_fallback_to_original_when_risk_not_ready},
            {'p4.debug_csv_enable': p4_debug_csv_enable},
            {'p4.debug_csv_path': p4_debug_csv_path},
            {'risk_overlay/enable': risk_overlay_enable},
            {'risk_overlay/use_for_astar': risk_overlay_use_for_astar},
            {'risk_overlay/use_for_bspline': risk_overlay_use_for_bspline},
            {'risk_overlay/topic': risk_overlay_topic},
            {'risk_overlay/lambda_unknown': risk_overlay_lambda_unknown},
            {'risk_overlay/lambda_stale': risk_overlay_lambda_stale},
            {'risk_overlay/stale_timeout_s': risk_overlay_stale_timeout_s},
            {'risk_overlay/stale_tau_s': risk_overlay_stale_tau_s},
            {'risk_overlay/r_soft': risk_overlay_r_soft},
            {'risk_overlay/w_soft': risk_overlay_w_soft},
            {'risk_overlay/w_hard': risk_overlay_w_hard},
            {'risk_overlay/c_unsafe': risk_overlay_c_unsafe},
            {'risk_overlay/eps_al_m': risk_overlay_eps_al_m},
            {'risk_overlay/gamma_h': risk_overlay_gamma_h},
            {'risk_overlay/gamma_v': risk_overlay_gamma_v},
            {'risk_overlay/drone_radius_m': risk_overlay_drone_radius_m},
            {'risk_overlay/safety_buffer_m': risk_overlay_safety_buffer_m},
            {'risk_overlay/clearance_max_m': risk_overlay_clearance_max_m},
            {'risk_overlay/clearance_unknown_m': risk_overlay_clearance_unknown_m},
            {'risk_overlay/edge_sample_alpha': risk_overlay_edge_sample_alpha},
            {'risk_overlay/debug_publish': risk_overlay_debug_publish},
            {'risk_overlay/debug_topic': risk_overlay_debug_topic},
            {'risk_overlay/debug_publish_hz': risk_overlay_debug_publish_hz},
            {'risk_overlay/debug_color_mode': risk_overlay_debug_color_mode},
            {'risk_overlay/debug_cost_max': risk_overlay_debug_cost_max},
            # planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': drone_id},
            {'manager/use_integrity_global_search': use_integrity_global_search},
            {'manager/integrity_global_astar_step_m': integrity_global_astar_step_m},
            {'manager/integrity_global_max_waypoints': integrity_global_max_waypoints},
            # Trajectory optimization parameters
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            {'optimization/use_integrity_cost': use_integrity_cost},
            {'optimization/lambda_integrity': lambda_integrity},
            {'optimization/integrity_cost_topic': '/iap/integrity_cost_field'},
            {'optimization/integrity_debug_csv_path': integrity_debug_csv_path},
            {'optimization/integrity_field_stale_timeout_s': integrity_field_stale_timeout_s},
            {'optimization/integrity_nearest_radius_m': integrity_nearest_radius_m},
            {'optimization/integrity_cost_max': integrity_cost_max},
            {'optimization/integrity_grad_norm_max': integrity_grad_norm_max},
            {'optimization/integrity_min_samples': 3},
            {'optimization/use_integrity_front_search': use_integrity_front_search},
            {'optimization/use_integrity_global_search': use_integrity_global_search},
            {'optimization/lambda_integrity_front': lambda_integrity_front},
            {'optimization/integrity_front_cost_topic': integrity_front_cost_topic},
            {'optimization/integrity_front_nearest_radius_m': integrity_front_nearest_radius_m},
            {'optimization/integrity_front_stale_timeout_s': integrity_front_stale_timeout_s},
            {'optimization/integrity_front_cost_max': integrity_front_cost_max},
            {'risk_overlay/bspline_samples_per_segment': risk_overlay_bspline_samples_per_segment},

            # B-Spline parameters
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # Object prediction parameters
            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    
    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    ld.add_action(point5_x_arg)
    ld.add_action(point5_y_arg)
    ld.add_action(point5_z_arg)
    ld.add_action(point6_x_arg)
    ld.add_action(point6_y_arg)
    ld.add_action(point6_z_arg)
    
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(use_integrity_cost_arg)
    ld.add_action(integrity_debug_csv_path_arg)
    ld.add_action(lambda_integrity_arg)
    ld.add_action(p1_use_integrity_cost_arg)
    ld.add_action(p1_metrics_only_arg)
    ld.add_action(p1_lambda_integrity_arg)
    ld.add_action(p1_sample_dt_min_s_arg)
    ld.add_action(p1_sample_dt_scale_arg)
    ld.add_action(p1_max_samples_per_eval_arg)
    ld.add_action(p1_integrity_cost_max_arg)
    ld.add_action(p1_integrity_grad_norm_max_arg)
    ld.add_action(p1_unknown_policy_arg)
    ld.add_action(p1_unknown_soft_penalty_arg)
    ld.add_action(p1_debug_csv_enable_arg)
    ld.add_action(p1_debug_csv_path_arg)
    ld.add_action(p2_enable_candidate_ranking_arg)
    ld.add_action(p2_metrics_only_arg)
    ld.add_action(p2_sample_dt_s_arg)
    ld.add_action(p2_lambda_candidate_integrity_arg)
    ld.add_action(p2_w_max_cost_arg)
    ld.add_action(p2_w_unknown_arg)
    ld.add_action(p2_w_stale_arg)
    ld.add_action(p2_min_valid_ratio_arg)
    ld.add_action(p2_debug_csv_enable_arg)
    ld.add_action(p2_debug_csv_path_arg)
    ld.add_action(p3_enable_local_reference_bias_arg)
    ld.add_action(p3_enable_global_reference_bias_arg)
    ld.add_action(p3_local_bias_radius_m_arg)
    ld.add_action(p3_min_improvement_ratio_arg)
    ld.add_action(p3_w_risk_arg)
    ld.add_action(p3_w_detour_arg)
    ld.add_action(p3_w_unknown_arg)
    ld.add_action(p3_min_corridor_valid_ratio_arg)
    ld.add_action(p3_station_spacing_m_arg)
    ld.add_action(p3_lateral_sample_step_m_arg)
    ld.add_action(p3_lateral_sample_count_each_side_arg)
    ld.add_action(p3_beam_width_arg)
    ld.add_action(p3_max_detour_ratio_arg)
    ld.add_action(p3_debug_csv_enable_arg)
    ld.add_action(p3_debug_csv_path_arg)
    ld.add_action(p4_enable_risk_aware_astar_arg)
    ld.add_action(p4_lambda_p4_risk_arg)
    ld.add_action(p4_risk_cost_max_arg)
    ld.add_action(p4_unknown_edge_penalty_arg)
    ld.add_action(p4_max_extra_path_ratio_arg)
    ld.add_action(p4_fallback_to_original_when_risk_not_ready_arg)
    ld.add_action(p4_debug_csv_enable_arg)
    ld.add_action(p4_debug_csv_path_arg)
    ld.add_action(integrity_field_stale_timeout_s_arg)
    ld.add_action(integrity_nearest_radius_m_arg)
    ld.add_action(integrity_cost_max_arg)
    ld.add_action(integrity_grad_norm_max_arg)
    ld.add_action(use_integrity_front_search_arg)
    ld.add_action(use_integrity_global_search_arg)
    ld.add_action(lambda_integrity_front_arg)
    ld.add_action(integrity_front_cost_topic_arg)
    ld.add_action(integrity_front_nearest_radius_m_arg)
    ld.add_action(integrity_front_stale_timeout_s_arg)
    ld.add_action(integrity_front_cost_max_arg)
    ld.add_action(risk_overlay_enable_arg)
    ld.add_action(risk_overlay_use_for_astar_arg)
    ld.add_action(risk_overlay_use_for_bspline_arg)
    ld.add_action(risk_overlay_topic_arg)
    ld.add_action(risk_overlay_lambda_unknown_arg)
    ld.add_action(risk_overlay_lambda_stale_arg)
    ld.add_action(risk_overlay_stale_timeout_s_arg)
    ld.add_action(risk_overlay_stale_tau_s_arg)
    ld.add_action(risk_overlay_r_soft_arg)
    ld.add_action(risk_overlay_w_soft_arg)
    ld.add_action(risk_overlay_w_hard_arg)
    ld.add_action(risk_overlay_c_unsafe_arg)
    ld.add_action(risk_overlay_eps_al_m_arg)
    ld.add_action(risk_overlay_gamma_h_arg)
    ld.add_action(risk_overlay_gamma_v_arg)
    ld.add_action(risk_overlay_drone_radius_m_arg)
    ld.add_action(risk_overlay_safety_buffer_m_arg)
    ld.add_action(risk_overlay_clearance_max_m_arg)
    ld.add_action(risk_overlay_clearance_unknown_m_arg)
    ld.add_action(risk_overlay_edge_sample_alpha_arg)
    ld.add_action(risk_overlay_bspline_samples_per_segment_arg)
    ld.add_action(risk_overlay_debug_publish_arg)
    ld.add_action(risk_overlay_debug_topic_arg)
    ld.add_action(risk_overlay_debug_publish_hz_arg)
    ld.add_action(risk_overlay_debug_color_mode_arg)
    ld.add_action(risk_overlay_debug_cost_max_arg)
    ld.add_action(integrity_global_astar_step_m_arg)
    ld.add_action(integrity_global_max_waypoints_arg)
    ld.add_action(p0_enable_risk_grid_arg)
    ld.add_action(p0_resolution_m_arg)
    ld.add_action(p0_size_x_m_arg)
    ld.add_action(p0_size_y_m_arg)
    ld.add_action(p0_size_z_m_arg)
    ld.add_action(p0_refresh_period_s_arg)
    ld.add_action(p0_stale_timeout_s_arg)
    ld.add_action(p0_skip_occupied_voxels_arg)
    ld.add_action(p0_debug_metrics_enable_arg)
    ld.add_action(p0_health_topic_arg)
    ld.add_action(p5_enable_runtime_gate_arg)
    ld.add_action(p5_enable_final_gate_arg)
    ld.add_action(p5_horizon_s_arg)
    ld.add_action(p5_sample_dt_s_arg)
    ld.add_action(p5_current_stale_to_replan_s_arg)
    ld.add_action(p5_current_stale_to_emergency_s_arg)
    ld.add_action(p5_current_low_margin_to_emergency_s_arg)
    ld.add_action(p5_future_unknown_to_emergency_s_arg)
    ld.add_action(p5_final_gate_max_consecutive_failures_arg)
    ld.add_action(p5_final_gate_max_failure_duration_s_arg)
    ld.add_action(p5_current_replan_margin_m_arg)
    ld.add_action(p5_current_emergency_margin_m_arg)
    ld.add_action(p5_future_replan_margin_m_arg)
    ld.add_action(p5_future_emergency_margin_m_arg)
    ld.add_action(p5_max_bad_ratio_arg)
    ld.add_action(p5_max_unknown_ratio_arg)
    ld.add_action(p5_bad_tick_to_replan_arg)
    ld.add_action(p5_good_tick_to_clear_arg)
    ld.add_action(p5_pred_alert_limit_mode_arg)
    ld.add_action(p5_pred_alert_limit_constant_hal_m_arg)
    ld.add_action(p5_pred_alert_limit_constant_val_m_arg)
    ld.add_action(p5_pred_alert_limit_min_hal_m_arg)
    ld.add_action(p5_pred_alert_limit_max_hal_m_arg)
    ld.add_action(p5_pred_alert_limit_min_val_m_arg)
    ld.add_action(p5_pred_alert_limit_max_val_m_arg)
    ld.add_action(p5_pred_alert_limit_clearance_search_radius_m_arg)
    ld.add_action(p5_pred_alert_limit_clearance_step_m_arg)
    ld.add_action(p5_pred_alert_limit_drone_radius_m_arg)
    ld.add_action(p5_pred_alert_limit_clearance_scale_arg)
    ld.add_action(p5_pred_alert_limit_vertical_scale_arg)
    ld.add_action(p5_debug_metrics_enable_arg)
    ld.add_action(p5_status_topic_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)


    # Add Node
    ld.add_action(ego_planner_node)

    return ld
