# IAP Topology Graph v0.2 - Function Cards (L0-L3)

Status: v0.2 (L3 anchored + GNSS/Viewer local variables completed)

## Card T0-M1

| Field | Content |
|---|---|
| Function | IapRosNode::create_core_subscriptions |
| Class | glim::IapRosNode |
| Lane | T0_Main |
| Source | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L254) |
| Trigger | Node construction phase |
| Downstream L1 | IMU callback lambda [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L260); PointCloud callback lambda [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L287) |
| Downstream L2 | validate_imu_stamp [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L266); preprocess [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L306) |
| Downstream L3 | AsyncOdometryEstimation::insert_imu [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L24); AsyncOdometryEstimation::insert_frame [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L30) |
| Input parameters | none |
| Return value | none |
| Read member variables | imu_topic_; points_topic_; imu_time_offset_; points_time_offset_; keep_raw_points_; intensity_field_; ring_field_ |
| Write member variables | imu_sub_; points_sub_ |
| Read key containers | preprocessor_; time_keeper_ |
| Write key containers | async_odom_ queue; async_sub_ queue; async_global_ queue |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| stamp | double | IMU and points timestamp after offset | YES |
| acc | Eigen::Vector3d | Linear acceleration sample | YES |
| gyro | Eigen::Vector3d | Angular velocity sample | YES |
| raw_points | PointCloud::Ptr | Input cloud decoded from PointCloud2 | YES |
| preprocessed | PreprocessedFrame::Ptr | Odom ingress payload | YES |

## Card T1-O0

| Field | Content |
|---|---|
| Function | AsyncOdometryEstimation::run |
| Class | glim::AsyncOdometryEstimation |
| Lane | T1_Odom |
| Source | [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L55) |
| Trigger | Dedicated thread from AsyncOdometryEstimation constructor |
| Downstream L1 | input queue drains [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L63); insert_imu [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L88); insert_frame [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L126) |
| Downstream L2 | OdometryEstimationIMU::insert_frame [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L168) |
| Downstream L3 | create_factors CPU/GPU [src/iap/src/iap/odometry/odometry_estimation_cpu.cpp](src/iap/src/iap/odometry/odometry_estimation_cpu.cpp#L103), [src/iap/src/iap/odometry/odometry_estimation_gpu.cpp](src/iap/src/iap/odometry/odometry_estimation_gpu.cpp#L267); update_smoother [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L601); update_frames [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L491) |
| Input parameters | none |
| Return value | none |
| Read member variables | enable_imu; end_of_sequence; kill_switch |
| Write member variables | internal_frame_queue_size |
| Read key containers | input_imu_queue; input_frame_queue |
| Write key containers | output_estimation_results; output_marginalized_frames |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| last_imu_time | double | Time gate for frame processing | YES |
| imu_frames | vector | Batch of IMU packets from queue drain | NO |
| new_raw_frames | vector | Fresh frames from queue drain | NO |
| raw_frames | deque | Internal pending frame backlog | YES |
| frame | PreprocessedFrame::Ptr | Current frame passed to odometry core | YES |
| marginalized | vector | Per-frame marginalized output | YES |

## Card T1-O1

| Field | Content |
|---|---|
| Function | OdometryEstimationIMU::insert_frame |
| Class | glim::OdometryEstimationIMU |
| Lane | T1_Odom |
| Source | [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L168) |
| Trigger | Called from AsyncOdometryEstimation::run |
| Downstream L1 | create_factors; on_smoother_update callback; update_smoother; update_frames |
| Downstream L2 | create_factors CPU/GPU [src/iap/src/iap/odometry/odometry_estimation_cpu.cpp](src/iap/src/iap/odometry/odometry_estimation_cpu.cpp#L103), [src/iap/src/iap/odometry/odometry_estimation_gpu.cpp](src/iap/src/iap/odometry/odometry_estimation_gpu.cpp#L267) |
| Downstream L3 | callback fanout to GNSS/Viewer [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L428), [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L461) |
| Input parameters | raw_frame; marginalized_frames output vector |
| Return value | EstimationFrame::ConstPtr |
| Read member variables | frames; smoother; imu_integration; deskewing; covariance_estimation; odometry_owns_clock_ |
| Write member variables | frames; marginalized_cursor |
| Read key containers | smoother estimate; active frame window |
| Write key containers | new_factors; new_values; new_stamps |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| current / last | int | Sliding-window indices | YES |
| last_stamp | double | Prior frame timestamp for integration | YES |
| num_imu_integrated | int | IMU availability gate | YES |
| imu_factor | shared_ptr<gtsam::ImuFactor> | Optional IMU factor edge | YES |
| predicted_clk | gtsam::Vector2 | Clock prediction for C(current) | YES |
| new_factors | gtsam::NonlinearFactorGraph | Batch factors for this frame | YES |
| new_values | gtsam::Values | Batch values for this frame | YES |
| new_stamps | gtsam::FixedLagSmootherKeyTimestampMap | Keepalive timestamps in fixed-lag smoother | YES |

## Card T2-B0

| Field | Content |
|---|---|
| Function | IapRosNode::queue_bridge_loop |
| Class | glim::IapRosNode |
| Lane | T2_Bridge |
| Source | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L389) |
| Trigger | Dedicated thread from IapRosNode constructor |
| Downstream L1 | async_odom_->get_results; async_sub_->insert_frame; async_sub_->get_results; async_global_->insert_submap |
| Downstream L2 | get_all_and_clear queue drains in AsyncOdometryEstimation and AsyncSubMapping |
| Downstream L3 | cross-thread handoff cycles every 5 ms [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L408) |
| Input parameters | none |
| Return value | none |
| Read member variables | queue_thread_stop_ |
| Write member variables | none |
| Read key containers | async_odom_ outputs; async_sub_ outputs |
| Write key containers | async_sub_ input_frame_queue; async_global_ input_submap_queue |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| estimated | vector<EstimationFrame::ConstPtr> | Pulled from odometry output queue | NO |
| marginalized | vector<EstimationFrame::ConstPtr> | Forwarded to sub-mapping lane | YES |
| submaps | vector<SubMap::Ptr> | Forwarded to global-mapping lane | YES |

## Card T3-S0

| Field | Content |
|---|---|
| Function | AsyncSubMapping::run |
| Class | glim::AsyncSubMapping |
| Lane | T3_Sub |
| Source | [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L47) |
| Trigger | Dedicated thread from AsyncSubMapping constructor |
| Downstream L1 | sub_mapping->get_submaps; insert_imu; insert_frame |
| Downstream L2 | SubMapping::insert_frame [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L104) |
| Downstream L3 | insert_keyframe [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L339); create_submap [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L415); submit_end_of_sequence [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L509) |
| Input parameters | none |
| Return value | none |
| Read member variables | kill_switch; end_of_sequence |
| Write member variables | none |
| Read key containers | input_imu_queue; input_frame_queue |
| Write key containers | output_submap_queue |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| submaps | vector<SubMap::Ptr> | Fresh output from sub_mapping->get_submaps() | YES |
| imu_frames | vector | Drained IMU batch | NO |
| odom_frames | vector<EstimationFrame::ConstPtr> | Drained marginalized frames | YES |
| frame | EstimationFrame::ConstPtr | Current odom frame insertion unit | YES |

## Card T4-G0

| Field | Content |
|---|---|
| Function | AsyncGlobalMapping::run |
| Class | glim::AsyncGlobalMapping |
| Lane | T4_Global |
| Source | [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L77) |
| Trigger | Dedicated thread from AsyncGlobalMapping constructor |
| Downstream L1 | global_mapping->insert_imu; insert_submap; optimize; recover_graph; find_overlapping_submaps |
| Downstream L2 | GlobalMapping::insert_submap [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L134) |
| Downstream L3 | create_between_factors [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L391); create_matching_cost_factors [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L442); update_isam2 [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L509) |
| Input parameters | none |
| Return value | none |
| Read member variables | request_to_optimize; request_to_recover; request_to_find_overlapping_submaps |
| Write member variables | request flags via exchange/reset |
| Read key containers | input_submap_queue; input_imu_queue |
| Write key containers | global_mapping internal graph under global_mapping_mutex |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| last_optimization_time | time_point | Idle optimization trigger baseline | YES |
| min_overlap | double | Pending overlap-request value | YES |
| imu_frames | vector | Batch of IMU packets | NO |
| submaps | vector<SubMap::Ptr> | Batch of incoming submaps | YES |

## Card T5-N0

| Field | Content |
|---|---|
| Function | GnssExtensionModule::on_smoother_update_ |
| Class | iap::GnssExtensionModule |
| Lane | T5_GNSS |
| Source | [src/iap/src/iap/gnss/gnss_extension.cpp](src/iap/src/iap/gnss/gnss_extension.cpp#L535) |
| Trigger | OdometryEstimationCallbacks::on_smoother_update |
| Downstream L1 | GnssHandler::get_factors; E(0)/R(0) insertion; ensure C(frame_id); ClockBetweenFactor chain |
| Downstream L2 | GnssHandler::get_factors [src/iap/src/iap/gnss/gnss_handler.cpp](src/iap/src/iap/gnss/gnss_handler.cpp#L46) |
| Downstream L3 | PseudorangeFactor creation [src/iap/src/iap/gnss/gnss_handler.cpp](src/iap/src/iap/gnss/gnss_handler.cpp#L74); DopplerFactor creation [src/iap/src/iap/gnss/gnss_handler.cpp](src/iap/src/iap/gnss/gnss_handler.cpp#L90) |
| Input parameters | smoother; new_factors; new_values; new_stamps |
| Return value | none |
| Read member variables | origin_set_; origin_ecef_; R_ecef_world_init_; prev_gnss_frame_id_; prev_gnss_frame_stamp_; gnss_owns_clock_ |
| Write member variables | prev_gnss_frame_id_; prev_gnss_frame_stamp_; ext_vars_inserted_; clock_chain_state_ |
| Read key containers | gnss_handler epoch queue; ephemeris caches; factor snapshots |
| Write key containers | new_factors; new_values; new_stamps; last_pr_factors_; last_dop_factors_ |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| frame_id | long | Current graph frame index for X(i)/V(i)/C(i) keys | YES |
| frame_stamp | double | Current frame time for epoch matching and clock dt | YES |
| anc_ecef | Eigen::Vector3d | Current ECEF anchor snapshot | YES |
| R_seed | Eigen::Matrix3d | Initial world->ECEF rotation snapshot | YES |
| consumed | vector<GnssEpoch> | Epochs consumed in this injection pass | NO |
| gnss_factors | gtsam::NonlinearFactorGraph | Newly built GNSS factor batch | YES |
| init_clk | gtsam::Vector2 | Warm-start clock state when C(frame_id) absent | YES |
| dt | double | Clock between-factor interval | YES |
| prev_clk_key | gtsam::Key | Previous C(prev) key used for chain continuity | YES |
| prev_clock_available | bool | Gate for ClockBetweenFactor insertion | YES |

## Card T5-N1

| Field | Content |
|---|---|
| Function | GnssExtensionModule::on_smoother_update_finish_ |
| Class | iap::GnssExtensionModule |
| Lane | T5_GNSS |
| Source | [src/iap/src/iap/gnss/gnss_extension.cpp](src/iap/src/iap/gnss/gnss_extension.cpp#L776) |
| Trigger | OdometryEstimationCallbacks::on_smoother_update_finish |
| Downstream L1 | smoother.calculateEstimate(C(frame_id)); residual loop over last_pr_factors_/last_dop_factors_ |
| Downstream L2 | NoiseModelFactor::unwhitenedError |
| Downstream L3 | debug CSV write of per-factor diagnostics |
| Input parameters | smoother |
| Return value | none |
| Read member variables | last_pr_factors_; last_dop_factors_; last_injected_frame_id_; debug_csv_enabled_ |
| Write member variables | last_clk_bias_; last_clk_drift_; last_clk_stamp_; clock_chain_state_; factor_count_diag_ |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| pr_factors / dop_factors | vector<NonlinearFactor::shared_ptr> | Snapshot of factors to evaluate post-opt | YES |
| frame_id | long | Frame index whose C(frame_id) is read post-opt | YES |
| clk_bias / clk_drift | double | Estimated clock state pulled from smoother | YES |
| clk_ok | bool | Guard for post-opt clock availability | YES |
| pr_rms / dop_rms | double | RMS diagnostic accumulators | NO |
| n_pr_ok / n_dop_ok | int | Number of valid residuals | NO |
| details | vector<FactorDetail> | Optional per-factor CSV payload rows | NO |

## Card T6-V0

| Field | Content |
|---|---|
| Function | RvizViewer::odometry_new_frame |
| Class | glim::RvizViewer |
| Lane | T6_Viewer |
| Source | [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L104) |
| Trigger | on_new_frame or on_update_new_frame callback |
| Downstream L1 | trajectory->add_odom/odom2world; TF publish; odom/pose publish |
| Downstream L2 | tf_buffer->lookupTransform; frame_to_pointcloud2 |
| Downstream L3 | invoke queue drain happens in spin_once [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L441) |
| Input parameters | new_frame; corrected |
| Return value | none |
| Read member variables | imu_frame_id; lidar_frame_id; base_frame_id; odom_frame_id; map_frame_id; tf_time_offset |
| Write member variables | trajectory; invoke_queue |
| Read key containers | new_frame->frame; new_frame->imu_rate_trajectory |
| Write key containers | ROS publishers; TF broadcaster |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| T_odom_imu | Eigen::Isometry3d | Odom pose for current frame | YES |
| quat_odom_imu | Eigen::Quaterniond | ROS-friendly orientation for odom pose | YES |
| v_odom_imu | Eigen::Vector3d | Velocity output in odom frame | YES |
| T_lidar_imu | Eigen::Isometry3d | Sensor extrinsic for IMU->LiDAR TF publish | YES |
| imu_end_time | double | Scan-end time fallback and publish stamp | YES |
| T_imubegin_imuend | Eigen::Isometry3d | Intra-scan delta for scan-end odom/pose | YES |
| T_world_odom | Eigen::Isometry3d | World->odom anchor from trajectory manager | YES |
| T_world_imu | Eigen::Isometry3d | World pose used for map-frame pose outputs | YES |
| stamp / tf_stamp / imu_end_stamp | rclcpp::Time | Timestamp set for regular TF and scan-end channels | YES |
| transformed | vector<Eigen::Vector4d> | Temporarily aligned points for map-frame publish | NO |

## Card T6-V1

| Field | Content |
|---|---|
| Function | RvizViewer::globalmap_on_update_submaps |
| Class | glim::RvizViewer |
| Lane | T6_Viewer |
| Source | [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L384) |
| Trigger | GlobalMappingCallbacks::on_update_submaps |
| Downstream L1 | trajectory->update_anchor; invoke |
| Downstream L2 | invoke queue closure merges map clouds |
| Downstream L3 | frame_to_pointcloud2 + map_pub->publish in invoked task |
| Input parameters | submaps |
| Return value | none |
| Read member variables | last_globalmap_pub_time; map_pub |
| Write member variables | submaps vector; last_globalmap_pub_time |

Local variables listed:

| Local variable | Type | Purpose | Key branch variable |
|---|---|---|---|
| latest_submap | SubMap::ConstPtr | Latest anchor source for trajectory update | YES |
| stamp_endpoint_R | double | Right endpoint stamp of newest submap | YES |
| T_world_endpoint_R | Eigen::Isometry3d | World endpoint pose used by trajectory anchor | YES |
| submap_poses | vector<Eigen::Isometry3d> | Per-submap pose list for global map merge | YES |
| total_num_points | int | Merge output size planning | NO |
| merged | PointCloudCPU::Ptr | Merged map point cloud buffer | YES |
| begin | int | Running copy offset while concatenating submaps | NO |
