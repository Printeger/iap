# IAP Topology Graph v0.2 - Cross Thread Handoffs

Status: v0.2 (payload schema + queue/backpressure + callback order completed)

## 1. Handoff Matrix

| Handoff ID | Data object | Producer function | Consumer function | Mechanism | Timing constraint | Source anchors |
|---|---|---|---|---|---|---|
| H01 | IMU sample (stamp, acc, gyro) | IMU callback in create_core_subscriptions | AsyncOdometryEstimation::run | AsyncOdometryEstimation::input_imu_queue | Odom frame gate waits for IMU coverage | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L260), [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L109) |
| H02 | PreprocessedFrame | PointCloud callback in create_core_subscriptions | AsyncOdometryEstimation::run | AsyncOdometryEstimation::input_frame_queue | Requires scan_end_time <= last_imu_time | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L287), [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L109) |
| H03 | EstimationFrame (estimated) | AsyncOdometryEstimation::run | queue_bridge_loop via get_results | output_estimation_results | polled at 5 ms bridge cadence | [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L129), [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L394) |
| H04 | EstimationFrame batch (marginalized) | AsyncOdometryEstimation::run | queue_bridge_loop via get_results | output_marginalized_frames | polled at 5 ms bridge cadence | [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L130), [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L394) |
| H05 | Marginalized EstimationFrame | queue_bridge_loop | AsyncSubMapping::run | AsyncSubMapping::input_frame_queue | bridge loop push every cycle | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L398), [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L56) |
| H06 | IMU sample for sub mapping | IMU callback in create_core_subscriptions | AsyncSubMapping::run | AsyncSubMapping::input_imu_queue | independent from odom queue drain | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L270), [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L55) |
| H07 | SubMap batch | AsyncSubMapping::run via SubMapping::get_submaps | queue_bridge_loop via get_results | AsyncSubMapping::output_submap_queue | polled at 5 ms bridge cadence | [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L49), [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L403) |
| H08 | SubMap | queue_bridge_loop | AsyncGlobalMapping::run | AsyncGlobalMapping::input_submap_queue | drained each run cycle, idle sleep 100 ms | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L406), [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L85) |
| H09 | IMU sample for global mapping | IMU callback in create_core_subscriptions | AsyncGlobalMapping::run | AsyncGlobalMapping::input_imu_queue | independent of submap arrival | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L273), [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L84) |
| H10 | Smoother update payload | OdometryEstimationIMU::insert_frame | GnssExtensionModule::on_smoother_update_ | OdometryEstimationCallbacks::on_smoother_update | callback fired before update_smoother apply | [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L428), [src/iap/src/iap/gnss/gnss_extension.cpp](src/iap/src/iap/gnss/gnss_extension.cpp#L535) |
| H11 | EstimationFrame for visualization | OdometryEstimationCallbacks::on_new_frame/on_update_new_frame | RvizViewer::odometry_new_frame | callback slot + viewer invoke queue | publish performed by viewer spin thread | [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L99), [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L436) |
| H12 | Submap update event | GlobalMapping::insert_submap/update_submaps | RvizViewer::globalmap_on_update_submaps | GlobalMappingCallbacks::on_update_submaps | triggered after global update | [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L238), [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L384) |

## 2. Payload Field-Level Schema (v0.2 completion)

### H03: EstimationFrame (estimated)

| Field | Type | Producer fill point | Consumer use |
|---|---|---|---|
| id | int | set in insert_frame [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L184) | viewer and downstream indexing |
| stamp | double | set in insert_frame [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L185) | time alignment and TF stamping |
| T_world_imu / T_world_lidar | Eigen::Isometry3d | predicted then corrected in insert_frame/update_frames [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L384), [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L524) | viewer pose and mapping relative transforms |
| v_world_imu | Eigen::Vector3d | set and updated in insert_frame/update_frames [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L386), [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L526) | odom twist publish and IMU chain |
| imu_bias | 6x1 vector | set and updated in insert_frame/update_frames [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L387), [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L527) | IMU continuation and diagnostics |
| frame | point cloud ptr | created in insert_frame [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L414) | viewer points and alignment publish |
| imu_rate_trajectory | 8xN matrix | optionally filled in insert_frame [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L393) | scan-end odom/pose extrapolation |

### H04: EstimationFrame batch (marginalized)

| Field | Type | Producer fill point | Consumer use |
|---|---|---|---|
| vector<EstimationFrame::ConstPtr> | batch container | built in while loop over marginalized_cursor [src/iap/src/iap/odometry/odometry_estimation_imu.cpp](src/iap/src/iap/odometry/odometry_estimation_imu.cpp#L435) | queue bridge forwards each frame to AsyncSubMapping |
| each frame payload | same as H03 frame core fields | same frame object as odom active window | sub-mapping delayed queue and submap graph construction |

### H07: SubMap batch

| Field | Type | Producer fill point | Consumer use |
|---|---|---|---|
| id | int | assigned when new submap enqueued [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L327) | global mapping submap index |
| T_world_origin | Eigen::Isometry3d | set in create_submap center pose [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L440) | global graph node initialization |
| T_origin_endpoint_L / R | Eigen::Isometry3d | set in create_submap [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L441) | between-submap initialization and IMU bridge factors |
| frames / odom_frames | vector<EstimationFrame::Ptr> | assigned in create_submap [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L444) | global mapping IMU endpoint states |
| frame | merged point cloud | built in create_submap merge phase [src/iap/src/iap/mapping/sub_mapping.cpp](src/iap/src/iap/mapping/sub_mapping.cpp#L478) | matching cost factors and viewer map output |

### H08: SubMap single item into global queue

| Field | Type | Producer fill point | Consumer use |
|---|---|---|---|
| SubMap::Ptr | pointer handoff | queue_bridge_loop inserts each submap [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L406) | AsyncGlobalMapping::run drains and calls insert_submap [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L123) |
| voxelmaps | vector<GaussianVoxelMap> | built/updated in GlobalMapping::insert_submap(current, submap) [src/iap/src/iap/mapping/global_mapping.cpp](src/iap/src/iap/mapping/global_mapping.cpp#L241) | used by create_matching_cost_factors |

## 3. Queue Growth and Backpressure Indicators (v0.2 completion)

| Queue | Observable size API | Producer side | Consumer side | Growth indicator | Backpressure behavior |
|---|---|---|---|---|---|
| AsyncOdometryEstimation::input_frame_queue | workload() = input_frame_queue.size() + internal_frame_queue_size [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L47) | points callback | AsyncOdometryEstimation::run | d(workload)/dt > 0 sustained | no hard cap by default (ConcurrentVector UNLIMITED policy) [src/iap/include/iap/util/concurrent_vector.hpp](src/iap/include/iap/util/concurrent_vector.hpp#L50) |
| AsyncSubMapping::input_frame_queue | workload() = input_frame_queue.size() [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L40) | queue bridge | AsyncSubMapping::run | queue size trend over bridge cycles | no hard cap unless policy override is wired in |
| AsyncGlobalMapping::input_submap_queue | workload() = input_submap_queue.size() [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L56) | queue bridge | AsyncGlobalMapping::run | queue size trend over global cycles | no hard cap unless policy override is wired in |
| GnssHandler::epoch_queue_ | queue_size() [src/iap/src/iap/gnss/gnss_handler.cpp](src/iap/src/iap/gnss/gnss_handler.cpp#L28) | on_range_meas_ | GnssHandler::get_factors | queue_size nearing max_epoch_queue | explicit cap: drop oldest epoch when full [src/iap/src/iap/gnss/gnss_handler.cpp](src/iap/src/iap/gnss/gnss_handler.cpp#L22) |

Indicator formulas used by ops docs:

1. Queue growth rate: growth_rate = (size_now - size_prev) / (t_now - t_prev)
2. Drain lag ratio: lag_ratio = producer_rate / max(consumer_rate, eps)
3. Saturation flag for bounded queues: saturated = (size_now >= max_size)

## 4. Callback Execution Order For Shared Slots (v0.2 completion)

| Rule | Evidence |
|---|---|
| Callback registration appends to slot vector and returns index | CallbackSlot::add push_back [src/iap/include/iap/util/callback_slot.hpp](src/iap/include/iap/util/callback_slot.hpp#L21) |
| Callback removal marks slot entry null, preserving relative order of remaining callbacks | CallbackSlot::remove [src/iap/include/iap/util/callback_slot.hpp](src/iap/include/iap/util/callback_slot.hpp#L30) |
| Callback execution iterates in vector order and skips null entries | CallbackSlot::call for-loop [src/iap/include/iap/util/callback_slot.hpp](src/iap/include/iap/util/callback_slot.hpp#L46) |

Execution-order implication for multi-extension hooks:

1. For the same slot (for example OdometryEstimationCallbacks::on_smoother_update), callback order is deterministic by registration order.
2. GNSS registers on_smoother_update in GnssExtensionModule ctor [src/iap/src/iap/gnss/gnss_extension.cpp](src/iap/src/iap/gnss/gnss_extension.cpp#L150).
3. Viewer registers on_new_frame/on_update_new_frame/on_update_submaps in set_callbacks [src/iap/src/iap/util/rviz_viewer.cpp](src/iap/src/iap/util/rviz_viewer.cpp#L97).
4. If two extensions attach to the same slot, whichever constructor/set_callbacks path runs first executes first.

## 5. Timing Gates

| Gate | Condition | Where enforced |
|---|---|---|
| Odom frame gate | raw_frame.scan_end_time <= last_imu_time | [src/iap/src/iap/odometry/async_odometry_estimation.cpp](src/iap/src/iap/odometry/async_odometry_estimation.cpp#L109) |
| Bridge polling cadence | fixed sleep 5 ms | [src/iap/apps/iap_rosnode.cpp](src/iap/apps/iap_rosnode.cpp#L408) |
| Sub thread idle cadence | fixed sleep 10 ms | [src/iap/src/iap/mapping/async_sub_mapping.cpp](src/iap/src/iap/mapping/async_sub_mapping.cpp#L62) |
| Global thread idle cadence | fixed sleep 100 ms | [src/iap/src/iap/mapping/async_global_mapping.cpp](src/iap/src/iap/mapping/async_global_mapping.cpp#L102) |
