#include <gtest/gtest.h>
#include <rcl/time.h>

// Exercise the actual command callback, queue and wire message. The process
// entrypoint is renamed only to let gtest own this synthetic-clock process.
#define main iap_traj_server_process_main
#include "../src/traj_server.cpp"
#undef main

TEST(TrajectoryServerTime, ActivationEvaluationAndStampUseOneCapturedTime) {
  if(!rclcpp::ok()) rclcpp::init(0,nullptr);
  server_node=rclcpp::Node::make_shared("traj_server_time_test");
  command_frame="map";
  pos_cmd_pub=server_node->create_publisher<quadrotor_msgs::msg::PositionCommand>("time_command_test",10);
  trajectory_curve_pub=server_node->create_publisher<visualization_msgs::msg::Marker>("time_curve_test",10);
  auto* clock=server_node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,100000000000LL),RCL_RET_OK);
  const auto curve=[&](int id,double start,double position,uint8_t mode) {
    auto message=std::make_shared<traj_utils::msg::Bspline>();
    message->order=3;message->traj_id=id;message->start_mode=mode;
    message->predecessor_id=receive_traj_ ? traj_id_ : -1;
    message->start_time=rclcpp::Time(static_cast<int64_t>(start*1e9));
    for(int i=0;i<11;++i) message->knots.push_back(i-3.);
    for(int i=0;i<7;++i) {
      geometry_msgs::msg::Point point;point.x=position+i-1.;point.z=1.;
      message->pos_pts.push_back(point);
    }
    return message;
  };
  last_yaw_=last_yaw_dot_=0.;time_forward_=.5;
  bsplineCallback(curve(1,100.,0.,traj_utils::msg::Bspline::IMMEDIATE));
  bsplineCallback(curve(2,102.,2.,traj_utils::msg::Bspline::AT_TIME));
  ASSERT_TRUE(pending_traj);
  // Time advances after the command tick was sampled, crossing activation.
  // An old ID stamped 103 would falsely acknowledge cancellation of ID2.
  ASSERT_EQ(rcl_set_ros_time_override(clock,103000000000LL),RCL_RET_OK);
  cmdCallbackAt(rclcpp::Time(101500000000LL,RCL_ROS_TIME));
  EXPECT_EQ(cmd.trajectory_id,1);
  EXPECT_EQ(rclcpp::Time(cmd.header.stamp).nanoseconds(),101500000000LL);
  EXPECT_NEAR(cmd.position.x,1.5,1e-12);
  EXPECT_TRUE(pending_traj);
  EXPECT_TRUE(std::isfinite(cmd.yaw));
  EXPECT_TRUE(std::isfinite(cmd.yaw_dot));
  auto cancellation=std::make_shared<traj_utils::msg::Bspline>();
  cancellation->start_mode=traj_utils::msg::Bspline::CANCEL_PENDING;
  cancellation->traj_id=999;bsplineCallback(cancellation);
  ASSERT_TRUE(pending_traj);EXPECT_EQ(pending_traj->id,2);
  cmdCallbackAt(rclcpp::Time(103000000000LL,RCL_ROS_TIME));
  EXPECT_EQ(cmd.trajectory_id,2);
  EXPECT_FALSE(pending_traj);
  EXPECT_EQ(rclcpp::Time(cmd.header.stamp).nanoseconds(),103000000000LL);
  EXPECT_NEAR(cmd.position.x,3.,1e-12);
  EXPECT_TRUE(std::isfinite(cmd.yaw_dot));
  cancellation->traj_id=2;bsplineCallback(cancellation);
  EXPECT_EQ(traj_id_,2);EXPECT_FALSE(pending_traj);
  cmdCallbackAt(rclcpp::Time(103100000000LL,RCL_ROS_TIME));
  EXPECT_EQ(cmd.trajectory_id,2); // A late cancellation cannot reverse activation.
  pending_traj.reset();pos_cmd_pub.reset();trajectory_curve_pub.reset();server_node.reset();
}

TEST(TrajectoryServerTime, ConfirmedStopIsAtomicallyReplacedWithoutEmptyQueue) {
  if(!rclcpp::ok()) rclcpp::init(0,nullptr);
  server_node=rclcpp::Node::make_shared("traj_server_replace_test");
  command_frame="map";receive_traj_=false;pending_traj.reset();highest_accepted_trajectory_id=-1;
  pos_cmd_pub=server_node->create_publisher<quadrotor_msgs::msg::PositionCommand>("replace_command_test",10);
  trajectory_curve_pub=server_node->create_publisher<visualization_msgs::msg::Marker>("replace_curve_test",10);
  auto* clock=server_node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock,200000000000LL),RCL_RET_OK);
  const auto curve=[](int id,double start,double position,uint8_t mode) {
    auto message=std::make_shared<traj_utils::msg::Bspline>();
    message->order=3;message->traj_id=id;message->start_mode=mode;
    message->predecessor_id=receive_traj_ ? traj_id_ : -1;
    message->start_time=rclcpp::Time(static_cast<int64_t>(start*1e9));
    for(int i=0;i<11;++i) message->knots.push_back(i-3.);
    for(int i=0;i<7;++i) {
      geometry_msgs::msg::Point point;point.x=position+i-1.;point.z=1.;
      message->pos_pts.push_back(point);
    }
    return message;
  };
  bsplineCallback(curve(10,200,0,0));
  auto b=curve(11,202,2,1);
  // True resting endpoint, exact moving boundary at t=0.
  b->pos_pts[5]=b->pos_pts[4];b->pos_pts[6]=b->pos_pts[4];
  bsplineCallback(b);
  ASSERT_TRUE(pending_traj);ASSERT_EQ(pending_traj->id,11);
  auto c=curve(12,202,2,3); // Authorized REPLACE_PENDING wire mode.
  c->replace_pending_id=11;
  ASSERT_TRUE(last_feedback.accepted);ASSERT_EQ(last_feedback.pending_id,11);
  const auto reject=[&](std::shared_ptr<traj_utils::msg::Bspline> request) {
    bsplineCallback(request);ASSERT_TRUE(pending_traj);EXPECT_EQ(pending_traj->id,11);
    EXPECT_FALSE(last_feedback.accepted);EXPECT_EQ(last_feedback.pending_id,11);
    EXPECT_EQ(last_feedback.active_id,10);
  };
  auto wrong=std::make_shared<traj_utils::msg::Bspline>(*c);wrong->predecessor_id=999;reject(wrong);
  wrong=std::make_shared<traj_utils::msg::Bspline>(*c);wrong->replace_pending_id=999;reject(wrong);
  wrong=std::make_shared<traj_utils::msg::Bspline>(*c);wrong->start_time=rclcpp::Time(203000000000LL);reject(wrong);
  wrong=std::make_shared<traj_utils::msg::Bspline>(*c);wrong->pos_pts[0].x+=.01;reject(wrong);
  wrong=std::make_shared<traj_utils::msg::Bspline>(*c);wrong->traj_id=11;reject(wrong);
  bsplineCallback(c);
  EXPECT_TRUE(last_feedback.accepted);EXPECT_EQ(last_feedback.reason,"PENDING_REPLACED");
  ASSERT_TRUE(pending_traj);
  EXPECT_EQ(pending_traj->id,12) << "A feasible normal C must replace confirmed B atomically";
  bsplineCallback(c); // Duplicate cannot consume/clear a pending replacement.
  ASSERT_TRUE(pending_traj);EXPECT_EQ(pending_traj->id,12);EXPECT_FALSE(last_feedback.accepted);
  auto reordered=std::make_shared<traj_utils::msg::Bspline>(*c);reordered->traj_id=13;
  bsplineCallback(reordered);ASSERT_TRUE(pending_traj);EXPECT_EQ(pending_traj->id,12);
  cmdCallbackAt(rclcpp::Time(202000000000LL,RCL_ROS_TIME));
  EXPECT_EQ(cmd.trajectory_id,12);
  EXPECT_NEAR(cmd.velocity.x,1.,1e-12);
  ASSERT_EQ(rcl_set_ros_time_override(clock,202100000000LL),RCL_RET_OK);
  bsplineCallback(reordered); // B already activated/replaced: no reinsertion.
  EXPECT_FALSE(last_feedback.accepted);EXPECT_FALSE(pending_traj);EXPECT_EQ(traj_id_,12);
  pending_traj.reset();pos_cmd_pub.reset();trajectory_curve_pub.reset();server_node.reset();
}
