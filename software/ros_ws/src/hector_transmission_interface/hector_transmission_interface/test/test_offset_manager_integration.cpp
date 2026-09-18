#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <vector>

#include "hector_testing_utils/hector_testing_utils.hpp"
#include "hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp"
#include "simple_test_hardware.hpp"

using namespace hector_transmission_interface;
using namespace hector_transmission_interface::test;
using hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

class OffsetManagerIntegrationTest : public hector_testing_utils::HectorTestFixture
{
protected:
  void SetUp() override
  {
    // 1. Setup Temporary Home Directory for Isolation
    char dt[] = "/tmp/ros_test_home_XXXXXX";
    test_dir_ = mkdtemp( dt );
    original_home_ = std::getenv( "HOME" );
    setenv( "HOME", test_dir_.c_str(), 1 );

    HectorTestFixture::SetUp();

    // Define standard joints for most tests
    joint_names_ = { "test_joint", "aux_joint" };
    hardware_ = std::make_shared<SimpleTestHardware>( joint_names_ );

    // Initialize Hardware
    hardware_interface::HardwareInfo info; // Dummy info
    hardware_->on_init( info );
    hardware_->on_configure( rclcpp_lifecycle::State() );
    hardware_->on_activate( rclcpp_lifecycle::State() );
    hardware_->on_export_state_interfaces();
    hardware_->on_export_command_interfaces();

    // Add hardware node to executor
    executor_->add_node( hardware_->get_node() );

    // Setup Client
    client_ = tester_node_->create_test_client<AdjustTransmissionOffsets>(
        "/simple_test_hw_node/adjust_transmission_offsets" );
  }

  void TearDown() override
  {
    if ( original_home_ ) {
      setenv( "HOME", original_home_, 1 );
    }
    std::filesystem::remove_all( test_dir_ );
    HectorTestFixture::TearDown();
  }

  std::string test_dir_;
  const char *original_home_;
  std::vector<std::string> joint_names_;
  std::shared_ptr<SimpleTestHardware> hardware_;
  std::shared_ptr<hector_testing_utils::TestClient<AdjustTransmissionOffsets>> client_;
};

TEST_F( OffsetManagerIntegrationTest, UpdateOffsetAndVerify )
{
  std::string j = "test_joint";
  // Initial: Actuator=1.0, Offset=0.0 -> Joint=1.0
  hardware_->set_raw_actuator_position( j, 1.0 );
  hardware_->propagate_read();

  ASSERT_NEAR( hardware_->get_joint_position( j ), 1.0, 1e-6 );

  ASSERT_TRUE( client_->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name.push_back( j );
  // Target: Joint=2.5. Expect correction from 1.0 to 2.5 -> Offset = +1.5
  request->external_joint_measurements.position.push_back( 2.5 );

  auto response = hector_testing_utils::call_service<AdjustTransmissionOffsets>(
      client_->get(), request, *executor_ );

  ASSERT_NE( response, nullptr );
  ASSERT_TRUE( response->success ) << response->message;
  EXPECT_NEAR( response->offset_changes[0], 1.5, 1e-4 );

  // Verify Internal State Updated
  hardware_->propagate_read();
  EXPECT_NEAR( hardware_->get_joint_position( j ), 2.5, 1e-6 );

  // Verify Persistence
  std::filesystem::path offset_file =
      std::filesystem::path( test_dir_ ) / ".ros" / "dynamic_offset_transmissions" / ( j + ".txt" );
  ASSERT_TRUE( std::filesystem::exists( offset_file ) );
}

TEST_F( OffsetManagerIntegrationTest, PersistenceAcrossRestarts )
{
  std::string j = "test_joint";
  // 1. Manually create offset file (Offset=3.14)
  std::filesystem::path offset_dir =
      std::filesystem::path( test_dir_ ) / ".ros" / "dynamic_offset_transmissions";
  std::filesystem::create_directories( offset_dir );
  std::ofstream f( offset_dir / ( j + ".txt" ) );
  f << "3.14";
  f.close();

  // 2. Re-initialize hardware (fresh instance)
  auto fresh_hardware = std::make_shared<SimpleTestHardware>( j );
  hardware_interface::HardwareInfo info;
  fresh_hardware->on_init( info );

  // 3. Verify it used the offset
  // Raw=1.0 + Offset=3.14 -> Joint=4.14
  fresh_hardware->set_raw_actuator_position( j, 1.0 );
  fresh_hardware->on_export_state_interfaces();
  fresh_hardware->on_export_command_interfaces();
  fresh_hardware->propagate_read();

  EXPECT_NEAR( fresh_hardware->get_joint_position( j ), 4.14, 1e-6 );

  // 4. Verify service call with original state logic
  // Add node to executor
  executor_->add_node( fresh_hardware->get_node() );
  auto fresh_client = tester_node_->create_test_client<AdjustTransmissionOffsets>(
      "/simple_test_hw_node/adjust_transmission_offsets" );

  hardware_.reset(); // Cleanup old HW

  ASSERT_TRUE( fresh_client->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name.push_back( j );
  // Target=5.14.
  // Internal (from HW) is 4.14.
  // Existing offset = 3.14.
  // New Offset = Target(5.14) - Internal(4.14) + Existing(3.14) = 4.14.
  // Offset Change = New(4.14) - Existing(3.14) = 1.0
  request->external_joint_measurements.position.push_back( 5.14 );

  auto response = hector_testing_utils::call_service<AdjustTransmissionOffsets>(
      fresh_client->get(), request, *executor_ );

  ASSERT_TRUE( response->success );
  EXPECT_NEAR( response->offset_changes[0], 1.0, 1e-4 );
}

TEST_F( OffsetManagerIntegrationTest, MultipleJoints )
{
  std::string j1 = "test_joint";
  std::string j2 = "aux_joint";

  // Initial: Both Raw=1.0, Offset=0.0 -> Joint=1.0
  hardware_->set_raw_actuator_position( j1, 1.0 );
  hardware_->set_raw_actuator_position( j2, 2.0 ); // Diff starting position
  hardware_->propagate_read();

  ASSERT_NEAR( hardware_->get_joint_position( j1 ), 1.0, 1e-6 );
  ASSERT_NEAR( hardware_->get_joint_position( j2 ), 2.0, 1e-6 );

  ASSERT_TRUE( client_->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { j1, j2 };
  request->external_joint_measurements.position = { 1.5, 3.0 };
  // Expected Offsets:
  // J1: 1.5 - 1.0 + 0 = 0.5
  // J2: 3.0 - 2.0 + 0 = 1.0

  auto response = hector_testing_utils::call_service<AdjustTransmissionOffsets>(
      client_->get(), request, *executor_ );

  ASSERT_TRUE( response->success );
  ASSERT_EQ( response->offset_changes.size(), 2u );
  EXPECT_NEAR( response->offset_changes[0], 0.5, 1e-4 );
  EXPECT_NEAR( response->offset_changes[1], 1.0, 1e-4 );

  hardware_->propagate_read();
  EXPECT_NEAR( hardware_->get_joint_position( j1 ), 1.5, 1e-6 );
  EXPECT_NEAR( hardware_->get_joint_position( j2 ), 3.0, 1e-6 );
}

TEST_F( OffsetManagerIntegrationTest, UnknownJointName )
{
  ASSERT_TRUE( client_->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "ghost_joint" };
  request->external_joint_measurements.position = { 1.0 };

  auto response = hector_testing_utils::call_service<AdjustTransmissionOffsets>(
      client_->get(), request, *executor_ );

  // Service should return false for success if no joints were adjusted
  ASSERT_TRUE( response );
  EXPECT_FALSE( response->success );
  EXPECT_TRUE( response->offset_changes.empty() );
}

TEST_F( OffsetManagerIntegrationTest, NaNInputRobustness )
{
  std::string j = "test_joint";
  hardware_->set_raw_actuator_position( j, 1.0 );
  hardware_->propagate_read();

  ASSERT_TRUE( client_->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { j };
  request->external_joint_measurements.position = { std::numeric_limits<double>::quiet_NaN() };

  auto response = hector_testing_utils::call_service<AdjustTransmissionOffsets>(
      client_->get(), request, *executor_ );

  ASSERT_TRUE( response );
  EXPECT_FALSE( response->success ); // Nothing adjusted
  EXPECT_TRUE( response->offset_changes.empty() );

  // Ensure position didn't change (garbage not applied)
  hardware_->propagate_read();
  EXPECT_NEAR( hardware_->get_joint_position( j ), 1.0, 1e-6 );
}

TEST_F( OffsetManagerIntegrationTest, OffsetAccumulation )
{
  // Tests Drift Correction / Tuning
  std::string j = "test_joint";
  hardware_->set_raw_actuator_position( j, 1.0 );
  hardware_->propagate_read(); // = 1.0

  ASSERT_TRUE( client_->wait_for_service( *executor_, std::chrono::seconds( 5 ) ) );

  // Step 1: Adjust +0.5 -> Target 1.5
  {
    auto req = std::make_shared<AdjustTransmissionOffsets::Request>();
    req->external_joint_measurements.name = { j };
    req->external_joint_measurements.position = { 1.5 };
    auto resp = hector_testing_utils::call_service<AdjustTransmissionOffsets>( client_->get(), req,
                                                                               *executor_ );
    ASSERT_TRUE( resp->success );
    EXPECT_NEAR( resp->offset_changes[0], 0.5, 1e-4 );
  }

  hardware_->propagate_read();
  EXPECT_NEAR( hardware_->get_joint_position( j ), 1.5, 1e-6 );

  // Step 2: Adjust +0.1 more -> Target 1.6
  {
    auto req = std::make_shared<AdjustTransmissionOffsets::Request>();
    req->external_joint_measurements.name = { j };
    req->external_joint_measurements.position = { 1.6 };
    auto resp = hector_testing_utils::call_service<AdjustTransmissionOffsets>( client_->get(), req,
                                                                               *executor_ );
    ASSERT_TRUE( resp->success );
    // New Offset = Target(1.6) - Internal(1.5) + Existing(0.5) = 0.6
    // Offset Change = New(0.6) - Existing(0.5) = 0.1
    EXPECT_NEAR( resp->offset_changes[0], 0.1, 1e-4 );
  }

  hardware_->propagate_read(); // Raw(1.0) + Offset(0.6) = 1.6
  EXPECT_NEAR( hardware_->get_joint_position( j ), 1.6, 1e-6 );
}
