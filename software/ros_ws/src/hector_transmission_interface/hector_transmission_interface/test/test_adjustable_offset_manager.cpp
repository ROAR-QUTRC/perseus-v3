#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hector_transmission_interface/adjustable_offset_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtest/service_mock.hpp"

using namespace hector_transmission_interface;
using namespace hector_transmission_interface_msgs::srv;
using testing::DoubleEq;
using testing::ElementsAre;
using testing::Field;
using testing::Return;

class MockAdjustableOffset : public IAdjustableOffset
{
public:
  MOCK_METHOD( void, adjustOffset, ( double offset ), ( override ) );
  MOCK_METHOD( double, getOffset, (), ( const, override ) );
};

class AdjustableOffsetManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "test_node" );
    manager_ = std::make_shared<AdjustableOffsetManager>( node_ );

    // 1. Try to find the service. If it fails, stop the test immediately to avoid Segfault.
    auto service =
        rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
    ASSERT_NE( service, nullptr ) << "Service not found in rtest registry!";
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<AdjustableOffsetManager> manager_;
};

class AdjustableOffsetManagerTestMutex : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "test_node" );
    manager_ = std::make_shared<AdjustableOffsetManager>( node_, std::ref( blocking_mutex ) );

    // 1. Try to find the service. If it fails, stop the test immediately to avoid Segfault.
    auto service =
        rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
    ASSERT_NE( service, nullptr ) << "Service not found in rtest registry!";
  }
  rclcpp::Node::SharedPtr node_;
  std::mutex blocking_mutex;
  std::shared_ptr<AdjustableOffsetManager> manager_;
};

TEST_F( AdjustableOffsetManagerTest, RobustOffsetCalculation )
{
  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  std::string joint_name = "flipper_joint";

  // Logic: NewOffset = External(1.2) - Internal(1.0) + CurrentOffset(0.5) = 0.7
  //        OffsetChange = NewOffset(0.7) - CurrentOffset(0.5) = 0.2
  double current_internal_val = 1.0;
  double old_offset = 0.5;
  double external_meas = 1.2;
  double expected_new_offset = 0.7;
  double expected_offset_change = expected_new_offset - old_offset; // 0.2

  // Setup Mock expectations for the internal transmission handles
  EXPECT_CALL( *mock_state, getOffset() ).WillRepeatedly( Return( old_offset ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );

  // Inject mocks into manager
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = [&]() { return current_internal_val; };
  manager_->add_managed_joint( joint_name, mj );

  // Retrieve the service mock via rtest
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  ASSERT_TRUE( service );

  // 1. Set up Request Header (rtest requirement)
  auto request_header = std::make_shared<rmw_request_id_t>();
  request_header->sequence_number = 1L;

  // 2. Set up Request Data
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name.push_back( joint_name );
  request->external_joint_measurements.position.push_back( external_meas );

  // 3. Set up Expected Response for validation
  AdjustTransmissionOffsets::Response expected_response;
  expected_response.success = true;
  expected_response.offset_changes.push_back( expected_offset_change );
  // Optional: match message content if desired

  // 4. Expect that the service will call send_response with the correct data
  // Using ElementsAre to check the floating point vector content
  EXPECT_CALL( *service,
               send_response(
                   *request_header,
                   testing::AllOf( Field( &AdjustTransmissionOffsets::Response::success, true ),
                                   Field( &AdjustTransmissionOffsets::Response::offset_changes,
                                          ElementsAre( DoubleEq( expected_offset_change ) ) ) ) ) );

  // 5. Simulate the service call
  service->handle_request( request_header, request );
}

TEST_F( AdjustableOffsetManagerTest, CalculationWithOriginalState )
{
  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  std::string joint_name = "flipper_joint";

  // Current hardware state is 1.0, but original measurement state was 0.8
  // NewOffset = External(1.5) - OriginalInternal(0.8) + CurrentOffset(0.0) = 0.7
  double current_internal_val = 1.0;
  double original_internal_val = 0.8;
  double external_meas = 1.5;
  double expected_new_offset = 0.7;

  EXPECT_CALL( *mock_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );

  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = [&]() { return current_internal_val; };
  manager_->add_managed_joint( joint_name, mj );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name.push_back( joint_name );
  request->external_joint_measurements.position.push_back( external_meas );

  // Provide the original state to bypass the position_getter
  request->original_joint_state.name.push_back( joint_name );
  request->original_joint_state.position.push_back( original_internal_val );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, true ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Partial Data and Fallback Logic
 * Scenario: Request contains 2 joints. Joint A has an 'original_joint_state',
 * Joint B does not. Joint B should fallback to the position_getter.
 */
TEST_F( AdjustableOffsetManagerTest, PartialOriginalStateFallback )
{
  auto mock_a_state = std::make_shared<MockAdjustableOffset>();
  auto mock_a_cmd = std::make_shared<MockAdjustableOffset>();
  auto mock_b_state = std::make_shared<MockAdjustableOffset>();
  auto mock_b_cmd = std::make_shared<MockAdjustableOffset>();

  // Joint A: External(1.5) - Original(1.0) + Offset(0.0) = 0.5
  // Joint B: External(2.5) - Current(2.0) + Offset(0.0) = 0.5

  AdjustableOffsetManager::ManagedJoint mj_a, mj_b;
  mj_a.state_handle = mock_a_state;
  mj_a.command_handle = mock_a_cmd;
  mj_a.position_getter = []() { return 999.0; }; // Should be ignored for A

  mj_b.state_handle = mock_b_state;
  mj_b.command_handle = mock_b_cmd;
  mj_b.position_getter = []() { return 2.0; }; // Used for B

  manager_->add_managed_joint( "joint_a", mj_a );
  manager_->add_managed_joint( "joint_b", mj_b );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "joint_a", "joint_b" };
  request->external_joint_measurements.position = { 1.5, 2.5 };

  // Only provide original state for joint_a
  request->original_joint_state.name = { "joint_a" };
  request->original_joint_state.position = { 1.0 };

  EXPECT_CALL( *mock_a_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_a_state, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );
  EXPECT_CALL( *mock_a_cmd, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );

  EXPECT_CALL( *mock_b_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_b_state, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );
  EXPECT_CALL( *mock_b_cmd, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, true ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Invalid Joint Names
 * Scenario: Request contains a joint name that the manager doesn't know about.
 * Manager should return success=false or at least not crash/adjust wrong joints.
 */
TEST_F( AdjustableOffsetManagerTest, NonExistentJointInRequest )
{
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "ghost_joint" };
  request->external_joint_measurements.position = { 1.0 };

  // We expect success = false because nothing was adjusted
  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, false ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Corrupted Data (NaN)
 * Scenario: External measurement is NaN. Manager should skip or handle gracefully.
 */
TEST_F( AdjustableOffsetManagerTest, HandlingNaNValues )
{
  auto mock = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mj.command_handle = mock;
  mj.position_getter = []() { return 1.0; };
  manager_->add_managed_joint( "nan_joint", mj );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "nan_joint" };
  request->external_joint_measurements.position = { std::numeric_limits<double>::quiet_NaN() };

  // The manager should NOT call adjustOffset if the input is NaN
  EXPECT_CALL( *mock, adjustOffset( testing::_ ) ).Times( 0 );

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, false ) ) );

  service->handle_request( request_header, request );
}

/**
 * Test: Mismatched Array Sizes
 * Scenario: User sends 2 names but only 1 position.
 * This tests the robustness of the loop indices.
 */
TEST_F( AdjustableOffsetManagerTest, MismatchedArraySizes )
{
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();

  request->external_joint_measurements.name = { "j1", "j2" };
  request->external_joint_measurements.position = { 1.0 }; // Only one!

  // Manager should handle this without out_of_bounds exception
  // Usually by iterating based on the smaller of the two sizes or the name size with a check.
  EXPECT_CALL( *service, send_response( *request_header, testing::_ ) ).Times( 1 );

  service->handle_request( request_header, request );
}

/* Test: Mutex Locking Behavior
 * Scenario: The manager is constructed with a mutex reference.
 * We test that the service handler waits for the mutex to be released.
 */
TEST_F( AdjustableOffsetManagerTestMutex, VerifyServiceWaitsForMutex )
{

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "locked_joint" };
  request->external_joint_measurements.position = { 1.0 };

  // 1. Grab the lock in the test thread
  std::unique_lock<std::mutex> test_lock( blocking_mutex );

  // 2. Launch service call in background
  bool service_finished = false;
  auto future = std::async( std::launch::async, [&]() {
    service->handle_request( request_header, request );
    service_finished = true;
  } );

  // 3. Wait a bit and verify the service is still blocked
  std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
  EXPECT_FALSE( service_finished ) << "Service proceeded even though mutex was locked by test!";

  // 4. Release lock and verify service finishes
  test_lock.unlock();

  // Use a timeout to avoid infinite hangs if the test fails
  auto status = future.wait_for( std::chrono::seconds( 1 ) );
  EXPECT_EQ( status, std::future_status::ready );
  EXPECT_TRUE( service_finished );
}

/* Test: Full Offset Calculation with Mutex
 * Scenario: Same as RobustOffsetCalculation but with mutex locking.
 */
TEST_F( AdjustableOffsetManagerTestMutex, RobustOffsetCalculation )
{
  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  std::string joint_name = "flipper_joint";

  // Logic: NewOffset = External(1.2) - Internal(1.0) + CurrentOffset(0.5) = 0.7
  //        OffsetChange = NewOffset(0.7) - CurrentOffset(0.5) = 0.2
  double current_internal_val = 1.0;
  double old_offset = 0.5;
  double external_meas = 1.2;
  double expected_new_offset = 0.7;
  double expected_offset_change = expected_new_offset - old_offset; // 0.2

  // Setup Mock expectations for the internal transmission handles
  EXPECT_CALL( *mock_state, getOffset() ).WillRepeatedly( Return( old_offset ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( expected_new_offset ) ) ).Times( 1 );

  // Inject mocks into manager
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = [&]() { return current_internal_val; };
  manager_->add_managed_joint( joint_name, mj );

  // Retrieve the service mock via rtest
  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  ASSERT_TRUE( service );

  // 1. Set up Request Header (rtest requirement)
  auto request_header = std::make_shared<rmw_request_id_t>();
  request_header->sequence_number = 1L;

  // 2. Set up Request Data
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name.push_back( joint_name );
  request->external_joint_measurements.position.push_back( external_meas );

  // 3. Set up Expected Response for validation
  AdjustTransmissionOffsets::Response expected_response;
  expected_response.success = true;
  expected_response.offset_changes.push_back( expected_offset_change );
  // Optional: match message content if desired

  // 4. Expect that the service will call send_response with the correct data
  // Using ElementsAre to check the floating point vector content
  EXPECT_CALL( *service,
               send_response(
                   *request_header,
                   testing::AllOf( Field( &AdjustTransmissionOffsets::Response::success, true ),
                                   Field( &AdjustTransmissionOffsets::Response::offset_changes,
                                          ElementsAre( DoubleEq( expected_offset_change ) ) ) ) ) );

  // 5. Simulate the service call
  service->handle_request( request_header, request );
}
class AdjustableOffsetManagerTestCallbacks : public ::testing::Test
{
protected:
  void SetUp() override { node_ = std::make_shared<rclcpp::Node>( "test_node" ); }
  rclcpp::Node::SharedPtr node_;
};

/**
 * Test: Pre-callback returning false aborts service
 */
TEST_F( AdjustableOffsetManagerTestCallbacks, PreCallbackReturnsFalseAbortsService )
{
  bool pre_called = false;
  auto pre_callback = [&pre_called]() {
    pre_called = true;
    return false; // Abort
  };

  auto manager = std::make_shared<AdjustableOffsetManager>(
      node_, std::nullopt, std::make_optional( pre_callback ), std::nullopt );

  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = []() { return 1.0; };
  manager->add_managed_joint( "test_joint", mj );

  // adjustOffset should NOT be called if pre-callback returns false
  EXPECT_CALL( *mock_state, adjustOffset( testing::_ ) ).Times( 0 );
  EXPECT_CALL( *mock_cmd, adjustOffset( testing::_ ) ).Times( 0 );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  ASSERT_TRUE( service );

  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "test_joint" };
  request->external_joint_measurements.position = { 1.5 };

  EXPECT_CALL(
      *service,
      send_response( *request_header,
                     testing::AllOf( Field( &AdjustTransmissionOffsets::Response::success, false ),
                                     Field( &AdjustTransmissionOffsets::Response::message,
                                            testing::HasSubstr( "Pre-callback" ) ) ) ) );

  service->handle_request( request_header, request );
  EXPECT_TRUE( pre_called );
}

/**
 * Test: Pre-callback returning true allows processing
 */
TEST_F( AdjustableOffsetManagerTestCallbacks, PreCallbackReturnsTrueAllowsProcessing )
{
  bool pre_called = false;
  auto pre_callback = [&pre_called]() {
    pre_called = true;
    return true; // Continue
  };

  auto manager = std::make_shared<AdjustableOffsetManager>(
      node_, std::nullopt, std::make_optional( pre_callback ), std::nullopt );

  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = []() { return 1.0; };
  manager->add_managed_joint( "test_joint", mj );

  EXPECT_CALL( *mock_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_state, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( DoubleEq( 0.5 ) ) ).Times( 1 );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "test_joint" };
  request->external_joint_measurements.position = { 1.5 };

  EXPECT_CALL( *service,
               send_response( *request_header,
                              Field( &AdjustTransmissionOffsets::Response::success, true ) ) );

  service->handle_request( request_header, request );
  EXPECT_TRUE( pre_called );
}

/**
 * Test: Post-callback is called after successful processing
 */
TEST_F( AdjustableOffsetManagerTestCallbacks, PostCallbackCalledAfterProcessing )
{
  bool post_called = false;
  auto post_callback = [&post_called]() {
    post_called = true;
    return true;
  };

  auto manager = std::make_shared<AdjustableOffsetManager>( node_, std::nullopt, std::nullopt,
                                                            std::make_optional( post_callback ) );

  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = []() { return 1.0; };
  manager->add_managed_joint( "test_joint", mj );

  EXPECT_CALL( *mock_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_state, adjustOffset( testing::_ ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( testing::_ ) ).Times( 1 );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "test_joint" };
  request->external_joint_measurements.position = { 1.5 };

  EXPECT_CALL( *service, send_response( *request_header, testing::_ ) );

  service->handle_request( request_header, request );
  EXPECT_TRUE( post_called );
}

/**
 * Test: Post-callback NOT called when pre-callback aborts
 */
TEST_F( AdjustableOffsetManagerTestCallbacks, PostCallbackNotCalledWhenPreAborts )
{
  bool pre_called = false;
  bool post_called = false;
  auto pre_callback = [&pre_called]() {
    pre_called = true;
    return false;
  };
  auto post_callback = [&post_called]() {
    post_called = true;
    return true;
  };

  auto manager = std::make_shared<AdjustableOffsetManager>(
      node_, std::nullopt, std::make_optional( pre_callback ), std::make_optional( post_callback ) );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "any" };
  request->external_joint_measurements.position = { 1.0 };

  EXPECT_CALL( *service, send_response( *request_header, testing::_ ) );

  service->handle_request( request_header, request );
  EXPECT_TRUE( pre_called );
  EXPECT_FALSE( post_called );
}

/**
 * Test: Both callbacks with full processing
 */
TEST_F( AdjustableOffsetManagerTestCallbacks, BothCallbacksExecuteInOrder )
{
  int call_order = 0;
  int pre_order = 0, post_order = 0;

  auto pre_callback = [&]() {
    pre_order = ++call_order;
    return true;
  };
  auto post_callback = [&]() {
    post_order = ++call_order;
    return true;
  };

  auto manager = std::make_shared<AdjustableOffsetManager>(
      node_, std::nullopt, std::make_optional( pre_callback ), std::make_optional( post_callback ) );

  auto mock_state = std::make_shared<MockAdjustableOffset>();
  auto mock_cmd = std::make_shared<MockAdjustableOffset>();
  AdjustableOffsetManager::ManagedJoint mj;
  mj.state_handle = mock_state;
  mj.command_handle = mock_cmd;
  mj.position_getter = []() { return 1.0; };
  manager->add_managed_joint( "test_joint", mj );

  EXPECT_CALL( *mock_state, getOffset() ).WillOnce( Return( 0.0 ) );
  EXPECT_CALL( *mock_state, adjustOffset( testing::_ ) ).Times( 1 );
  EXPECT_CALL( *mock_cmd, adjustOffset( testing::_ ) ).Times( 1 );

  auto service =
      rtest::findService<AdjustTransmissionOffsets>( node_, "~/adjust_transmission_offsets" );
  auto request_header = std::make_shared<rmw_request_id_t>();
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = { "test_joint" };
  request->external_joint_measurements.position = { 1.5 };

  EXPECT_CALL( *service, send_response( *request_header, testing::_ ) );

  service->handle_request( request_header, request );

  EXPECT_EQ( pre_order, 1 );
  EXPECT_EQ( post_order, 2 );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
