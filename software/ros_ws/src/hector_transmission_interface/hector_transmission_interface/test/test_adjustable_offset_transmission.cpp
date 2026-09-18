//
// Created by aljoscha-schmidt on 7/12/25.
//
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hector_transmission_interface/adjustable_offset_transmission.hpp>

using namespace hector_transmission_interface;

class AdjustableOffsetTransmissionTest : public ::testing::Test
{
protected:
  std::string joint_name = "test_joint_offset";
  std::string secondary_joint_name = "test_joint_offset_secondary";
  std::filesystem::path offset_file_1;
  std::filesystem::path offset_file_2;

  void SetUp() override
  {
    offset_file_1 = std::filesystem::path( std::getenv( "HOME" ) ) /
                    ".ros/dynamic_offset_transmissions" / ( joint_name + ".txt" );
    // Clean before each test
    std::filesystem::remove( offset_file_1 );
    offset_file_2 = std::filesystem::path( std::getenv( "HOME" ) ) /
                    ".ros/dynamic_offset_transmissions" / ( secondary_joint_name + ".txt" );
    // Clean before each test
    std::filesystem::remove( offset_file_2 );
  }

  void TearDown() override
  {
    std::filesystem::remove( offset_file_1 );
    std::filesystem::remove( offset_file_2 );
  }

  bool fileContainsValue( double expected, double tolerance = 1e-6 )
  {
    std::ifstream file( offset_file_1 );
    double val = 0.0;
    file >> val;
    bool near = std::abs( val - expected ) < tolerance;
    if ( !near )
      std::cout << "Expected: " << expected << ", got: " << val << std::endl;
    return near;
  }

  /**
   * @brief Helper to create a configured AdjustableOffsetTransmission with position handles.
   *
   * Sets up actuator and joint position handles so that actuator_to_joint() can operate.
   * The caller owns the actuator_pos and joint_pos doubles and must keep them alive.
   */
  static AdjustableOffsetTransmission createConfiguredTransmission( const std::string &name,
                                                                    double reduction, double offset,
                                                                    double &actuator_pos,
                                                                    double &joint_pos )
  {
    AdjustableOffsetTransmission trans( name, reduction, offset );

    std::vector<transmission_interface::JointHandle> joint_handles;
    joint_handles.emplace_back( name, hardware_interface::HW_IF_POSITION, &joint_pos );

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    actuator_handles.emplace_back( "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos );

    trans.configure( joint_handles, actuator_handles );
    return trans;
  }
};

TEST_F( AdjustableOffsetTransmissionTest, LoadAndSaveOffset )
{
  // 1) Create instance with initial offset
  AdjustableOffsetTransmission trans( joint_name, 1.0, 0.123 );
  EXPECT_NEAR( trans.get_joint_offset(), 0.123, 1e-9 );
  EXPECT_TRUE( fileContainsValue( 0.123 ) );

  // 2) Adjust offset and check file was updated
  trans.adjustTransmissionOffset( 0.456 );
  EXPECT_NEAR( trans.get_joint_offset(), 0.456, 1e-9 );
  EXPECT_TRUE( fileContainsValue( 0.456 ) );

  // 3) Reload new instance from file, check offset persists
  AdjustableOffsetTransmission trans2( joint_name, 1.0 );
  EXPECT_NEAR( trans2.get_joint_offset(), 0.456, 1e-9 );

  // 4) Load second instance with different joint name -> make sure different files are used
  AdjustableOffsetTransmission trans3( secondary_joint_name, 1.0, 2.0 );
  EXPECT_NEAR( trans3.get_joint_offset(), 2.0,
               1e-9 ); // Should be 2.0 -> no file exists for this joint use provided default
}

// ============================================================================
// 2pi Jump Detection Tests
// ============================================================================

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_NoJumpOnFirstRead )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  // First call should just establish baseline, no correction
  trans.actuator_to_joint();
  EXPECT_EQ( trans.getCorrectionCount(), 0 );
  EXPECT_NEAR( joint_pos, 1.0 / 2.0, 1e-9 ); // actuator / reduction + offset
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_NoJumpOnSmallChange )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // establish baseline

  // Small movement, no jump expected
  actuator_pos = 1.1;
  trans.actuator_to_joint();
  EXPECT_EQ( trans.getCorrectionCount(), 0 );
  EXPECT_NEAR( joint_pos, 1.1 / 2.0, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_Positive2PiJump )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline: actuator=1.0
  double joint_pos_before = joint_pos;

  // Simulate a +2pi jump on the actuator (power glitch)
  actuator_pos = 1.0 + 2.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  // Joint position should be unchanged (the jump was corrected)
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_Negative2PiJump )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline
  double joint_pos_before = joint_pos;

  // Simulate a -2pi jump on the actuator
  actuator_pos = 1.0 - 2.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_Multiple2PiJumps )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline
  double joint_pos_before = joint_pos;

  // Simulate a +4pi (2x 2pi) jump on the actuator
  actuator_pos = 1.0 + 4.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 2 ); // abs(n_jumps) = 2
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_WithNegativeReduction )
{
  // Flippers use negative reductions (e.g. -2.0)
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, -2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline
  double joint_pos_before = joint_pos;

  // +2pi jump on actuator
  actuator_pos = 1.0 + 2.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_OffsetPersistedToDisk )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  double initial_offset = 0.5;
  auto trans =
      createConfiguredTransmission( joint_name, 2.0, initial_offset, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline

  // Trigger a +2pi jump
  actuator_pos = 1.0 + 2.0 * M_PI;
  trans.actuator_to_joint();

  // The corrected offset should be persisted to disk
  double expected_offset = initial_offset - ( 2.0 * M_PI / 2.0 ); // offset -= 2pi/reduction
  EXPECT_NEAR( trans.get_joint_offset(), expected_offset, 1e-9 );

  // Verify persistence: reload from file
  AdjustableOffsetTransmission trans2( joint_name, 2.0, 0.0 );
  EXPECT_NEAR( trans2.get_joint_offset(), expected_offset, 1e-4 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_NonJumpLargeMovementNotCorrected )
{
  // A movement that is large but NOT close to 2pi should not trigger correction
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline

  // Move by 3.0 rad (not close to 2pi = 6.28)
  actuator_pos = 4.0;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 0 );
  EXPECT_NEAR( joint_pos, 4.0 / 2.0, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_NoCorrectionAfterManualAdjust )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline at actuator=1.0

  // Manually adjust offset (simulates calibration service call)
  // This internally calls actuator_to_joint() and resets prev_actuator_position_
  trans.adjustTransmissionOffset( 5.0 );

  // Now simulate a read with the same actuator position — no jump should be detected
  trans.actuator_to_joint();
  EXPECT_EQ( trans.getCorrectionCount(), 0 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_ConsecutiveJumpsHandledCorrectly )
{
  double actuator_pos = 1.0;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 2.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline
  double joint_pos_stable = joint_pos;

  // First jump: +2pi
  actuator_pos = 1.0 + 2.0 * M_PI;
  trans.actuator_to_joint();
  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  EXPECT_NEAR( joint_pos, joint_pos_stable, 1e-9 );

  // Several normal reads at the new actuator position
  for ( int i = 0; i < 5; ++i ) { trans.actuator_to_joint(); }
  EXPECT_EQ( trans.getCorrectionCount(), 1 ); // no new corrections

  // Second jump: another +2pi from current position
  actuator_pos += 2.0 * M_PI;
  trans.actuator_to_joint();
  EXPECT_EQ( trans.getCorrectionCount(), 2 );
  EXPECT_NEAR( joint_pos, joint_pos_stable, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, JumpDetection_WithReduction1_JointJumpsBy2Pi )
{
  // With reduction=1.0, a 2pi actuator jump = 2pi joint jump
  // The correction should still work and keep joint position stable
  double actuator_pos = 0.5;
  double joint_pos = 0.0;
  auto trans = createConfiguredTransmission( joint_name, 1.0, 0.0, actuator_pos, joint_pos );

  trans.actuator_to_joint(); // baseline
  double joint_pos_before = joint_pos;

  actuator_pos = 0.5 + 2.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}

// ============================================================================
// Passthrough behavior (pass_through_effort + pass_through_interfaces)
// ============================================================================

TEST_F( AdjustableOffsetTransmissionTest, DefaultEffortStillScalesByReduction )
{
  // Regression guard: with pass_through_effort=false (the default), effort is
  // scaled by reduction in actuator_to_joint, matching SimpleTransmission semantics.
  double actuator_pos = 0.0, joint_pos = 0.0;
  double actuator_eff = 0.0, joint_eff = 0.0;
  AdjustableOffsetTransmission trans( joint_name, 2.0, 0.0 );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, hardware_interface::HW_IF_EFFORT, &joint_eff } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", hardware_interface::HW_IF_EFFORT, &actuator_eff } };
  trans.configure( jh, ah );

  actuator_eff = 1.5;
  trans.actuator_to_joint();
  EXPECT_NEAR( joint_eff, 1.5 * 2.0, 1e-9 ); // base SimpleTransmission: tau_j = n * tau_a

  joint_eff = 4.0;
  trans.joint_to_actuator();
  EXPECT_NEAR( actuator_eff, 4.0 / 2.0, 1e-9 ); // tau_a = tau_j / n
}

TEST_F( AdjustableOffsetTransmissionTest, PassThroughEffort_ActuatorToJointIsIdentity )
{
  double actuator_pos = 0.0, joint_pos = 0.0;
  double actuator_eff = 0.0, joint_eff = 0.0;
  AdjustableOffsetTransmission trans( joint_name, 2.0, 0.0,
                                      /*pass_through_effort=*/true );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, hardware_interface::HW_IF_EFFORT, &joint_eff } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", hardware_interface::HW_IF_EFFORT, &actuator_eff } };
  trans.configure( jh, ah );

  actuator_eff = 1.5;
  trans.actuator_to_joint();
  EXPECT_NEAR( joint_eff, 1.5, 1e-9 ); // identity, NOT scaled

  // Position still scales normally
  actuator_pos = 4.0;
  trans.actuator_to_joint();
  EXPECT_NEAR( joint_pos, 4.0 / 2.0, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, PassThroughEffort_JointToActuatorIsIdentity )
{
  double actuator_pos = 0.0, joint_pos = 0.0;
  double actuator_eff = 0.0, joint_eff = 0.0;
  AdjustableOffsetTransmission trans( joint_name, 2.0, 0.0,
                                      /*pass_through_effort=*/true );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, hardware_interface::HW_IF_EFFORT, &joint_eff } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", hardware_interface::HW_IF_EFFORT, &actuator_eff } };
  trans.configure( jh, ah );

  joint_eff = 4.0;
  trans.joint_to_actuator();
  EXPECT_NEAR( actuator_eff, 4.0, 1e-9 ); // identity, NOT divided
}

TEST_F( AdjustableOffsetTransmissionTest, PassThroughCustomInterface_BothDirections )
{
  // A non-standard interface name like "current" should bridge as identity
  // when listed in pass_through_interfaces, even though SimpleTransmission
  // does not know about it (and would otherwise drop it entirely).
  double actuator_pos = 0.0, joint_pos = 0.0;
  double actuator_cur = 0.0, joint_cur = 0.0;
  AdjustableOffsetTransmission trans( joint_name, -10.5, 0.0,
                                      /*pass_through_effort=*/false,
                                      /*pass_through_interfaces=*/{ "current" } );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, "current", &joint_cur } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", "current", &actuator_cur } };
  trans.configure( jh, ah );

  // joint -> actuator: command flows out
  joint_cur = 2.5;
  trans.joint_to_actuator();
  EXPECT_NEAR( actuator_cur, 2.5, 1e-9 );

  // actuator -> joint: state flows in
  actuator_cur = 7.0;
  trans.actuator_to_joint();
  EXPECT_NEAR( joint_cur, 7.0, 1e-9 );
}

TEST_F( AdjustableOffsetTransmissionTest, PassThroughEffortAndCustom_PositionStillScaled )
{
  // Combined: gripper-like config — position scales, effort+current passthrough.
  double actuator_pos = 0.0, joint_pos = 0.0;
  double actuator_eff = 0.0, joint_eff = 0.0;
  double actuator_cur = 0.0, joint_cur = 0.0;
  AdjustableOffsetTransmission trans( joint_name, -10.5, 0.5,
                                      /*pass_through_effort=*/true,
                                      /*pass_through_interfaces=*/{ "current" } );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, hardware_interface::HW_IF_EFFORT, &joint_eff },
      { joint_name, "current", &joint_cur } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", hardware_interface::HW_IF_EFFORT, &actuator_eff },
      { "actuator", "current", &actuator_cur } };
  trans.configure( jh, ah );

  actuator_pos = 3.0;
  actuator_eff = 1.2;
  actuator_cur = 0.8;
  trans.actuator_to_joint();
  EXPECT_NEAR( joint_pos, 3.0 / -10.5 + 0.5, 1e-9 ); // scaled with offset
  EXPECT_NEAR( joint_eff, 1.2, 1e-9 );               // identity
  EXPECT_NEAR( joint_cur, 0.8, 1e-9 );               // identity

  joint_pos = 0.5;
  joint_eff = 2.0;
  joint_cur = 1.5;
  trans.joint_to_actuator();
  EXPECT_NEAR( actuator_pos, ( 0.5 - 0.5 ) * -10.5, 1e-9 ); // (joint - offset) * reduction
  EXPECT_NEAR( actuator_eff, 2.0, 1e-9 );                   // identity
  EXPECT_NEAR( actuator_cur, 1.5, 1e-9 );                   // identity
}

TEST_F( AdjustableOffsetTransmissionTest, PassThroughEffort_JumpCorrectionStillWorks )
{
  // The 2pi jump correction must still trigger on actuator position even when
  // effort is configured as passthrough.
  double actuator_pos = 1.0, joint_pos = 0.0;
  double actuator_eff = 0.0, joint_eff = 0.0;
  AdjustableOffsetTransmission trans( joint_name, 2.0, 0.0,
                                      /*pass_through_effort=*/true );
  std::vector<transmission_interface::JointHandle> jh{
      { joint_name, hardware_interface::HW_IF_POSITION, &joint_pos },
      { joint_name, hardware_interface::HW_IF_EFFORT, &joint_eff } };
  std::vector<transmission_interface::ActuatorHandle> ah{
      { "actuator", hardware_interface::HW_IF_POSITION, &actuator_pos },
      { "actuator", hardware_interface::HW_IF_EFFORT, &actuator_eff } };
  trans.configure( jh, ah );

  trans.actuator_to_joint(); // baseline
  const double joint_pos_before = joint_pos;

  actuator_pos = 1.0 + 2.0 * M_PI;
  trans.actuator_to_joint();

  EXPECT_EQ( trans.getCorrectionCount(), 1 );
  EXPECT_NEAR( joint_pos, joint_pos_before, 1e-9 );
}
