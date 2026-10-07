#ifndef BUCKET_HARDWARE__CAN_BOARD_INTERFACE_HPP_
#define BUCKET_HARDWARE__CAN_BOARD_INTERFACE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <hi_can_raw.hpp>  // RawCanInterface, Packet, PacketManager
#include <memory>
#include <mutex>
#include <string>

namespace payloads
{

    // Bank = one CAN-addressable actuator group.
    // LIFT and JAWS each drive two actuators from a single bank command
    // (deliberately independent. per-side control is possible in hardware but
    // is disallowed over CAN to prevent torsion/twisting of the bucket).
    // TILT has one actuator but is still read from two encoders across its axle.
    //
    // Values here should match
    // excavation::bucket::controller::bank_group's underlying values
    // (LIFT=0x00, TILT=0x01, JAWS=0x02) so casts between the two are direct.
    enum class Axis : uint8_t
    {
        LIFT = 0,
        TILT = 1,
        JAWS = 2
    };

    // Side identifies one of the two magnetic angle encoders on an axle. It is
    // ONLY used for encoder feedback (encoder_group is per-side) - it plays no
    // part in commanding an actuator, since commands are sent to the whole bank.
    enum class Side : uint8_t
    {
        LEFT = 0,
        RIGHT = 1
    };

    constexpr size_t kNumAxes = 3;

    /// SET_SPEED duty (int16, +/-32767 = 100%) that takes a bank out of position
    /// mode without moving it. The firmware ignores SET_SPEEDs inside +/-1% while
    /// in position mode (so teleop's stream of zeros can't cancel a setpoint), which
    /// means a plain zero does NOT stop a bank that is driving to a SET_POSITION.
    /// 2% is just outside that deadband and well under the ~10% the actuators need
    /// to move at all; a zero follows it. Same trick as STOP_NUDGE_PERCENT in
    /// firmware/excavation-bucket/testbench/bucket_tui.py.
    constexpr int16_t kStopNudgeDuty = 655;
    constexpr size_t kNumSides = 2;

    inline size_t index_of(Axis axis, Side side)
    {
        return static_cast<size_t>(axis) * kNumSides + static_cast<size_t>(side);
    }

    /// Wrap an angle in degrees into (-180, 180].
    inline double wrap_degrees(double degrees)
    {
        double wrapped = std::fmod(degrees + 180.0, 360.0);
        if (wrapped <= 0.0)
        {
            wrapped += 360.0;
        }
        return wrapped - 180.0;
    }

    /// One encoder's last known reading (encoder_group::GET_ANGLE, broadcast by the
    /// board every 50 ms). This is the authoritative per-side position, used for each
    /// joint's `position` state interface.
    ///
    /// `degrees` is in the firmware's frame (its calibrated zero and direction),
    /// wrapped to (-180, 180]. BucketHardware maps it into the URDF joint frame.
    ///
    /// There is no hardware velocity readback; BucketHardware differentiates it.
    struct EncoderState
    {
        double degrees = 0.0;
        bool stale = true;  // true until the first real message arrives
    };

    /// One bank's last known reading. Current/fault are bank_parameters, so there is
    /// exactly one of each per axis (not per side). Both side of LIFT/JAWS should
    /// readthe same BankState
    ///
    /// average_degrees comes from bank_parameter::GET_POSITION, which the firmware
    /// reports as the average of that bank's two encoders. It is NOT a substitute
    /// for the per-side EncoderState readings. It should be used as a diagnostic
    /// cross-check (flagging skew, webui, etc.), never as a joint's position source
    ///
    /// `fault` is a plain bool (matches
    /// hi_can::parameters::excavation::bucket::controller::status_t,
    /// which wraps a single bool), not a bitfield. If the firmware needs to reports
    /// more than one fault condition, status_t will change upstream first.
    struct BankState
    {
        double average_degrees = 0.0;  // firmware frame, bank_parameter::GET_POSITION
        double current = 0.0;          // amps, bank_parameter::GET_CURRENT
        bool fault = 0;                // bank_parameter::GET_FAULT
        bool stale = true;
    };

    /// Invoked by the board decodes a new encoder_group::GET_ANGLE frame
    using EncoderUpdateCallback =
        std::function<void(Axis axis, Side side, const EncoderState& state)>;

    /// Invoked whenever the board decodes a new bank-level frame
    /// (GET_CURRENT / GET_FAULT / GET_POSITION-as-average)
    using BankUpdateCallback = std::function<void(Axis axis, const BankState& state)>;

    /// Thin adapter between ros2_control's read()/write() and hi-can.
    ///
    /// Unlike a threaded driver, hi_can::PacketManager is polled: nothing
    /// arrives asynchronously on a background thread, so poll() MUST be called
    /// once per control cycle (from BucketHardware::read(), before any joint
    /// state is read out) or encoder/bank state will simply never update.
    ///
    /// Sends bypass PacketManager's scheduled-transmission machinery - command
    /// setpoints are event-driven off the control loop, not a fixed interval, so
    /// send_position_command()/stop_axis() build and transmit a
    /// Packet directly via the underlying RawCanInterface.
    ///
    /// Nothing outside this file needs to know how CAN framing, IDs, or
    /// scaling work; bucket_hardware.cpp only ever talks to Axis/Side/
    /// AxisCommand/AxisState.
    class CanBoardInterface
    {
    public:
        CanBoardInterface() = default;
        ~CanBoardInterface();

        /// Open the CAN connection and register receive filters. Called once from
        /// BucketHardware::on_configure(). Returns false (rather than throwing) if
        /// hi_can::RawCanInterface's constructor throws - see .cpp for why.
        bool connect(const std::string& can_interface_name);

        /// Close the CAN connection. Called from BucketHardware::on_cleanup() /
        /// on_shutdown()
        void disconnect();

        /// Pump receive processing. MUST be called once per control cycle (from
        /// BucketHardware::read()) - this decodes incoming frames and fires
        /// on_encoder_received()/on_bank_received() and PacketManager's timeout
        /// callbacks. Does nothing if not connected.
        ///
        /// The board broadcasts GET_ANGLE and GET_POSITION every 50 ms
        /// (firmware motor_parameter_group.cpp), so nothing needs requesting.
        void poll();

        /// Register the function called whenever a new per-encoder angle frame is
        /// decoded (or when it goes stale via PacketManager's timeout callback).
        void register_encoder_callback(EncoderUpdateCallback callback);

        /// Register the function called whenever a new bank-level
        /// (current/fault/average-angle) frame is decoded, or when one goes stale.
        void register_bank_callback(BankUpdateCallback callback);

        /// Command a bank to a target angle in the firmware's frame, in degrees, via
        /// bank_parameter::SET_POSITION. There is intentionally no per-side overload -
        /// both actuators in a bank always receive the same setpoint. Called from
        /// BucketHardware::write() when the position command interface is claimed.
        void send_position_command(Axis axis, double degrees);

        /// Stop a bank immediately and drop it out of position mode: a
        /// bank_parameter::SET_SPEED of kStopNudgeDuty, then one of zero.
        ///
        /// These are the only SET_SPEEDs this class sends - general speed control
        /// belongs to the teleop driver (bucket_driver). Called from
        /// BucketHardware::on_deactivate() and when a controller releases a joint,
        /// so the bucket stops promptly rather than driving on toward its last
        /// setpoint, which the firmware never times out.
        void stop_axis(Axis axis);

        /// One-shot: zero a bank's encoder reference (bank_parameter::SET_ZERO_POS).
        /// Call during on_configure()/on_activate(), not every write() cycle.
        void zero_axis(Axis axis);

        // TODO: bank_parameter::SET_PID_PARAMS

        /// Latest known reading for one encoder. read() in BucketHardware pulls
        /// from here after calling poll(), it does NOT talk to CAN directly.
        EncoderState get_last_encoder_state(Axis axis, Side side) const;

        /// Latest known reading for one bank (current/fault/average angle).
        BankState get_last_bank_state(Axis axis) const;

    private:
        // Registers one hi_can::PacketManager::set_callback() per encoder (6) and
        // per bank read-parameter (3 axes x {GET_CURRENT, GET_FAULT, GET_POSITION} =
        // 9), wired to on_encoder_received()/on_bank_received(). Called once from
        // connect().
        void register_receive_filters();

        void on_encoder_received(Axis axis, Side side, const EncoderState& state);
        void on_bank_received(Axis axis, const BankState& state);

        mutable std::mutex state_mutex_;
        std::array<EncoderState, kNumAxes * kNumSides> last_encoder_state_;
        std::array<BankState, kNumAxes> last_bank_state_;

        EncoderUpdateCallback encoder_cb_;
        BankUpdateCallback bank_cb_;

        std::unique_ptr<hi_can::RawCanInterface> can_interface_;
        std::unique_ptr<hi_can::PacketManager> packet_manager_;

        bool connected_ = false;
    };

}  // namespace payloads

#endif  // BUCKET_HARDWARE__CAN_BOARD_INTERFACE_HPP_