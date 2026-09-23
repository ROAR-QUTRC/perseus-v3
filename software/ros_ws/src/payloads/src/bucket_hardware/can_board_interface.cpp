#include "bucket_hardware/can_board_interface.hpp"

#include <cmath>
#include <hi_can_address.hpp>
#include <hi_can_parameter.hpp>
#include <iostream>

namespace payloads
{

    namespace
    {

        using namespace hi_can;  // NOLINT
        namespace bucket_addr = addressing::excavation::bucket::controller;
        namespace bucket_param = parameters::excavation::bucket::controller;

        const uint8_t CAN_TIMEOUT_INTERVAL = 100;  // in milliseconds

        const addressing::standard_address_t kDeviceAddress{
            addressing::excavation::SYSTEM_ID, addressing::excavation::bucket::SUBSYSTEM_ID,
            bucket_addr::DEVICE_ID};

        bucket_addr::bank_group bank_group_for(Axis axis)
        {
            // Axis's underlying values are defined to match bank_group's exactly.
            return static_cast<bucket_addr::bank_group>(axis);
        }

        bucket_addr::encoder_group encoder_group_for(Axis axis, Side side)
        {
            // Not a clean arithmetic progression from (axis, side) worth using
            // Instead using an explicit table against real enum values.
            static constexpr bucket_addr::encoder_group kTable[kNumAxes][kNumSides] = {
                /* LIFT */ {bucket_addr::encoder_group::LIFT_L, bucket_addr::encoder_group::LIFT_R},
                /* TILT */ {bucket_addr::encoder_group::TILT_L, bucket_addr::encoder_group::TILT_R},
                /* JAWS */ {bucket_addr::encoder_group::JAWS_L, bucket_addr::encoder_group::JAWS_R},
            };
            return kTable[static_cast<size_t>(axis)][static_cast<size_t>(side)];
        }

        addressing::standard_address_t bank_address(Axis axis, bucket_addr::bank_parameter parameter)
        {
            return addressing::standard_address_t{
                kDeviceAddress, static_cast<uint8_t>(bank_group_for(axis)),
                static_cast<uint8_t>(parameter)};
        }

        addressing::standard_address_t encoder_address(
            Axis axis, Side side, bucket_addr::encoder_parameter parameter)
        {
            return addressing::standard_address_t{
                kDeviceAddress, static_cast<uint8_t>(encoder_group_for(axis, side)),
                static_cast<uint8_t>(parameter)};
        }

        // TODO: confirm against the encoder firmware - position_t's
        // raw int16_t is a degree not radians. Placeholder assumes
        // 1850 = 180.5 degrees. This must match whatever the firmware itself
        // uses to produce/consume the int16_t defined in hi_can_parameter.hpp
        constexpr double kCountsPerRadian = 10.0;

        // TODO: confirm against h-bridge current-sense circuit's actual scale -
        // current_t's raw uint16_t is unscaled. Placeholder: 1 count = 1 mA.
        // NOTE: This can change depending on firmware implementation
        constexpr double kCountsPerAmp = 1000.0;

        double decode_position(const std::vector<uint8_t>& data)
        {
            bucket_param::position_t param(data);
            return static_cast<double>(param.value) / kCountsPerRadian;
        }

        std::vector<uint8_t> encode_position(double position)
        {
            bucket_param::position_t param{
                static_cast<int16_t>(std::lround(position * kCountsPerRadian))};
            return param.serialize_data();
        }

        std::vector<uint8_t> encode_speed(double velocity)
        {
            // TODO: confirm speed_t's scale the same way as position - placeholder
            // reuses kCountsPerRadian as counts-per-(radian/s); almost certainly
            // wrong, needs the firmware's actual speed unit.
            bucket_param::speed_t param{static_cast<int16_t>(std::lround(velocity * kCountsPerRadian))};
            return param.serialize_data();
        }

        double decode_current(const std::vector<uint8_t>& data)
        {
            bucket_param::current_t param(data);
            return static_cast<double>(param.value) / kCountsPerAmp;
        }

        bool decode_fault(const std::vector<uint8_t>& data)
        {
            bucket_param::status_t param(data);
            return param.value;
        }

        // Registers one PacketManager callback for an exact-match address. The
        // default filter_t mask already matches every bit of `address`
        // (addressing::MASK_ALL), so no mask needs to be set explicitly.
        void set_frame_callback(
            PacketManager& manager, const addressing::standard_address_t& address,
            std::function<void(const Packet&)> data_callback,
            std::function<void()> timeout_callback = nullptr)
        {
            addressing::filter_t filter{};
            filter.address = addressing::flagged_address_t(address);

            PacketManager::callback_config_t config{};
            config.data_callback = std::move(data_callback);
            if (timeout_callback)
            {
                // 2 missed frames before timeout fires, per the Hi-CAN spec - so this is
                // the *expected inter-frame gap*, not the total time-to-timeout.
                // TODO: tune to the board's actual broadcast interval.
                config.timeout = std::chrono::milliseconds(CAN_TIMEOUT_INTERVAL);
                config.timeout_callback = std::move(timeout_callback);
            }
            manager.set_callback(filter, config);
        }

    }  // namespace

    CanBoardInterface::~CanBoardInterface()
    {
        if (connected_)
        {
            disconnect();
        }
    }

    bool CanBoardInterface::connect(const std::string& can_interface_name)
    {
        // hi_can uses exceptions for unusual errors (e.g. the named CAN interface
        // not existing) rather than a bool return - translate that into the bool
        // API the rest of BucketHardware expects.
        try
        {
            can_interface_ = std::make_unique<hi_can::RawCanInterface>(can_interface_name);
            packet_manager_ = std::make_unique<hi_can::PacketManager>(*can_interface_);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to open CAN interface '" << can_interface_name << "': " << e.what()
                      << std::endl;
            can_interface_.reset();
            packet_manager_.reset();
            return false;
        }

        register_receive_filters();
        connected_ = true;
        return true;
    }

    void CanBoardInterface::disconnect()
    {
        packet_manager_.reset();
        can_interface_.reset();
        connected_ = false;
    }

    void CanBoardInterface::poll()
    {
        if (!connected_)
        {
            return;
        }
        // Non-blocking: decodes any buffered frames, fires matching callbacks
        // (and any due timeout callbacks), then returns immediately.
        // Nothing here schedules transmissions, since commands are sent directly
        // via send_position_command()/send_velocity_command() rather than through
        // PacketManager's interval-transmission machinery.
        packet_manager_->handle_receive();
    }

    void CanBoardInterface::register_encoder_callback(EncoderUpdateCallback callback)
    {
        encoder_cb_ = std::move(callback);
    }

    void CanBoardInterface::register_bank_callback(BankUpdateCallback callback)
    {
        bank_cb_ = std::move(callback);
    }

    void CanBoardInterface::register_receive_filters()
    {
        // --- Per-encoder GET_ANGLE frames (6 total) ---
        for (uint8_t axis_idx = 0; axis_idx < kNumAxes; ++axis_idx)
        {
            for (uint8_t side_idx = 0; side_idx < kNumSides; ++side_idx)
            {
                const auto axis = static_cast<Axis>(axis_idx);
                const auto side = static_cast<Side>(side_idx);
                const auto address = encoder_address(
                    axis, side, bucket_addr::encoder_parameter::GET_ANGLE);

                set_frame_callback(
                    *packet_manager_, address,
                    [this, axis, side](const Packet& frame)
                    {
                        EncoderState state;
                        state.position = decode_position(frame.get_data());
                        state.stale = false;
                        this->on_encoder_received(axis, side, state);
                    },
                    /*timeout_callback=*/
                    [this, axis, side]()
                    {
                        auto state = get_last_encoder_state(axis, side);
                        state.stale = true;
                        this->on_encoder_received(axis, side, state);
                    });
            }
        }

        // --- Per-bank GET_CURRENT / GET_FAULT / GET_ANGLE(average) frames ---
        for (uint8_t axis_idx = 0; axis_idx < kNumAxes; ++axis_idx)
        {
            const auto axis = static_cast<Axis>(axis_idx);

            auto update_bank = [this, axis](std::function<void(BankState&)> apply)
            {
                auto state = get_last_bank_state(axis);
                apply(state);
                state.stale = false;
                this->on_bank_received(axis, state);
            };

            set_frame_callback(
                *packet_manager_, bank_address(axis, bucket_addr::bank_parameter::GET_CURRENT),
                [update_bank](const Packet& frame)
                {
                    const double current = decode_current(frame.get_data());
                    update_bank([current](BankState& s)
                                { s.current = current; });
                });

            set_frame_callback(
                *packet_manager_, bank_address(axis, bucket_addr::bank_parameter::GET_FAULT),
                [update_bank](const Packet& frame)
                {
                    const bool fault = decode_fault(frame.get_data());
                    update_bank([fault](BankState& s)
                                { s.fault = fault; });
                });

            set_frame_callback(
                *packet_manager_, bank_address(axis, bucket_addr::bank_parameter::GET_ANGLE),
                [update_bank](const Packet& frame)
                {
                    const double angle = decode_position(frame.get_data());
                    update_bank([angle](BankState& s)
                                { s.average_position = angle; });
                });
        }
    }  // register_receive_filters()

    void CanBoardInterface::send_position_command(Axis axis, double position)
    {
        if (!can_interface_)
        {
            return;
        }
        const auto address = bank_address(axis, bucket_addr::bank_parameter::SET_ANGLE);
        Packet packet(addressing::flagged_address_t(address), encode_position(position));
        can_interface_->transmit(packet);
    }

    void CanBoardInterface::send_velocity_command(Axis axis, double velocity)
    {
        if (!can_interface_)
        {
            return;
        }
        const auto address = bank_address(axis, bucket_addr::bank_parameter::SET_SPEED);
        Packet packet(addressing::flagged_address_t(address), encode_speed(velocity));
        can_interface_->transmit(packet);
    }

    void CanBoardInterface::zero_axis(Axis axis)
    {
        if (!can_interface_)
        {
            return;
        }
        // TODO: confirm SET_ZERO_POS expects an empty payload - no dedicated
        // parameter type exists for it in hi_can_parameter.hpp to check against.
        // See can_board_interface.hpp for type defs.
        const auto address = bank_address(axis, bucket_addr::bank_parameter::SET_ZERO_POS);
        Packet packet(addressing::flagged_address_t(address), {});
        can_interface_->transmit(packet);
    }

    void CanBoardInterface::reset_axis(Axis axis)
    {
        if (!can_interface_)
        {
            return;
        }
        // TODO: confirm RESET_TO_ZERO expects an empty payload - same caveat as
        // zero_axis().
        // See can_board_interface.hpp for type defs.
        const auto address = bank_address(axis, bucket_addr::bank_parameter::RESET_TO_ZERO);
        Packet packet(addressing::flagged_address_t(address), {});
        can_interface_->transmit(packet);
    }

    void CanBoardInterface::set_axis_sleep(Axis axis, bool sleep)
    {
        if (!can_interface_)
        {
            return;
        }
        // TODO: confirm SET_SLEEP reuses status_t (bool) - no dedicated type is
        // declared for it in hi_can_parameter.hpp, only GET_FAULT is explicitly
        // tied to status_t.
        bucket_param::status_t param{sleep};
        const auto address = bank_address(axis, bucket_addr::bank_parameter::SET_SLEEP);
        Packet packet(addressing::flagged_address_t(address), param.serialize_data());
        can_interface_->transmit(packet);
    }

    EncoderState CanBoardInterface::get_last_encoder_state(Axis axis, Side side) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return last_encoder_state_[index_of(axis, side)];
    }

    BankState CanBoardInterface::get_last_bank_state(Axis axis) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return last_bank_state_[static_cast<size_t>(axis)];
    }

    void CanBoardInterface::on_encoder_received(Axis axis, Side side, const EncoderState& state)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_encoder_state_[index_of(axis, side)] = state;
        }
        if (encoder_cb_)
        {
            encoder_cb_(axis, side, state);
        }
    }

    void CanBoardInterface::on_bank_received(Axis axis, const BankState& state)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_bank_state_[static_cast<size_t>(axis)] = state;
        }
        if (bank_cb_)
        {
            bank_cb_(axis, state);
        }
    }

}  // namespace payloads