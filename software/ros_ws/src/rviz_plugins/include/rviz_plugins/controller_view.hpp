#pragma once

/// @file controller_view.hpp
/// @brief Painted gamepad diagram that mirrors a sensor_msgs/Joy message.

#include <QPointF>
#include <QRectF>
#include <QWidget>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

class QPainter;

namespace rviz_plugins
{

    /// @brief An Xbox-style controller drawn like QGroundControl's joystick
    /// calibration page: sticks move, triggers fill, buttons and D-pad light up.
    ///
    /// Layout follows the ROS 2 joy_node mapping for an Xbox pad (the one teleop's
    /// controller configs assume):
    ///   axes     0 LX, 1 LY, 2 RX, 3 RY, 4 LT, 5 RT, 6 D-pad X, 7 D-pad Y
    ///   buttons  0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 Guide,
    ///            9 L-stick click, 10 R-stick click
    /// Triggers rest at +1 and go to -1 when pulled; sticks read up as +1 on Y.
    ///
    /// Radio layout follows teleop/config/taranis_controller.yaml:
    ///   axes  2 / 0  left gimbal  X / Y   (drive: turn / forward)
    ///         1 / 3  right gimbal X / Y   (bucket: tilt / lift)
    ///         4, 5   three-position switches (drive enable, turbo)
    ///         6      slider (jaws)
    ///   Any further axes are shown as thin bars. Gimbals draw +1 upwards/left.
    ///
    /// Purely a view: set_state() stores the latest values and repaints.
    class ControllerView : public QWidget
    {
        Q_OBJECT

    public:
        /// @brief What kind of controller is drawn.
        enum class Layout
        {
            Gamepad,  ///< Xbox-style pad
            Radio     ///< RF transmitter (Taranis), channels as teleop's taranis_controller.yaml
        };

        explicit ControllerView(QWidget* parent = nullptr);

        /// @brief Switches the picture; the same Joy message is read either way.
        void set_layout(Layout layout);

        /// @brief Replaces the displayed state; missing axes/buttons read as 0.
        void set_state(const std::vector<float>& axes, const std::vector<int32_t>& buttons);

        /// @brief Clears everything back to rest (no controller / stale data).
        void clear_state();

        /// @brief Dims the picture when no Joy message is arriving.
        void set_connected(bool connected);

        QSize sizeHint() const override { return {420, 300}; }
        QSize minimumSizeHint() const override { return {280, 200}; }

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        static constexpr int NUM_BUTTONS = 11;
        static constexpr int NUM_AXES = 16;

        void _paint_gamepad(QPainter& p) const;
        void _paint_radio(QPainter& p) const;
        void _draw_gimbal(QPainter& p, const QRectF& box, float x, float y,
                          const QString& label) const;
        void _draw_switch(QPainter& p, const QPointF& top_left, float value,
                          const QString& label) const;

        float _axis(int index) const;
        bool _button(int index) const;

        void _draw_stick(QPainter& p, const QPointF& centre, double radius, float x, float y,
                         bool pressed) const;
        void _draw_trigger(QPainter& p, const QRectF& rect, float value, const QString& label) const;
        void _draw_bumper(QPainter& p, const QRectF& rect, bool pressed, const QString& label) const;
        void _draw_round_button(QPainter& p, const QPointF& centre, double radius, bool pressed,
                                const QString& label, const QColor& colour) const;

        std::array<float, NUM_AXES> _axes{};
        std::array<bool, NUM_BUTTONS> _buttons{};
        bool _connected{false};
        Layout _layout{Layout::Gamepad};
        std::size_t _axis_count{0};  // axes in the last message, for the extra-axes bars
    };

}  // namespace rviz_plugins
