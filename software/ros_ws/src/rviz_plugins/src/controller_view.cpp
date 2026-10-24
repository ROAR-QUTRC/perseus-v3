/// @file controller_view.cpp
/// @brief Implementation of ControllerView.

#include "rviz_plugins/controller_view.hpp"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace rviz_plugins
{

    namespace
    {
        // Joy indices, see the header.
        constexpr int AX_LX = 0, AX_LY = 1, AX_RX = 2, AX_RY = 3, AX_LT = 4, AX_RT = 5,
                      AX_DPAD_X = 6, AX_DPAD_Y = 7;
        constexpr int BTN_A = 0, BTN_B = 1, BTN_X = 2, BTN_Y = 3, BTN_LB = 4, BTN_RB = 5,
                      BTN_BACK = 6, BTN_START = 7, BTN_GUIDE = 8, BTN_LS = 9, BTN_RS = 10;

        const QColor BODY{70, 74, 82};
        const QColor BODY_EDGE{40, 43, 49};
        const QColor WELL{36, 38, 43};
        const QColor IDLE{105, 110, 120};
        const QColor ACTIVE{80, 200, 120};
        const QColor TEXT{225, 228, 233};

        // Design space the drawing is laid out in; scaled to fit the widget.
        constexpr double DESIGN_W = 420.0;
        constexpr double DESIGN_H = 300.0;
    }  // namespace

    ControllerView::ControllerView(QWidget* parent) : QWidget(parent)
    {
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void ControllerView::set_state(const std::vector<float>& axes,
                                   const std::vector<int32_t>& buttons)
    {
        _axis_count = axes.size();
        for (int i = 0; i < NUM_AXES; ++i)
            _axes[i] = i < static_cast<int>(axes.size()) ? axes[i] : 0.0F;
        for (int i = 0; i < NUM_BUTTONS; ++i)
            _buttons[i] = i < static_cast<int>(buttons.size()) && buttons[i] != 0;
        // Triggers rest at +1; a pad that never sent them should not show them pulled.
        if (_layout == Layout::Gamepad)
        {
            if (static_cast<int>(axes.size()) <= AX_LT) _axes[AX_LT] = 1.0F;
            if (static_cast<int>(axes.size()) <= AX_RT) _axes[AX_RT] = 1.0F;
        }
        update();
    }

    void ControllerView::clear_state()
    {
        _axes.fill(0.0F);
        _axis_count = 0;
        if (_layout == Layout::Gamepad)
        {
            _axes[AX_LT] = 1.0F;
            _axes[AX_RT] = 1.0F;
        }
        _buttons.fill(false);
        update();
    }

    void ControllerView::set_layout(Layout layout)
    {
        _layout = layout;
        update();
    }

    void ControllerView::set_connected(bool connected)
    {
        if (_connected == connected) return;
        _connected = connected;
        update();
    }

    float ControllerView::_axis(int index) const { return _axes[index]; }
    bool ControllerView::_button(int index) const { return _buttons[index]; }

    void ControllerView::_draw_stick(QPainter& p, const QPointF& centre, double radius, float x,
                                     float y, bool pressed) const
    {
        p.setPen(QPen(BODY_EDGE, 2));
        p.setBrush(WELL);
        p.drawEllipse(centre, radius, radius);
        // Joy: stick left = +X and up = +Y; the screen's x grows rightwards, y downwards.
        const double travel = radius * 0.45;
        const QPointF knob = centre + QPointF(-x * travel, -y * travel);
        p.setPen(QPen(pressed ? ACTIVE.darker(150) : BODY_EDGE, 2));
        p.setBrush(pressed ? ACTIVE : IDLE);
        p.drawEllipse(knob, radius * 0.62, radius * 0.62);
    }

    void ControllerView::_draw_trigger(QPainter& p, const QRectF& rect, float value,
                                       const QString& label) const
    {
        p.setPen(QPen(BODY_EDGE, 1.5));
        p.setBrush(WELL);
        p.drawRoundedRect(rect, 4, 4);
        // Rest = +1, fully pulled = -1.
        const double pulled = std::clamp((1.0 - value) / 2.0, 0.0, 1.0);
        QRectF fill = rect.adjusted(2, 2, -2, -2);
        fill.setTop(fill.bottom() - fill.height() * pulled);
        p.setPen(Qt::NoPen);
        p.setBrush(ACTIVE);
        p.drawRoundedRect(fill, 3, 3);
        p.setPen(TEXT);
        p.drawText(rect, Qt::AlignCenter, label);
    }

    void ControllerView::_draw_bumper(QPainter& p, const QRectF& rect, bool pressed,
                                      const QString& label) const
    {
        p.setPen(QPen(BODY_EDGE, 1.5));
        p.setBrush(pressed ? ACTIVE : IDLE);
        p.drawRoundedRect(rect, 6, 6);
        p.setPen(pressed ? Qt::black : TEXT);
        p.drawText(rect, Qt::AlignCenter, label);
    }

    void ControllerView::_draw_round_button(QPainter& p, const QPointF& centre, double radius,
                                            bool pressed, const QString& label,
                                            const QColor& colour) const
    {
        p.setPen(QPen(BODY_EDGE, 1.5));
        p.setBrush(pressed ? colour : WELL);
        p.drawEllipse(centre, radius, radius);
        p.setPen(pressed ? Qt::black : colour);
        p.drawText(QRectF(centre.x() - radius, centre.y() - radius, 2 * radius, 2 * radius),
                   Qt::AlignCenter, label);
    }

    void ControllerView::paintEvent(QPaintEvent*)
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        const double scale = std::min(width() / DESIGN_W, height() / DESIGN_H);
        p.translate((width() - DESIGN_W * scale) / 2.0, (height() - DESIGN_H * scale) / 2.0);
        p.scale(scale, scale);
        if (!_connected) p.setOpacity(0.35);

        QFont font = p.font();
        font.setPixelSize(12);
        font.setBold(true);
        p.setFont(font);

        if (_layout == Layout::Radio)
            _paint_radio(p);
        else
            _paint_gamepad(p);

        p.setOpacity(1.0);
        if (!_connected)
        {
            p.setPen(TEXT);
            p.drawText(QRectF(0, 250, DESIGN_W, 40), Qt::AlignCenter, "No /joy messages");
        }
    }

    void ControllerView::_paint_gamepad(QPainter& p) const
    {
        // Triggers and bumpers sit behind the body.
        _draw_trigger(p, QRectF(70, 6, 56, 38), _axis(AX_LT), "LT");
        _draw_trigger(p, QRectF(294, 6, 56, 38), _axis(AX_RT), "RT");
        _draw_bumper(p, QRectF(62, 50, 80, 22), _button(BTN_LB), "LB");
        _draw_bumper(p, QRectF(278, 50, 80, 22), _button(BTN_RB), "RB");

        // Body: a wide rounded centre with two grips.
        QPainterPath body;
        body.addRoundedRect(QRectF(50, 68, 320, 140), 60, 60);
        body.addEllipse(QRectF(40, 120, 110, 170));
        body.addEllipse(QRectF(270, 120, 110, 170));
        p.setPen(QPen(BODY_EDGE, 2));
        p.setBrush(BODY);
        p.drawPath(body.simplified());

        // Left stick (upper left), D-pad (lower left), right stick (lower right),
        // face buttons (upper right) -- the Xbox arrangement.
        _draw_stick(p, QPointF(112, 118), 30, _axis(AX_LX), _axis(AX_LY),
                    _button(BTN_LS));
        _draw_stick(p, QPointF(268, 190), 30, _axis(AX_RX), _axis(AX_RY), _button(BTN_RS));

        // D-pad: four arms lit by the hat axes (X: left = +1, Y: up = +1).
        const QPointF dc(152, 190);
        const struct
        {
            QPointF offset;
            bool on;
        } arms[] = {{{0, -20}, _axis(AX_DPAD_Y) > 0.5F},
                    {{0, 20}, _axis(AX_DPAD_Y) < -0.5F},
                    {{-20, 0}, _axis(AX_DPAD_X) > 0.5F},
                    {{20, 0}, _axis(AX_DPAD_X) < -0.5F}};
        p.setPen(QPen(BODY_EDGE, 1.5));
        for (const auto& arm : arms)
        {
            p.setBrush(arm.on ? ACTIVE : WELL);
            p.drawRoundedRect(QRectF(dc.x() + arm.offset.x() - 11, dc.y() + arm.offset.y() - 11,
                                     22, 22),
                              4, 4);
        }
        p.setBrush(WELL);
        p.drawRect(QRectF(dc.x() - 11, dc.y() - 11, 22, 22));

        // Face buttons.
        const QPointF fc(308, 118);
        _draw_round_button(p, fc + QPointF(0, -26), 14, _button(BTN_Y), "Y", QColor(240, 200, 60));
        _draw_round_button(p, fc + QPointF(0, 26), 14, _button(BTN_A), "A", QColor(80, 200, 120));
        _draw_round_button(p, fc + QPointF(-26, 0), 14, _button(BTN_X), "X", QColor(90, 150, 240));
        _draw_round_button(p, fc + QPointF(26, 0), 14, _button(BTN_B), "B", QColor(230, 80, 80));

        // Centre cluster: Back, Guide, Start.
        _draw_round_button(p, QPointF(210, 100), 15, _button(BTN_GUIDE), "X", TEXT);
        _draw_round_button(p, QPointF(180, 128), 9, _button(BTN_BACK), "", TEXT);
        _draw_round_button(p, QPointF(240, 128), 9, _button(BTN_START), "", TEXT);
    }

    void ControllerView::_draw_gimbal(QPainter& p, const QRectF& box, float x, float y,
                                      const QString& label) const
    {
        p.setPen(QPen(BODY_EDGE, 2));
        p.setBrush(WELL);
        p.drawRoundedRect(box, 8, 8);
        // Crosshair.
        p.setPen(QPen(IDLE, 1, Qt::DashLine));
        p.drawLine(QPointF(box.left() + 6, box.center().y()), QPointF(box.right() - 6, box.center().y()));
        p.drawLine(QPointF(box.center().x(), box.top() + 6), QPointF(box.center().x(), box.bottom() - 6));
        const double half = box.width() / 2.0 - 12.0;
        const QPointF knob = box.center() + QPointF(-std::clamp(x, -1.0F, 1.0F) * half,
                                                    -std::clamp(y, -1.0F, 1.0F) * half);
        p.setPen(QPen(BODY_EDGE, 2));
        p.setBrush(ACTIVE);
        p.drawEllipse(knob, 9, 9);
        p.setPen(TEXT);
        p.drawText(QRectF(box.left(), box.bottom() + 2, box.width(), 16), Qt::AlignCenter, label);
    }

    void ControllerView::_draw_switch(QPainter& p, const QPointF& top_left, float value,
                                      const QString& label) const
    {
        const QRectF slot(top_left.x(), top_left.y(), 18, 54);
        p.setPen(QPen(BODY_EDGE, 1.5));
        p.setBrush(WELL);
        p.drawRoundedRect(slot, 5, 5);
        // -1 / 0 / +1 -> top / middle / bottom detent.
        const double t = (std::clamp(value, -1.0F, 1.0F) + 1.0F) / 2.0;
        const QPointF knob(slot.center().x(), slot.top() + 9 + t * (slot.height() - 18));
        p.setBrush(value > 0.5F || value < -0.5F ? ACTIVE : IDLE);
        p.drawEllipse(knob, 7, 7);
        p.setPen(TEXT);
        p.drawText(QRectF(slot.left() - 15, slot.bottom() + 2, slot.width() + 30, 16),
                   Qt::AlignCenter, label);
    }

    void ControllerView::_paint_radio(QPainter& p) const
    {
        // Antennas and body.
        p.setPen(QPen(BODY_EDGE, 6, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(300, 30), QPointF(330, 4));
        p.setPen(QPen(BODY_EDGE, 2));
        p.setBrush(BODY);
        p.drawRoundedRect(QRectF(20, 24, 380, 230), 28, 28);

        // Gimbals: left = drive (X turn, Y forward), right = bucket (X tilt, Y lift).
        _draw_gimbal(p, QRectF(40, 120, 100, 100), _axis(2), _axis(0), "Drive");
        _draw_gimbal(p, QRectF(280, 120, 100, 100), _axis(1), _axis(3), "Bucket");

        // Switches (enable, turbo) and the jaws slider between the gimbals.
        _draw_switch(p, QPointF(160, 50), _axis(4), "Enable");
        _draw_switch(p, QPointF(242, 50), _axis(5), "Turbo");

        const QRectF rail(165, 160, 90, 12);
        p.setPen(QPen(BODY_EDGE, 1.5));
        p.setBrush(WELL);
        p.drawRoundedRect(rail, 6, 6);
        const double jaws = (std::clamp(_axis(6), -1.0F, 1.0F) + 1.0F) / 2.0;
        p.setBrush(ACTIVE);
        p.drawEllipse(QPointF(rail.left() + 8 + jaws * (rail.width() - 16), rail.center().y()), 8, 8);
        p.setPen(TEXT);
        p.drawText(QRectF(rail.left(), rail.bottom() + 8, rail.width(), 16), Qt::AlignCenter, "Jaws");

        // Any channels beyond the seven above, as thin centre-zero bars.
        const int extra = static_cast<int>(std::min<std::size_t>(_axis_count, NUM_AXES)) - 7;
        for (int i = 0; i < extra; ++i)
        {
            const QRectF bar(160 + 12.0 * i, 205, 8, 40);
            p.setPen(QPen(BODY_EDGE, 1));
            p.setBrush(WELL);
            p.drawRect(bar);
            const double v = std::clamp(_axis(7 + i), -1.0F, 1.0F);
            p.setPen(Qt::NoPen);
            p.setBrush(ACTIVE);
            p.drawRect(QRectF(bar.left(), bar.center().y() - std::max(0.0, v) * 20.0, 8,
                              std::abs(v) * 20.0));
        }
    }

}  // namespace rviz_plugins
