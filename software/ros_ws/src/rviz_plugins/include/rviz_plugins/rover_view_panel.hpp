#pragma once

/// @file rover_view_panel.hpp
/// @brief RViz panel with a second, fixed 3D view of the rover from its front left.

#include <QTimer>
#include <cstdint>
#include <rviz_common/panel.hpp>

namespace Ogre
{
    class Light;
    class Viewport;
}  // namespace Ogre

namespace rviz_common
{
    class DisplayGroup;
    class RenderPanel;
    class ViewController;
}  // namespace rviz_common

namespace rviz_plugins
{

    /// @brief A small 3D view that keeps the rover in frame, seen from its front left,
    /// whatever the main view is doing.
    ///
    /// It draws the main scene, so it shows the RobotModel display: the wheels and the
    /// bucket's lift, tilt and jaw move in it as the joint states and TF say they do.
    /// Needs a RobotModel display enabled, and nothing else. It shows only that: the
    /// panel has a visibility bit of its own, its viewport draws only objects that
    /// carry it, and the bit is cleared on every object not in the RobotModel's subtree
    /// (clouds, maps, TF, grid).
    /// The main view is unaffected.
    ///
    /// The camera is an Orbit view on the rover's frame, starting from the front left,
    /// and is attached to the rover rigidly, roll and pitch included, so the model
    /// itself never moves in the view; only its joints do.
    /// It can be moved with the mouse like the main view (orbit, pan, zoom, with the
    /// Move Camera or Interact tool active), and it keeps that angle relative to the
    /// rover: Orbit's yaw is about the fixed frame, so each tick the rover's turn is
    /// added to it, which is what keeps the camera in place as the rover turns.
    class RoverViewPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit RoverViewPanel(QWidget* parent = nullptr);
        ~RoverViewPanel() override;

        void onInitialize() override;

    private Q_SLOTS:
        void _tick();

    private:
        /// @brief Leaves the panel's bit on the RobotModel's objects only, and clears
        /// it on everything else in the scene, so only the rover is drawn here. Run
        /// every tick, since a display makes new objects as data arrives.
        void _apply_visibility();

        /// @brief Sets each light's visibility for this view: this panel's own light on,
        /// every other off. Run every tick, since a window may add a light at any time.
        void _apply_lights();

        Ogre::Light* _light{nullptr};
        Ogre::Viewport* _main_viewport{nullptr};
        uint32_t _bit{0};
        bool _mask_set{false};
        double _heading{0.0};
        bool _heading_known{false};
        rviz_common::RenderPanel* _render_panel{nullptr};
        rviz_common::ViewController* _view{nullptr};
        QTimer* _timer{nullptr};
    };

}  // namespace rviz_plugins
