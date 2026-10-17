/// @file rover_view_panel.cpp
/// @brief Implementation of RoverViewPanel.

#include "rviz_plugins/rover_view_panel.hpp"

#include <OgreMovableObject.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreViewport.h>

#include <QVBoxLayout>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/bit_allocator.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <set>

namespace rviz_plugins
{

    namespace
    {
        // The frame the camera follows, and takes the heading from.
        const std::string ROVER_FRAME = "base_footprint";
        // Rover-forward is +x, so front-left is 45 deg round from it.
        constexpr double FRONT_LEFT_YAW = M_PI / 4.0;
        // Almost level and close in, so the perspective is strong and each near wheel
        // hides the far-side wheel on its axle; the bucket arms still read as angles.
        constexpr double PITCH_RAD = 0.1;
        constexpr double DISTANCE_M = 1.8;
        constexpr int REFRESH_PERIOD_MS = 33;
        // The camera's heading follows the rover's with this time constant. The pose
        // from odometry jitters a little even at rest, and the camera turns on the
        // heading, so unsmoothed that shows as the whole view shaking.
        constexpr double HEADING_SMOOTHING_S = 0.25;
        const QString ROBOT_MODEL_CLASS = "rviz_default_plugins/RobotModel";
    }  // namespace

    RoverViewPanel::RoverViewPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        auto* layout = new QVBoxLayout();
        layout->setContentsMargins(0, 0, 0, 0);
        setLayout(layout);
    }

    RoverViewPanel::~RoverViewPanel()
    {
        // The window holds the controller's camera, so it goes first.
        delete _render_panel;
        delete _view;
        if (_bit != 0 && getDisplayContext() != nullptr)
        {
            Ogre::MovableObject::setDefaultVisibilityFlags(
                Ogre::MovableObject::getDefaultVisibilityFlags() | _bit);
            getDisplayContext()->visibilityBits()->freeBits(_bit);
        }
    }

    void RoverViewPanel::onInitialize()
    {
        auto* context = getDisplayContext();

        _render_panel = new rviz_common::RenderPanel(this);
        // true: draw the main scene, so the RobotModel display shows up here too.
        _render_panel->initialize(context, true);
        layout()->addWidget(_render_panel);

        // Draw only what carries this bit, which is the RobotModel alone. The window has
        // no viewport yet, so the mask is set from _tick once it does.
        _bit = context->visibilityBits()->allocBit();
        // Objects made from here on start without the bit, so a new cloud never shows
        // for even one frame; _apply_visibility gives it back to the RobotModel's.
        Ogre::MovableObject::setDefaultVisibilityFlags(
            Ogre::MovableObject::getDefaultVisibilityFlags() & ~_bit);
        _apply_visibility();

        _view = context->getViewManager()->create("rviz_default_plugins/Orbit");
        _view->subProp("Target Frame")->setValue(QString::fromStdString(ROVER_FRAME));
        _view->subProp("Pitch")->setValue(PITCH_RAD);
        _view->subProp("Distance")->setValue(DISTANCE_M);
        _render_panel->setViewController(_view);

        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &RoverViewPanel::_tick);
        _timer->start(REFRESH_PERIOD_MS);
    }

    namespace
    {
        // Sets or clears @p bit on every object under @p node, but not under a node in
        // @p keep, which is left as it is. Lights are skipped: Ogre tests their flags
        // against the viewport's mask too, so clearing one would unlight the rover.
        void set_bit(Ogre::SceneNode* node, uint32_t bit, bool on,
                     const std::set<Ogre::SceneNode*>& keep)
        {
            if (keep.count(node) != 0)
                return;
            auto objects = node->getAttachedObjectIterator();
            while (objects.hasMoreElements())
            {
                Ogre::MovableObject* object = objects.getNext();
                if (object->getMovableType() == "Light")
                    continue;
                const uint32_t flags = object->getVisibilityFlags();
                object->setVisibilityFlags(on ? (flags | bit) : (flags & ~bit));
            }
            auto children = node->getChildIterator();
            while (children.hasMoreElements())
                set_bit(static_cast<Ogre::SceneNode*>(children.getNext()), bit, on, keep);
        }

        // The scene nodes of the RobotModel displays under @p group.
        void collect_robot_nodes(rviz_common::DisplayGroup* group,
                                 std::set<Ogre::SceneNode*>& nodes)
        {
            for (int i = 0; i < group->numDisplays(); ++i)
            {
                auto* display = group->getDisplayAt(i);
                if (auto* nested = dynamic_cast<rviz_common::DisplayGroup*>(display))
                    collect_robot_nodes(nested, nodes);
                else if (display->getClassId() == ROBOT_MODEL_CLASS)
                    nodes.insert(display->getSceneNode());
            }
        }
    }  // namespace

    void RoverViewPanel::_apply_visibility()
    {
        auto* context = getDisplayContext();
        std::set<Ogre::SceneNode*> robot_nodes;
        collect_robot_nodes(context->getRootDisplayGroup(), robot_nodes);

        // Not just the displays' own nodes: a point cloud hangs its objects off a node
        // of its own under the scene root. So clear the bit on the whole scene, leaving
        // the RobotModel's subtree, then set it there.
        const std::set<Ogre::SceneNode*> none;
        set_bit(context->getSceneManager()->getRootSceneNode(), _bit, false, robot_nodes);
        for (auto* node : robot_nodes)
            set_bit(node, _bit, true, none);
    }

    void RoverViewPanel::_tick()
    {
        if (_view == nullptr)
            return;

        if (!_mask_set)
        {
            auto* viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
                _render_panel->getRenderWindow());
            if (viewport != nullptr)
            {
                viewport->setVisibilityMask(_bit);
                _mask_set = true;
            }
        }

        // Every tick: a display makes new objects as its data arrives, and a cloud is a
        // new object per message.
        _apply_visibility();

        // The rover's heading in the fixed frame; with no transform yet the camera
        // keeps its last yaw.
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (getDisplayContext()->getFrameManager()->getTransform(ROVER_FRAME, position,
                                                                 orientation))
        {
            const double heading = std::atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
            const double dt_s = REFRESH_PERIOD_MS / 1000.0;
            auto* yaw = _view->subProp("Yaw");
            if (!_heading_known)
            {
                _heading = heading;
                _heading_known = true;
                yaw->setValue(_heading + FRONT_LEFT_YAW);
            }
            else
            {
                // Through the short way round, so a turn across +-pi does not spin it.
                const double error = std::remainder(heading - _heading, 2.0 * M_PI);
                const double step = error * (1.0 - std::exp(-dt_s / HEADING_SMOOTHING_S));
                _heading += step;
                // Add the rover's turn to the camera's yaw rather than setting it, so
                // wherever the user has orbited to stays the same relative to the rover.
                yaw->setValue(yaw->getValue().toDouble() + step);
            }
        }

        const float dt = REFRESH_PERIOD_MS / 1000.0f;
        _view->update(dt, dt);
        _render_panel->getRenderWindow()->renderLater();
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::RoverViewPanel, rviz_common::Panel)
