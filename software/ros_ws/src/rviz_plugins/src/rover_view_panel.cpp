/// @file rover_view_panel.cpp
/// @brief Implementation of RoverViewPanel.

#include "rviz_plugins/rover_view_panel.hpp"

#include <OgreColourValue.h>
#include <OgreLight.h>
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
        // The flat frame the camera orbits and takes its heading from, and the rover's
        // own frame, which carries its roll and pitch.
        const std::string ROVER_FRAME = "base_footprint";
        const std::string BODY_FRAME = "base_link";
        // Rover-forward is +x, so front-left is 45 deg round from it.
        constexpr double FRONT_LEFT_YAW = M_PI / 4.0;
        // Almost level and close in, so the perspective is strong and each near wheel
        // hides the far-side wheel on its axle; the bucket arms still read as angles.
        constexpr double PITCH_RAD = 0.15;
        constexpr double DISTANCE_M = 3.5;
        constexpr int REFRESH_PERIOD_MS = 33;
        const QString ROBOT_MODEL_CLASS = "rviz_default_plugins/RobotModel";
        // Direction the panel's own light shines, in the camera's frame (+x right, +y up,
        // looking down -z): from above-left of the camera, forward and down. Fixed to the
        // camera, so the rover is lit the same from wherever the camera is orbited to.
        const Ogre::Vector3 LIGHT_DIRECTION_IN_CAMERA(0.3, -0.6, -1.0);
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
        if (_light != nullptr && getDisplayContext() != nullptr)
        {
            getDisplayContext()->getSceneManager()->destroyLight(_light);
            _light = nullptr;
        }
        if (_main_viewport != nullptr && _bit != 0)
        {
            _main_viewport->setVisibilityMask(_main_viewport->getVisibilityMask() | _bit);
        }
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

        // The panel's own light. RViz gives each render window a headlight, all in the one
        // scene, so the main view's light shines on this view too and turns with the main
        // camera: the rover's brightness followed whatever the main view was doing. So
        // this view ignores every other light (_apply_lights) and uses this one, which is
        // on for this view's bit only and moves with this camera.
        auto* scene = context->getSceneManager();
        _light = scene->createLight();
        _light->setType(Ogre::Light::LT_DIRECTIONAL);
        _light->setDiffuseColour(Ogre::ColourValue(1.0F, 1.0F, 1.0F));
        _light->setSpecularColour(Ogre::ColourValue(0.4F, 0.4F, 0.4F));
        _light->setVisibilityFlags(_bit);
        scene->getRootSceneNode()->attachObject(_light);

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

    void RoverViewPanel::_apply_lights()
    {
        auto lights = getDisplayContext()->getSceneManager()->getMovableObjectIterator("Light");
        while (lights.hasMoreElements())
        {
            auto* light = static_cast<Ogre::Light*>(lights.getNext());
            // Every light but this panel's own is hidden from this view: the lights of
            // the other render windows, which turn with those windows' cameras.
            const uint32_t flags = light->getVisibilityFlags();
            light->setVisibilityFlags(light == _light ? _bit : (flags & ~_bit));
        }
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

                // The panel's light carries only this view's bit, so the main view must
                // not take it: its own mask drops the bit, which leaves everything else,
                // the rover included (it has every bit), drawn there as before.
                if (auto* main_panel = getDisplayContext()->getViewManager()->getRenderPanel())
                {
                    _main_viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
                        main_panel->getRenderWindow());
                    if (_main_viewport != nullptr)
                        _main_viewport->setVisibilityMask(_main_viewport->getVisibilityMask() &
                                                          ~_bit);
                }
            }
        }

        // Every tick: a display makes new objects as its data arrives, and a cloud is a
        // new object per message.
        _apply_visibility();
        _apply_lights();

        // The rover's heading in the fixed frame; with no transform yet the camera
        // keeps its last yaw.
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        Ogre::Quaternion flat_orientation;
        bool have_flat = false;
        if (getDisplayContext()->getFrameManager()->getTransform(ROVER_FRAME, position,
                                                                 orientation))
        {
            const double heading = std::atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
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
                // Not smoothed: the camera is attached to the rover below, and that
                // only holds if the yaw added here is exactly the heading it removes.
                const double step = std::remainder(heading - _heading, 2.0 * M_PI);
                _heading += step;
                // Add the rover's turn to the camera's yaw rather than setting it, so
                // wherever the user has orbited to stays the same relative to the rover.
                yaw->setValue(yaw->getValue().toDouble() + step);
            }
            flat_orientation = orientation;
            have_flat = true;
        }

        const float dt = REFRESH_PERIOD_MS / 1000.0f;
        _view->update(dt, dt);

        // Fixed relative to the camera, so the brightness cannot depend on the angle.
        _light->setDirection(_view->getCamera()->getDerivedOrientation() *
                             LIGHT_DIRECTION_IN_CAMERA.normalisedCopy());

        // Attach the camera to the rover. The orbit above is about the flat frame, so
        // on its own the rover would roll and pitch in the view with the terrain. The
        // camera's own scene node (which holds the orbit offset) hangs off a target node
        // that the controller places at the flat frame; moving that target node to the
        // rover's own frame, turned by the rover's roll and pitch (its orientation with
        // the flat frame's taken out), carries the camera along rigidly. Nothing of the
        // rover's own pose is then left in the picture: only its joints move.
        Ogre::Vector3 body_position;
        Ogre::Quaternion body_orientation;
        auto* camera = _view->getCamera();
        auto* camera_node = camera != nullptr ? camera->getParentSceneNode() : nullptr;
        auto* target_node = camera_node != nullptr ? camera_node->getParent() : nullptr;
        if (have_flat && target_node != nullptr &&
            getDisplayContext()->getFrameManager()->getTransform(BODY_FRAME, body_position,
                                                                 body_orientation))
        {
            target_node->setPosition(body_position);
            target_node->setOrientation(body_orientation * flat_orientation.Inverse());
        }

        _render_panel->getRenderWindow()->renderLater();
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::RoverViewPanel, rviz_common::Panel)
