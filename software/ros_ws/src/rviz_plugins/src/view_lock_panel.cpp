/// @file view_lock_panel.cpp
/// @brief Implementation of ViewLockPanel.

#include "rviz_plugins/view_lock_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace rviz_plugins
{

    namespace
    {
        const QString FREE_VIEW_TYPE = "rviz_default_plugins/Orbit";
    }  // namespace

    ViewLockPanel::ViewLockPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        _lock_check = new QCheckBox("Lock view to");
        _lock_check->setToolTip(
            "Ticked: switch to the saved view and hold it there.\n"
            "Unticked: free Orbit camera on the fixed frame - it no longer follows "
            "the rover.");
        _view_combo = new QComboBox();
        _view_combo->setToolTip("Saved views from this RViz config");

        auto* row = new QHBoxLayout();
        row->addWidget(_lock_check);
        row->addWidget(_view_combo, 1);

        _status_label = new QLabel();
        _status_label->setStyleSheet("color: #888; font-size: 10px;");

        auto* layout = new QVBoxLayout();
        layout->addLayout(row);
        layout->addWidget(_status_label);
        layout->addStretch(1);
        setLayout(layout);

        connect(_lock_check, &QCheckBox::toggled, this, &ViewLockPanel::_on_lock_toggled);
        connect(_view_combo, QOverload<int>::of(&QComboBox::activated), this,
                &ViewLockPanel::_on_view_selected);
    }

    void ViewLockPanel::onInitialize()
    {
        _enforce_timer = new QTimer(this);
        connect(_enforce_timer, &QTimer::timeout, this, &ViewLockPanel::_enforce);
        _enforce_timer->start(ENFORCE_PERIOD_MS);
        _refresh_views();
    }

    void ViewLockPanel::_refresh_views()
    {
        auto* views = getDisplayContext() ? getDisplayContext()->getViewManager() : nullptr;
        if (views == nullptr)
            return;

        QStringList names;
        for (int i = 0; i < views->getNumViews(); ++i)
            names << views->getViewAt(i)->getName();

        QStringList current;
        for (int i = 0; i < _view_combo->count(); ++i)
            current << _view_combo->itemText(i);
        if (names == current)
            return;

        _view_combo->blockSignals(true);
        _view_combo->clear();
        _view_combo->addItems(names);
        const int index = _view_combo->findText(_wanted_view);
        if (index >= 0)
            _view_combo->setCurrentIndex(index);
        _view_combo->blockSignals(false);
        _view_combo->setEnabled(!names.isEmpty());
        if (names.isEmpty())
            _set_status("No saved views in this config", true);
    }

    rviz_common::ViewController* ViewLockPanel::_selected_view() const
    {
        auto* views = getDisplayContext() ? getDisplayContext()->getViewManager() : nullptr;
        if (views == nullptr)
            return nullptr;
        const QString name = _view_combo->currentText();
        for (int i = 0; i < views->getNumViews(); ++i)
        {
            if (views->getViewAt(i)->getName() == name)
                return views->getViewAt(i);
        }
        return nullptr;
    }

    bool ViewLockPanel::_matches(rviz_common::ViewController* saved) const
    {
        auto* current = getDisplayContext()->getViewManager()->getCurrent();
        if (current == nullptr || current->getClassId() != saved->getClassId())
            return false;
        // Same view type, so the same property list; compare them one by one. Name
        // is skipped: the current view is always called "Current View".
        for (int i = 0; i < saved->numChildren(); ++i)
        {
            const auto* saved_property = saved->childAt(i);
            if (saved_property->getName() == "Name")
                continue;
            const auto* current_property = current->subProp(saved_property->getName());
            if (current_property == nullptr ||
                current_property->getValue() != saved_property->getValue())
            {
                return false;
            }
        }
        return true;
    }

    void ViewLockPanel::_apply()
    {
        auto* saved = _selected_view();
        if (saved == nullptr)
        {
            _set_status("Saved view \"" + _view_combo->currentText() + "\" not found", true);
            return;
        }
        getDisplayContext()->getViewManager()->setCurrentFrom(saved);
    }

    void ViewLockPanel::_free()
    {
        auto* views = getDisplayContext()->getViewManager();
        // Changing the view type mimics the old camera, so it starts where it was.
        if (views->getCurrent() == nullptr ||
            views->getCurrent()->getClassId() != FREE_VIEW_TYPE)
        {
            views->setCurrentViewControllerType(FREE_VIEW_TYPE);
        }
        // The mimicked Orbit inherits the follower's target frame (base_footprint),
        // which would still drag the camera along with the rover. Re-targeting it to
        // the fixed frame detaches it; Orbit shifts its focal point to compensate, so
        // the camera does not jump.
        if (auto* frame = views->getCurrent()->subProp("Target Frame"))
            frame->setValue(rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING);
    }

    void ViewLockPanel::_on_lock_toggled(bool locked)
    {
        _view_combo->setEnabled(!locked && _view_combo->count() > 0);
        if (locked)
        {
            _apply();
            _set_status("Locked to " + _view_combo->currentText());
        }
        else
        {
            _free();
            _set_status("Free orbit camera on the fixed frame");
        }
    }

    void ViewLockPanel::_on_view_selected()
    {
        _wanted_view = _view_combo->currentText();
        if (_lock_check->isChecked())
            _apply();
    }

    void ViewLockPanel::_enforce()
    {
        _refresh_views();
        if (!_state_applied && getDisplayContext()->getViewManager()->getCurrent() != nullptr)
        {
            _state_applied = true;
            _on_lock_toggled(_lock_check->isChecked());
        }
        if (!_lock_check->isChecked())
            return;
        auto* saved = _selected_view();
        if (saved != nullptr && !_matches(saved))
        {
            _apply();
            _set_status("Locked to " + _view_combo->currentText());
        }
    }

    void ViewLockPanel::_set_status(const QString& text, bool error)
    {
        _status_label->setText(text);
        _status_label->setStyleSheet(QString("color: %1; font-size: 10px;")
                                         .arg(error ? "#b32424" : "#888"));
    }

    void ViewLockPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Locked", _lock_check->isChecked());
        config.mapSetValue("View", _view_combo->count() > 0 ? _view_combo->currentText()
                                                            : _wanted_view);
    }

    void ViewLockPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);
        config.mapGetString("View", &_wanted_view);
        _refresh_views();
        const int index = _view_combo->findText(_wanted_view);
        if (index >= 0)
            _view_combo->setCurrentIndex(index);
        // Only the checkbox here; the camera itself is switched on the first timer
        // tick, after RViz has loaded the config's current view (see _state_applied).
        bool locked = false;
        if (config.mapGetBool("Locked", &locked))
        {
            _lock_check->blockSignals(true);
            _lock_check->setChecked(locked);
            _lock_check->blockSignals(false);
            _view_combo->setEnabled(!locked && _view_combo->count() > 0);
        }
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::ViewLockPanel, rviz_common::Panel)
