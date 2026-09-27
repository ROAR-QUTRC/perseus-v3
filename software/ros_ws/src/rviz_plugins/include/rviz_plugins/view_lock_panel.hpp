#pragma once

/// @file view_lock_panel.hpp
/// @brief RViz panel that locks the camera to one of the config's saved views.

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QString>
#include <QTimer>
#include <rviz_common/panel.hpp>

namespace rviz_common
{
    class ViewController;
}

namespace rviz_plugins
{

    /// @brief One checkbox instead of the whole Views panel.
    ///
    ///   Ticked    the camera switches to a saved view (the config's "Far" by
    ///             default -- a ThirdPersonFollower on base_footprint, so it follows
    ///             the rover) and is held there.
    ///   Unticked  the camera becomes a free Orbit view attached to the fixed frame
    ///             (odom), so it can be orbited, panned and zoomed anywhere and stays
    ///             put while the rover drives off. It starts from wherever the camera
    ///             was, so unticking does not make the view jump.
    ///
    /// Saved views belong to RViz's ViewManager and live in the config's
    /// Visualization Manager / Views / Saved section, not in the Views panel, so they
    /// survive that panel being removed -- this panel only reads them.
    ///
    /// Holding is done by comparison, not by blocking input: RViz gives a panel no
    /// way to stop the view controller from handling mouse events, so while locked a
    /// timer checks the current camera against the saved view and restores it when
    /// they differ. A drag therefore shows for a moment before snapping back.
    class ViewLockPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit ViewLockPanel(QWidget* parent = nullptr);

        void onInitialize() override;

        /// @brief Persists the lock state and the chosen view's name.
        void save(rviz_common::Config config) const override;

        /// @brief Restores them, and re-applies the lock if it was on.
        void load(const rviz_common::Config& config) override;

    private Q_SLOTS:
        void _on_lock_toggled(bool locked);
        void _on_view_selected();
        /// @brief While locked, restores the saved view if the camera has moved.
        void _enforce();

    private:
        static inline const QString DEFAULT_VIEW = "Far";
        static constexpr int ENFORCE_PERIOD_MS = 100;

        /// @brief Refills the dropdown from the ViewManager's saved views, keeping
        /// the current choice if it still exists.
        void _refresh_views();

        /// @brief The saved view named in the dropdown, or nullptr.
        rviz_common::ViewController* _selected_view() const;

        /// @brief True if the current camera matches @p saved in every property
        /// except its name.
        bool _matches(rviz_common::ViewController* saved) const;

        /// @brief Switches to the selected saved view.
        void _apply();

        /// @brief Switches the camera to an Orbit view on the fixed frame, keeping
        /// its current position.
        void _free();
        void _set_status(const QString& text, bool error = false);

        QCheckBox* _lock_check{nullptr};
        QComboBox* _view_combo{nullptr};
        QLabel* _status_label{nullptr};
        QTimer* _enforce_timer{nullptr};

        // The name to select once the saved views are known: load() runs before the
        // ViewManager has necessarily filled its list.
        QString _wanted_view{DEFAULT_VIEW};

        // Whether the loaded lock state has been applied yet. RViz loads the current
        // view from the config after the panels, so doing it in load() would just be
        // overwritten; the first timer tick does it instead.
        bool _state_applied{false};
    };

}  // namespace rviz_plugins
