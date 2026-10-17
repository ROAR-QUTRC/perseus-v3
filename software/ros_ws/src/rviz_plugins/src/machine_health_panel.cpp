/// @file machine_health_panel.cpp
/// @brief Implementation of the machine health RViz panel.

#include "rviz_plugins/machine_health_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QVBoxLayout>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <vector>

namespace rviz_plugins
{
    namespace
    {

        /// @brief Column order of the table, kept in one place so the header and the
        /// row filling below cannot drift apart.
        enum table_columns
        {
            COLUMN_MACHINE = 0,
            COLUMN_CPU,
            COLUMN_MEMORY,
            COLUMN_TEMPERATURE,
            COLUMN_AGE,
            COLUMN_COUNT,
        };

        constexpr double CPU_WARN_PERCENT = 80.0;
        constexpr double CPU_CRITICAL_PERCENT = 95.0;
        constexpr double MEMORY_WARN_PERCENT = 80.0;
        constexpr double MEMORY_CRITICAL_PERCENT = 95.0;
        constexpr double TEMPERATURE_WARN_C = 70.0;
        constexpr double TEMPERATURE_CRITICAL_C = 85.0;

        constexpr double BYTES_PER_GIB = 1024.0 * 1024.0 * 1024.0;

        /// @brief Columns of the power bus table.
        enum bus_columns
        {
            BUS_COLUMN_NAME = 0,
            BUS_COLUMN_STATE,
            BUS_COLUMN_VOLTAGE,
            BUS_COLUMN_CURRENT,
            BUS_COLUMN_AGE,
            BUS_COLUMN_COUNT,
        };

        /// @brief The buses, in the order the rover control board has them, so they
        /// are listed even before the first status arrives.
        const std::vector<std::string> BUS_NAMES = {"compute", "drive", "aux", "spare"};

        /// @brief A bus's state, by the value in RcbPowerStatus.status; this is the
        /// board's power_status enum in hi_can_parameter.hpp, in the same order.
        const QStringList BUS_STATES = {"OFF", "ON", "PRECHARGING", "SHORT CIRCUIT",
                                        "SWITCH FAILED", "OVERLOAD", "FAULT"};
        constexpr int BUS_STATE_OFF = 0;
        constexpr int BUS_STATE_ON = 1;
        constexpr int BUS_STATE_PRECHARGING = 2;

        /// @brief The message carries millivolts and milliamps, as the board sends them.
        constexpr double MILLI_PER_UNIT = 1000.0;

    }  // namespace

    MachineHealthPanel::MachineHealthPanel(QWidget* parent)
        : rviz_common::Panel(parent)
    {
        _summary_label = new QLabel("Waiting for machines...");

        _table = new QTableWidget(0, COLUMN_COUNT);
        _table->setHorizontalHeaderLabels(
            {"Machine", "CPU (%)", "Memory", "Temp (C)", "Age (s)"});
        _table->horizontalHeader()->setSectionResizeMode(COLUMN_MACHINE,
                                                         QHeaderView::Stretch);
        _table->horizontalHeader()->setStyleSheet(
            "QHeaderView::section { font-weight: normal; }");
        _table->verticalHeader()->setVisible(false);
        _table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        _table->setSelectionBehavior(QAbstractItemView::SelectRows);

        _bus_label = new QLabel("Rover control board power: waiting for /can_to_ros...");

        _bus_table = new QTableWidget(static_cast<int>(BUS_NAMES.size()), BUS_COLUMN_COUNT);
        _bus_table->setHorizontalHeaderLabels(
            {"Bus", "State", "Voltage (V)", "Current (A)", "Age (s)"});
        _bus_table->horizontalHeader()->setSectionResizeMode(BUS_COLUMN_NAME,
                                                             QHeaderView::Stretch);
        _bus_table->horizontalHeader()->setStyleSheet(
            "QHeaderView::section { font-weight: normal; }");
        _bus_table->verticalHeader()->setVisible(false);
        _bus_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        _bus_table->setSelectionBehavior(QAbstractItemView::SelectRows);

        // The two tables sit side by side, each under its own summary line.
        auto* machines_column = new QVBoxLayout;
        machines_column->addWidget(_summary_label);
        machines_column->addWidget(_table, 1);

        auto* buses_column = new QVBoxLayout;
        buses_column->addWidget(_bus_label);
        buses_column->addWidget(_bus_table, 1);

        auto* layout = new QHBoxLayout;
        layout->addLayout(machines_column, 3);
        layout->addLayout(buses_column, 2);
        setLayout(layout);

        _refresh_timer = new QTimer(this);
        connect(_refresh_timer, &QTimer::timeout, this, &MachineHealthPanel::_refresh);
        _discovery_timer = new QTimer(this);
        connect(_discovery_timer, &QTimer::timeout, this,
                &MachineHealthPanel::_discover);
    }

    void MachineHealthPanel::onInitialize()
    {
        _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        _bus_subscription = _node->create_subscription<interfaces::msg::RcbPowerStatus>(
            RCB_TOPIC, rclcpp::QoS(SUBSCRIPTION_QUEUE_DEPTH),
            [this](interfaces::msg::RcbPowerStatus::ConstSharedPtr message)
            { _on_bus_status(std::move(message)); });

        _discover();
        _discovery_timer->start(DISCOVERY_PERIOD_MS);
        _refresh_timer->start(REFRESH_PERIOD_MS);
    }

    void MachineHealthPanel::_discover()
    {
        for (const auto& [topic, types] : _node->get_topic_names_and_types())
        {
            const bool is_machine_topic =
                topic.size() > TOPIC_SUFFIX.size() &&
                topic.compare(topic.size() - TOPIC_SUFFIX.size(), TOPIC_SUFFIX.size(),
                              TOPIC_SUFFIX) == 0 &&
                std::find(types.begin(), types.end(), TOPIC_TYPE) != types.end();
            if (!is_machine_topic || _subscriptions.count(topic) != 0)
            {
                continue;
            }

            _subscriptions[topic] =
                _node->create_subscription<interfaces::msg::MachineHealth>(
                    topic, rclcpp::QoS(SUBSCRIPTION_QUEUE_DEPTH),
                    [this, topic](interfaces::msg::MachineHealth::ConstSharedPtr message)
                    {
                        _on_health(topic, message);
                    });
        }
    }

    void MachineHealthPanel::_on_health(
        const std::string& topic, interfaces::msg::MachineHealth::ConstSharedPtr message)
    {
        const std::lock_guard<std::mutex> lock(_state_mutex);
        _machines[topic] = {std::move(message), std::chrono::steady_clock::now()};
    }

    void MachineHealthPanel::_on_bus_status(
        interfaces::msg::RcbPowerStatus::ConstSharedPtr message)
    {
        const std::lock_guard<std::mutex> lock(_bus_mutex);
        const std::string name = message->name;
        _buses[name] = {std::move(message), std::chrono::steady_clock::now()};
    }

    QColor MachineHealthPanel::_level_color(double value, double warn, double critical)
    {
        // Same palette as TopicHealthPanel, so the two panels read alike.
        if (value >= critical)
        {
            return QColor(245, 150, 150);
        }
        if (value >= warn)
        {
            return QColor(255, 200, 120);
        }
        return QColor(200, 240, 200);
    }

    void MachineHealthPanel::_refresh_buses()
    {
        std::map<std::string, BusState> buses;
        {
            const std::lock_guard<std::mutex> lock(_bus_mutex);
            buses = _buses;
        }

        const auto now = std::chrono::steady_clock::now();
        const QColor no_data_color(230, 230, 230);
        const QColor stale_color(245, 150, 150);
        const QColor ok_color(200, 240, 200);
        const QColor busy_color(255, 200, 120);
        const QColor fault_color(245, 150, 150);

        int fault_count = 0;
        int stale_count = 0;
        for (int row = 0; row < static_cast<int>(BUS_NAMES.size()); ++row)
        {
            const auto found = buses.find(BUS_NAMES[row]);

            QString values[BUS_COLUMN_COUNT] = {QString::fromStdString(BUS_NAMES[row]),
                                                "N/A", "N/A", "N/A", "N/A"};
            QColor colors[BUS_COLUMN_COUNT] = {no_data_color, no_data_color, no_data_color,
                                               no_data_color, no_data_color};
            if (found != buses.end())
            {
                const auto& message = *found->second.message;
                const double age_sec =
                    std::chrono::duration<double>(now - found->second.received_at).count();
                const bool stale = age_sec > RCB_STALE_AFTER_SEC;
                stale_count += stale ? 1 : 0;

                const int state = message.status;
                const bool known_state = state >= 0 && state < BUS_STATES.size();
                const bool faulted = known_state && state > BUS_STATE_PRECHARGING;
                fault_count += (faulted && !stale) ? 1 : 0;

                values[BUS_COLUMN_STATE] =
                    known_state ? BUS_STATES[state] : QString("UNKNOWN (%1)").arg(state);
                values[BUS_COLUMN_VOLTAGE] =
                    QString::number(message.voltage / MILLI_PER_UNIT, 'f', 2);
                values[BUS_COLUMN_CURRENT] =
                    QString::number(message.current / MILLI_PER_UNIT, 'f', 2);
                values[BUS_COLUMN_AGE] = QString::number(age_sec, 'f', 1);

                // A bus that has gone quiet shows its last reading in red, like a machine:
                // an old "ON" in green would say the bus is fine when nothing says so.
                QColor state_color = fault_color;
                if (known_state && state == BUS_STATE_ON)
                    state_color = ok_color;
                else if (known_state && state == BUS_STATE_OFF)
                    state_color = no_data_color;
                else if (known_state && state == BUS_STATE_PRECHARGING)
                    state_color = busy_color;
                colors[BUS_COLUMN_STATE] = state_color;
                colors[BUS_COLUMN_VOLTAGE] = ok_color;
                colors[BUS_COLUMN_CURRENT] = ok_color;
                colors[BUS_COLUMN_AGE] = ok_color;
                if (stale)
                    for (auto& color : colors)
                        color = stale_color;
            }

            for (int column = 0; column < BUS_COLUMN_COUNT; ++column)
            {
                auto* item = _bus_table->item(row, column);
                if (item == nullptr)
                {
                    item = new QTableWidgetItem;
                    _bus_table->setItem(row, column, item);
                }
                item->setText(values[column]);
                item->setBackground(colors[column]);
                item->setForeground(QColor(20, 20, 20));
            }
        }

        _bus_label->setText(
            buses.empty()
                ? QString("Rover control board power: waiting for %1...")
                      .arg(QString::fromStdString(RCB_TOPIC))
                : QString("Rover control board power    %1 fault(s)    %2 stale")
                      .arg(fault_count)
                      .arg(stale_count));
    }

    void MachineHealthPanel::_refresh()
    {
        _refresh_buses();

        std::map<std::string, MachineState> machines;
        {
            const std::lock_guard<std::mutex> lock(_state_mutex);
            machines = _machines;
        }
        if (machines.empty())
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const QColor no_data_color(230, 230, 230);
        const QColor stale_color(245, 150, 150);

        int stale_count = 0;
        _table->setRowCount(static_cast<int>(machines.size()));
        int row = 0;
        for (const auto& [topic, state] : machines)
        {
            const auto& message = *state.message;
            const double age_sec =
                std::chrono::duration<double>(now - state.received_at).count();
            const bool stale = age_sec > STALE_AFTER_SEC;
            stale_count += stale ? 1 : 0;

            // Values the monitor could not measure are negative (or zero memory total),
            // and show as "N/A" on a neutral background instead of a fake reading.
            const bool has_cpu = message.cpu_percent >= 0.0F;
            const bool has_memory = message.memory_percent >= 0.0F;
            const bool has_temperature = message.temperature_c >= 0.0F;

            // The panel is keyed on topic, so the topic prefix is what identifies the
            // machine even if a message carried a stale or empty device_name.
            QString machine = QString::fromStdString(
                topic.substr(1, topic.size() - TOPIC_SUFFIX.size() - 1));
            if (machine.isEmpty())
            {
                machine = QString::fromStdString(message.device_name);
            }

            const QString values[COLUMN_COUNT] = {
                machine,
                has_cpu ? QString::number(message.cpu_percent, 'f', 1) : QString("N/A"),
                has_memory
                    ? QString("%1% (%2/%3 GiB)")
                          .arg(message.memory_percent, 0, 'f', 1)
                          .arg(static_cast<double>(message.memory_used_bytes) /
                                   BYTES_PER_GIB,
                               0, 'f', 1)
                          .arg(static_cast<double>(message.memory_total_bytes) /
                                   BYTES_PER_GIB,
                               0, 'f', 1)
                    : QString("N/A"),
                has_temperature ? QString::number(message.temperature_c, 'f', 1)
                                : QString("N/A"),
                QString::number(age_sec, 'f', 1),
            };
            const QColor colors[COLUMN_COUNT] = {
                stale ? stale_color : no_data_color,
                has_cpu ? _level_color(message.cpu_percent, CPU_WARN_PERCENT,
                                       CPU_CRITICAL_PERCENT)
                        : no_data_color,
                has_memory ? _level_color(message.memory_percent, MEMORY_WARN_PERCENT,
                                          MEMORY_CRITICAL_PERCENT)
                           : no_data_color,
                has_temperature ? _level_color(message.temperature_c, TEMPERATURE_WARN_C,
                                               TEMPERATURE_CRITICAL_C)
                                : no_data_color,
                stale ? stale_color : QColor(200, 240, 200),
            };

            for (int column = 0; column < COLUMN_COUNT; ++column)
            {
                auto* item = _table->item(row, column);
                if (item == nullptr)
                {
                    item = new QTableWidgetItem;
                    _table->setItem(row, column, item);
                }
                item->setText(values[column]);
                // A stale machine's readings are old, so the whole row goes red rather than
                // leaving an OK-looking green on a machine that has gone quiet.
                item->setBackground(stale ? stale_color : colors[column]);
                item->setForeground(QColor(20, 20, 20));
            }
            ++row;
        }

        _summary_label->setText(QString("%1 machine(s)    %2 stale")
                                    .arg(machines.size())
                                    .arg(stale_count));
    }

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::MachineHealthPanel, rviz_common::Panel)
