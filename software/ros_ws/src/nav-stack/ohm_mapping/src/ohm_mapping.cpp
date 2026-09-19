// Copyright 2026 ROAR QUTRC
#include "ohm_mapping/ohm_mapping.hpp"

#include <ohm/Aabb.h>
#include <ohm/MapFlag.h>
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohmheightmap/Heightmap.h>
#include <ohmheightmap/HeightmapMode.h>
#include <ohmheightmap/HeightmapVoxel.h>
#include <ohmheightmap/HeightmapVoxelType.h>
#include <ohmheightmap/UpAxis.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ohm_mapping
{
    namespace
    {

        /// nav_msgs/OccupancyGrid's "no information" value.
        constexpr int8_t kUnknownCell = -1;
        constexpr int8_t kLethalCell = 100;

        ohm::HeightmapMode parseMode(const std::string& name, const rclcpp::Logger& logger)
        {
            if (name == "planar")
            {
                return ohm::HeightmapMode::kPlanar;
            }
            if (name == "simple_fill")
            {
                return ohm::HeightmapMode::kSimpleFill;
            }
            if (name == "layered_fill_unordered")
            {
                return ohm::HeightmapMode::kLayeredFillUnordered;
            }
            if (name == "layered_fill")
            {
                return ohm::HeightmapMode::kLayeredFill;
            }
            RCLCPP_WARN(
                logger, "Unknown heightmap mode '%s'; falling back to planar.", name.c_str());
            return ohm::HeightmapMode::kPlanar;
        }

        /// Median height of the REAL (measured, non-virtual) surface cells within
        /// `radius_cells` of (col, row), or false if there are none.
        ///
        /// Median rather than max: one spurious high return on a pit rim would drag a
        /// max-based reference up and turn the surrounding floor into a phantom depression.
        bool localRealSurfaceImpl(const std::vector<float>& height,
                                  const std::vector<uint8_t>& is_virtual, int span, int col,
                                  int row, int radius_cells, double* out)
        {
            std::vector<float> samples;
            for (int r = std::max(0, row - radius_cells);
                 r <= std::min(span - 1, row + radius_cells); ++r)
            {
                for (int c = std::max(0, col - radius_cells);
                     c <= std::min(span - 1, col + radius_cells); ++c)
                {
                    const auto i =
                        static_cast<size_t>(r) * static_cast<size_t>(span) + static_cast<size_t>(c);
                    if (is_virtual[i] == 0U && !std::isnan(height[i]))
                    {
                        samples.push_back(height[i]);
                    }
                }
            }
            if (samples.empty())
            {
                return false;
            }
            const auto mid = static_cast<std::ptrdiff_t>(samples.size() / 2);
            std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
            *out = static_cast<double>(samples[static_cast<size_t>(mid)]);
            return true;
        }

    }  // namespace

    OhmMapping::OhmMapping(const rclcpp::NodeOptions& options)
        : rclcpp::Node("ohm_mapping", options)
    {
        declareParameters();

        // kVoxelMean gives sub-voxel surface positions, which is what stops a 0.15 m voxel from
        // quantising the ground into 0.15 m steps -- the heightmap reads those refined positions
        // rather than voxel centres. kCompressed keeps idle regions compressed off-thread, which
        // is the difference between a bounded map and a slowly growing one on a 16 GB board.
        const auto flags = ohm::MapFlag::kVoxelMean | ohm::MapFlag::kCompressed;
        map_ = std::make_unique<ohm::OccupancyMap>(cfg_.resolution_m, flags);
        map_->setHitProbability(static_cast<float>(cfg_.hit_probability));
        map_->setMissProbability(static_cast<float>(cfg_.miss_probability));

        mapper_ = std::make_unique<ohm::RayMapperOccupancy>(map_.get());
        if (!mapper_->valid())
        {
            // Only reachable if the occupancy layer is missing, which cannot happen for a map
            // constructed as above -- but integrateRays() is documented as unsafe otherwise, so
            // this is checked rather than assumed.
            throw std::runtime_error("OHM ray mapper failed to validate against the occupancy map");
        }

        heightmap_ = std::make_unique<ohm::Heightmap>(
            cfg_.heightmap_resolution_m, cfg_.min_clearance_m, ohm::UpAxis::kZ);
        heightmap_->setOccupancyMap(map_.get());
        heightmap_->setMode(parseMode(cfg_.mode, get_logger()));
        heightmap_->setGenerateVirtualSurface(cfg_.generate_virtual_surface);
        heightmap_->setVirtualSurfaceFilterThreshold(
            static_cast<unsigned>(std::max(0, cfg_.virtual_surface_filter_threshold)));
        if (cfg_.ceiling_m > 0.0)
        {
            heightmap_->setCeiling(cfg_.ceiling_m);
        }
        if (cfg_.floor_m > 0.0)
        {
            heightmap_->setFloor(cfg_.floor_m);
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Best effort on the cloud, matching how BIEVR-LIO advertises it and how the rest of
        // this stack treats sensor data: a dropped scan costs one update, a late one drags the
        // map behind the robot.
        const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(2);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg_.cloud_topic, cloud_qos,
            std::bind(&OhmMapping::cloudCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            cfg_.odom_topic, rclcpp::QoS(50),
            std::bind(&OhmMapping::odomCallback, this, std::placeholders::_1));

        // Transient local so a costmap or an RViz display that subscribes after a build still
        // gets the last grid instead of waiting up to 1/heightmap_rate_hz for the next one.
        const auto latched = rclcpp::QoS(1).transient_local();
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/heightmap", latched);
        if (cfg_.publish_cloud)
        {
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/heightmap_cloud", latched);
        }

        const auto period = std::chrono::duration<double>(1.0 / std::max(0.05, cfg_.heightmap_rate_hz));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OhmMapping::buildAndPublish, this));

        RCLCPP_INFO(
            get_logger(),
            "OHM mapping up: %.2f m voxels, %.2f m heightmap over +/-%.1f m at %.1f Hz, mode %s, "
            "virtual surfaces %s",
            cfg_.resolution_m, cfg_.heightmap_resolution_m, cfg_.heightmap_radius_m,
            cfg_.heightmap_rate_hz, cfg_.mode.c_str(),
            cfg_.generate_virtual_surface ? "on" : "off");
    }

    // Out of line, and not defaulted in the header: Config holds complete types but the three
    // OHM classes are forward declared there, so unique_ptr's deleter has to be instantiated
    // somewhere that has seen their definitions.
    OhmMapping::~OhmMapping() = default;

    void OhmMapping::declareParameters()
    {
        cfg_.cloud_topic = declare_parameter<std::string>("cloud_topic", "/bievr_lio/points/registered");
        cfg_.odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
        cfg_.world_frame = declare_parameter<std::string>("world_frame", "odom");
        cfg_.robot_frame = declare_parameter<std::string>("robot_frame", "base_link");
        cfg_.lidar_frame = declare_parameter<std::string>("lidar_frame", "livox_frame");
        cfg_.odom_match_tolerance_s = declare_parameter<double>("odom_match_tolerance_s", 0.05);
        cfg_.pending_timeout_s = declare_parameter<double>("pending_timeout_s", 0.5);

        cfg_.resolution_m = declare_parameter<double>("resolution_m", 0.15);
        cfg_.hit_probability = declare_parameter<double>("hit_probability", 0.7);
        cfg_.miss_probability = declare_parameter<double>("miss_probability", 0.4);
        cfg_.ray_max_range_m = declare_parameter<double>("ray_max_range_m", 15.0);
        cfg_.ray_stride = declare_parameter<int>("ray_stride", 1);
        cfg_.map_retain_radius_m = declare_parameter<double>("map_retain_radius_m", 30.0);

        cfg_.heightmap_resolution_m = declare_parameter<double>("heightmap_resolution_m", 0.10);
        cfg_.heightmap_radius_m = declare_parameter<double>("heightmap_radius_m", 10.0);
        cfg_.heightmap_rate_hz = declare_parameter<double>("heightmap_rate_hz", 1.0);
        cfg_.min_clearance_m = declare_parameter<double>("min_clearance_m", 0.6);
        cfg_.ground_margin_m = declare_parameter<double>("ground_margin_m", 0.15);
        cfg_.ceiling_m = declare_parameter<double>("ceiling_m", 2.0);
        cfg_.floor_m = declare_parameter<double>("floor_m", 2.0);
        cfg_.mode = declare_parameter<std::string>("mode", "planar");
        cfg_.generate_virtual_surface = declare_parameter<bool>("generate_virtual_surface", true);
        cfg_.virtual_surface_filter_threshold =
            declare_parameter<int>("virtual_surface_filter_threshold", 3);

        cfg_.max_slope_deg = declare_parameter<double>("max_slope_deg", 25.0);
        cfg_.slope_radius_m = declare_parameter<double>("slope_radius_m", 0.3);
        cfg_.min_slope_fit_cells = declare_parameter<int>("min_slope_fit_cells", 6);
        cfg_.virtual_surface_cost = declare_parameter<int>("virtual_surface_cost", 90);
        cfg_.virtual_flat_cost = declare_parameter<int>("virtual_flat_cost", 20);
        cfg_.virtual_drop_threshold_m = declare_parameter<double>("virtual_drop_threshold_m", 0.15);
        cfg_.virtual_reference_radius_m =
            declare_parameter<double>("virtual_reference_radius_m", 0.6);
        cfg_.low_clearance_cost = declare_parameter<int>("low_clearance_cost", 100);
        cfg_.publish_cloud = declare_parameter<bool>("publish_cloud", true);
    }

    void OhmMapping::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const double stamp_s = rclcpp::Time(msg->header.stamp).seconds();
        odom_history_.push_back({stamp_s, glm::dvec3(p.x, p.y, p.z)});
        while (odom_history_.size() > kOdomHistory)
        {
            odom_history_.pop_front();
        }
        last_position_ = glm::dvec3(p.x, p.y, p.z);
        have_pose_ = true;

        // The half that actually matters: this is normally the message a buffered cloud has
        // been waiting for.
        drainPending();
    }

    bool OhmMapping::robotPositionAt(const rclcpp::Time& stamp, glm::dvec3* position) const
    {
        if (odom_history_.empty())
        {
            return false;
        }
        const double target = stamp.seconds();
        double best_delta = std::numeric_limits<double>::infinity();
        const StampedPosition* best = nullptr;
        for (const auto& entry : odom_history_)
        {
            const double delta = std::abs(entry.stamp_s - target);
            if (delta < best_delta)
            {
                best_delta = delta;
                best = &entry;
            }
        }
        if (best == nullptr || best_delta > cfg_.odom_match_tolerance_s)
        {
            return false;
        }
        *position = best->position;
        return true;
    }

    bool OhmMapping::lidarOffset(glm::dvec3* offset)
    {
        if (lidar_offset_valid_)
        {
            *offset = lidar_offset_;
            return true;
        }
        // Static in practice (it comes off the URDF), so this resolves once and is then cached.
        // Asked for at time zero, i.e. "latest available", because a static transform has no
        // meaningful history to interpolate.
        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                cfg_.robot_frame, cfg_.lidar_frame, tf2::TimePointZero);
            lidar_offset_ = glm::dvec3(
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            lidar_offset_valid_ = true;
            RCLCPP_INFO(
                get_logger(), "Resolved %s -> %s sensor offset: (%.3f, %.3f, %.3f)",
                cfg_.robot_frame.c_str(), cfg_.lidar_frame.c_str(), lidar_offset_.x, lidar_offset_.y,
                lidar_offset_.z);
            *offset = lidar_offset_;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "No %s -> %s transform yet (%s); carving rays from the robot origin meanwhile.",
                cfg_.robot_frame.c_str(), cfg_.lidar_frame.c_str(), ex.what());
            *offset = glm::dvec3(0.0, 0.0, 0.0);
            return false;
        }
    }

    void OhmMapping::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        if (msg->width * msg->height == 0)
        {
            return;
        }
        // The cloud is published already registered into world_frame. Anything else would have
        // to be transformed here, and silently mapping a sensor-frame cloud as though it were
        // world-frame produces a map that looks plausible and is wrong, so it is refused.
        if (!msg->header.frame_id.empty() && msg->header.frame_id != cfg_.world_frame)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 10000,
                "Cloud on %s is in frame '%s', expected '%s'. Point it at a registered cloud "
                "(e.g. /bievr_lio/points/registered) or set world_frame to match.",
                cfg_.cloud_topic.c_str(), msg->header.frame_id.c_str(), cfg_.world_frame.c_str());
            return;
        }

        // Buffered rather than matched here: the odometry sharing this cloud's stamp has
        // almost certainly not arrived yet. See drainPending().
        pending_clouds_.push_back(msg);
        while (pending_clouds_.size() > kMaxPendingClouds)
        {
            pending_clouds_.pop_front();
        }
        drainPending();
    }

    void OhmMapping::drainPending()
    {
        while (!pending_clouds_.empty())
        {
            const auto& msg = pending_clouds_.front();
            const rclcpp::Time stamp(msg->header.stamp);

            glm::dvec3 robot_position{0.0, 0.0, 0.0};
            if (robotPositionAt(stamp, &robot_position))
            {
                glm::dvec3 sensor_offset{0.0, 0.0, 0.0};
                lidarOffset(&sensor_offset);
                // Translation only. The origin of a ray is a point, so the sensor's
                // orientation does not enter into it; the samples are already in world
                // coordinates.
                integrateCloud(*msg, robot_position + sensor_offset);
                pending_clouds_.pop_front();
                continue;
            }

            // No match yet. Either the odometry is still in flight -- the normal case, and
            // the reason for this buffer -- or it is never coming. Only the second is worth
            // reporting, and the newest odometry stamp is what distinguishes them: once
            // odometry has moved well past this cloud, no later message can match it.
            if (!odom_history_.empty() &&
                odom_history_.back().stamp_s - stamp.seconds() > cfg_.pending_timeout_s)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Dropping a scan: odometry has run %.2f s past it without ever landing "
                    "within %.0f ms of its stamp. Ray origins cannot be guessed -- without "
                    "them every ray would be traced from the world origin.",
                    odom_history_.back().stamp_s - stamp.seconds(),
                    cfg_.odom_match_tolerance_s * 1000.0);
                pending_clouds_.pop_front();
                continue;
            }
            // Oldest cloud cannot be resolved yet, so nothing behind it can be either.
            break;
        }
    }

    void OhmMapping::integrateCloud(const sensor_msgs::msg::PointCloud2& msg,
                                    const glm::dvec3& origin)
    {
        const int stride = std::max(1, cfg_.ray_stride);
        const double max_range = cfg_.ray_max_range_m;

        rays_.clear();
        rays_.reserve(2 * (msg.width * msg.height / static_cast<unsigned>(stride) + 1));

        sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

        int counter = 0;
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            if (counter++ % stride != 0)
            {
                continue;
            }
            const double px = static_cast<double>(*it_x);
            const double py = static_cast<double>(*it_y);
            const double pz = static_cast<double>(*it_z);
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
            {
                continue;
            }
            glm::dvec3 sample(px, py, pz);
            const glm::dvec3 delta = sample - origin;
            const double range = std::sqrt(glm::dot(delta, delta));
            if (range < 1e-6)
            {
                continue;
            }
            if (max_range > 0.0 && range > max_range)
            {
                // Clip rather than drop: the sample beyond honest range is not trustworthy, but the
                // free space between here and there was still measured. Truncating to just inside
                // max_range and letting it integrate as a sample would instead plant a phantom
                // surface, so the endpoint is pulled in and the ray is still carved as free.
                sample = origin + delta * (max_range / range);
                rays_.push_back(origin);
                rays_.push_back(sample);
                continue;
            }
            rays_.push_back(origin);
            rays_.push_back(sample);
        }

        if (rays_.empty())
        {
            return;
        }

        mapper_->integrateRays(rays_.data(), rays_.size());
        ++scans_integrated_;

        // Bound the map. Without this the region count grows for the whole run, and since the
        // heightmap walks the source map's extents the build cost grows with it.
        if (cfg_.map_retain_radius_m > 0.0)
        {
            map_->removeDistanceRegions(origin, static_cast<float>(cfg_.map_retain_radius_m));
        }
    }

    void OhmMapping::buildAndPublish()
    {
        if (!have_pose_ || scans_integrated_ == 0)
        {
            return;
        }

        const glm::dvec3 center = last_position_;
        const double r = cfg_.heightmap_radius_m;
        // The cull box must have positive extent on all three axes: buildHeightmap() applies the
        // clip per axis only where `cull_to.diagonal()[i] > 0`, so leaving z unset would silently
        // fall back to the full accumulated map extents on that axis.
        const double below = cfg_.floor_m > 0.0 ? cfg_.floor_m : 10.0;
        const double above = cfg_.ceiling_m > 0.0 ? cfg_.ceiling_m : 10.0;
        const ohm::Aabb cull(
            glm::dvec3(center.x - r, center.y - r, center.z - below),
            glm::dvec3(center.x + r, center.y + r, center.z + above));

        if (!heightmap_->buildHeightmap(center, cull))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Heightmap build failed.");
            return;
        }

        publishGrid(center, now());
    }

    void OhmMapping::publishGrid(const glm::dvec3& center, const rclcpp::Time& stamp)
    {
        const double res = cfg_.heightmap_resolution_m;
        const auto span = static_cast<int>(std::lround(2.0 * cfg_.heightmap_radius_m / res));
        if (span <= 0)
        {
            return;
        }
        const auto cells = static_cast<size_t>(span) * static_cast<size_t>(span);

        // Snap the grid origin to a multiple of the resolution so consecutive publishes land on
        // the same lattice. Without this the whole grid shifts by a fraction of a cell every
        // build as the robot moves, and a costmap consuming it sees the world shimmer.
        const double origin_x = std::floor((center.x - cfg_.heightmap_radius_m) / res) * res;
        const double origin_y = std::floor((center.y - cfg_.heightmap_radius_m) / res) * res;

        // Two passes: collect heights first, because slope needs neighbours that may not have
        // been visited yet when the voxel itself is read.
        constexpr float kNoHeight = std::numeric_limits<float>::quiet_NaN();
        std::vector<float> height(cells, kNoHeight);
        std::vector<float> clearance(cells, 0.0F);
        std::vector<uint8_t> is_virtual(cells, 0U);

        ohm::OccupancyMap& hm = heightmap_->heightmap();
        for (auto voxel_it = hm.begin(); voxel_it != hm.end(); ++voxel_it)
        {
            glm::dvec3 pos{0.0, 0.0, 0.0};
            ohm::HeightmapVoxel info{};
            const ohm::HeightmapVoxelType type = heightmap_->getHeightmapVoxelInfo(*voxel_it, &pos, &info);
            const bool surface = type == ohm::HeightmapVoxelType::kSurface;
            const bool virtual_surface = type == ohm::HeightmapVoxelType::kVirtualSurface;
            if (!surface && !virtual_surface)
            {
                continue;
            }

            const auto col = static_cast<int>(std::floor((pos.x - origin_x) / res));
            const auto row = static_cast<int>(std::floor((pos.y - origin_y) / res));
            if (col < 0 || row < 0 || col >= span || row >= span)
            {
                continue;
            }
            const auto idx = static_cast<size_t>(row) * static_cast<size_t>(span) + static_cast<size_t>(col);

            // A layered heightmap can put several surfaces in one column. Keep the one nearest the
            // robot's own height: that is the deck the robot is standing on, which is the one a 2D
            // costmap should describe. Flattening to the lowest instead would put a bridge's
            // underside into the plan.
            const auto z = static_cast<float>(pos.z);
            if (!std::isnan(height[idx]))
            {
                const double existing = std::abs(static_cast<double>(height[idx]) - center.z);
                const double candidate = std::abs(static_cast<double>(z) - center.z);
                if (candidate >= existing)
                {
                    continue;
                }
            }
            height[idx] = z;
            clearance[idx] = info.clearance;
            is_virtual[idx] = virtual_surface ? 1U : 0U;
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = stamp;
        grid.header.frame_id = cfg_.world_frame;
        grid.info.resolution = static_cast<float>(res);
        grid.info.width = static_cast<uint32_t>(span);
        grid.info.height = static_cast<uint32_t>(span);
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
        grid.info.origin.position.z = center.z;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(cells, kUnknownCell);

        const double max_slope_tan = std::tan(cfg_.max_slope_deg * M_PI / 180.0);
        size_t surface_cells = 0;
        size_t virtual_cells = 0;
        size_t virtual_drop_cells = 0;
        const auto slope_radius_cells =
            std::max(1, static_cast<int>(std::lround(cfg_.slope_radius_m / res)));
        const auto reference_radius_cells =
            std::max(1, static_cast<int>(std::lround(cfg_.virtual_reference_radius_m / res)));

        for (int row = 0; row < span; ++row)
        {
            for (int col = 0; col < span; ++col)
            {
                const auto idx =
                    static_cast<size_t>(row) * static_cast<size_t>(span) + static_cast<size_t>(col);
                if (std::isnan(height[idx]))
                {
                    continue;
                }
                ++surface_cells;

                if (is_virtual[idx] != 0U)
                {
                    ++virtual_cells;
                    // A virtual surface is INFERRED GROUND, not a hole, and conflating the two
                    // makes the output useless. Measured on rosbag2_1970_01_01-10_06_09: with
                    // inference on, 8396 of 11400 known cells are virtual, against 3017 known
                    // cells with it off. Costing all of those as near-lethal would wall off
                    // three quarters of the arena. Most of them are simply floor the laser
                    // never swept -- the MID-360's -7..+52 deg vertical FOV plus
                    // lidar.min_range_m leave a wide unscanned annulus around the robot, and
                    // free space over unknown space is exactly what that looks like.
                    //
                    // What IS worth avoiding is a virtual surface sitting well BELOW the real
                    // ground around it. That is the negative obstacle. So the cost comes from
                    // the drop, not from the mere fact of inference.
                    double reference = 0.0;
                    const bool have_reference = localRealSurfaceImpl(
                        height, is_virtual, span, col, row, reference_radius_cells, &reference);
                    const double drop =
                        have_reference ? reference - static_cast<double>(height[idx]) : 0.0;
                    if (have_reference && drop > cfg_.virtual_drop_threshold_m)
                    {
                        ++virtual_drop_cells;
                        grid.data[idx] =
                            static_cast<int8_t>(std::clamp(cfg_.virtual_surface_cost, 0, 100));
                    }
                    else
                    {
                        // Level with its surroundings, or with no measured ground nearby to
                        // compare against. Unconfirmed, so not free -- but not an obstacle.
                        grid.data[idx] =
                            static_cast<int8_t>(std::clamp(cfg_.virtual_flat_cost, 0, 100));
                    }
                    continue;
                }

                // Clearance is reported as zero when nothing is known to be overhead, which is
                // the common case outdoors and must not be read as "no headroom".
                //
                // ground_margin_m is the other half, and on this rover it is the important
                // half. navigation.yaml records the ablation for the equivalent test in
                // global_traversability: of the tests that flag open ground, clearance cost
                // 1079 cells against 231 for steepness, and "every offending cell sits
                // between ground_margin_m and 0.10 m of clearance -- not overhead structure,
                // just the map's second copy of the floor". LIO z-wander writes the same
                // ground twice 5-10 cm apart, and without a margin the upper copy reads as a
                // ceiling 5 cm above the floor. So an obstruction closer than the margin is
                // ground noise, not headroom.
                const auto clearance_m = static_cast<double>(clearance[idx]);
                if (clearance_m > cfg_.ground_margin_m && clearance_m < cfg_.min_clearance_m)
                {
                    grid.data[idx] = static_cast<int8_t>(std::clamp(cfg_.low_clearance_cost, 0, 100));
                    continue;
                }

                // Slope from a least-squares plane fit over a neighbourhood, NOT from a
                // one-cell finite difference.
                //
                // The difference is not cosmetic. A one-cell difference at 0.10 m spacing
                // calls 25 deg lethal on a 4.7 cm step between adjacent cells, which sand
                // scatter produces on its own -- and if the heightmap is finer than the
                // source voxel grid, a single voxel-boundary step reads as a ~56 deg cliff.
                // Fitting a plane over slope_radius_m averages the per-cell noise down by
                // roughly sqrt(N) and measures the slope the ROBOT experiences, over its
                // own footprint, rather than the slope between two adjacent samples.
                double slope = 0.0;
                {
                    // Normal equations for z = a*x + b*y + c over the window, with x and y
                    // taken relative to this cell so the system stays well conditioned.
                    double sxx = 0.0;
                    double sxy = 0.0;
                    double syy = 0.0;
                    double sx = 0.0;
                    double sy = 0.0;
                    double sz = 0.0;
                    double sxz = 0.0;
                    double syz = 0.0;
                    double n = 0.0;
                    for (int r = std::max(0, row - slope_radius_cells);
                         r <= std::min(span - 1, row + slope_radius_cells); ++r)
                    {
                        for (int c = std::max(0, col - slope_radius_cells);
                             c <= std::min(span - 1, col + slope_radius_cells); ++c)
                        {
                            const auto sidx = static_cast<size_t>(r) * static_cast<size_t>(span) +
                                              static_cast<size_t>(c);
                            if (std::isnan(height[sidx]))
                            {
                                continue;
                            }
                            // Virtual surfaces are excluded: they are inferred, and letting a
                            // guessed height tilt the plane would turn the boundary of the
                            // scanned region into a slope that was never measured.
                            if (is_virtual[sidx] != 0U)
                            {
                                continue;
                            }
                            const double dx = static_cast<double>(c - col) * res;
                            const double dy = static_cast<double>(r - row) * res;
                            const double dz = static_cast<double>(height[sidx]);
                            sxx += dx * dx;
                            sxy += dx * dy;
                            syy += dy * dy;
                            sx += dx;
                            sy += dy;
                            sz += dz;
                            sxz += dx * dz;
                            syz += dy * dz;
                            n += 1.0;
                        }
                    }
                    // Three points minimum to define a plane, and min_slope_fit_cells above
                    // that to stop a bare handful of noisy samples deciding "lethal". Too few
                    // is reported as flat rather than as steep: there is no gradient evidence
                    // either way, and guessing lethal would make every edge of the scanned
                    // region a wall.
                    if (n >= static_cast<double>(std::max(3, cfg_.min_slope_fit_cells)))
                    {
                        // Solve the 3x3 symmetric system by elimination of c.
                        const double a11 = sxx - sx * sx / n;
                        const double a12 = sxy - sx * sy / n;
                        const double a22 = syy - sy * sy / n;
                        const double b1 = sxz - sx * sz / n;
                        const double b2 = syz - sy * sz / n;
                        const double det = a11 * a22 - a12 * a12;
                        // Degenerate when the valid cells are collinear (a one-cell-wide
                        // sliver at the edge of the scanned region), which is exactly where a
                        // fitted slope would be meaningless.
                        if (std::abs(det) > 1e-12)
                        {
                            const double ga = (b1 * a22 - b2 * a12) / det;
                            const double gb = (a11 * b2 - a12 * b1) / det;
                            slope = std::sqrt(ga * ga + gb * gb);
                        }
                    }
                }

                if (max_slope_tan > 0.0 && slope >= max_slope_tan)
                {
                    grid.data[idx] = kLethalCell;
                }
                else
                {
                    const double ratio = max_slope_tan > 0.0 ? slope / max_slope_tan : 0.0;
                    grid.data[idx] = static_cast<int8_t>(std::clamp(ratio * 99.0, 0.0, 99.0));
                }
            }
        }

        grid_pub_->publish(grid);

        if (cloud_pub_)
        {
            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header = grid.header;
            sensor_msgs::PointCloud2Modifier mod(cloud);
            mod.setPointCloud2Fields(
                4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            mod.resize(surface_cells);
            sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> oy(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> oz(cloud, "z");
            sensor_msgs::PointCloud2Iterator<float> oi(cloud, "intensity");
            size_t written = 0;
            for (int row = 0; row < span && written < surface_cells; ++row)
            {
                for (int col = 0; col < span && written < surface_cells; ++col)
                {
                    const auto idx =
                        static_cast<size_t>(row) * static_cast<size_t>(span) + static_cast<size_t>(col);
                    if (std::isnan(height[idx]))
                    {
                        continue;
                    }
                    *ox = static_cast<float>(origin_x + (static_cast<double>(col) + 0.5) * res);
                    *oy = static_cast<float>(origin_y + (static_cast<double>(row) + 0.5) * res);
                    *oz = height[idx];
                    // Cost, so RViz's intensity colouring shows the same thing the costmap sees.
                    *oi = static_cast<float>(grid.data[idx]);
                    ++ox;
                    ++oy;
                    ++oz;
                    ++oi;
                    ++written;
                }
            }
            mod.resize(written);
            cloud_pub_->publish(cloud);
        }

        RCLCPP_DEBUG(
            get_logger(),
            "heightmap: %zu surface cells (%zu virtual, %zu of them drops) from %zu regions, "
            "%zu scans",
            surface_cells, virtual_cells, virtual_drop_cells, map_->regionCount(),
            scans_integrated_);
    }

}  // namespace ohm_mapping
