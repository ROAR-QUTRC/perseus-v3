// Copyright 2026 ROAR QUTRC
//
// A ROS 2 front end for CSIRO's OHM, which has none of its own: there is no rclcpp
// anywhere in the OHM tree, and `ohmapp` is an offline batch harness that reads a
// globally-optimised cloud plus a trajectory file and writes a .ohm. The library
// underneath streams perfectly well; this node is the streaming wrapper.
//
// What it produces that the existing stack does not: OHM's heightmap records, per 2D
// column, the supporting surface height AND the clearance above it, AND -- the reason to
// bother -- a "virtual surface" inferred at the boundary between free and unknown space.
// That last one is the negative-obstacle case. global_traversability, which grids
// /Laser_map, cannot represent a hole at all: a pit returns no points, so its cells simply
// fall below min_points_per_cell and publish as unknown, indistinguishable from ground the
// sensor has not swept yet. OHM carves free space along every ray, so the floor of a pit
// that was never hit still reads as "free space bounded below by unknown", which is
// exactly a virtual surface.
//
// This runs alongside global_traversability rather than replacing it. Both publish an
// OccupancyGrid; which one nav2 consumes is a launch-time choice.
#ifndef OHM_MAPPING__OHM_MAPPING_HPP_
#define OHM_MAPPING__OHM_MAPPING_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <glm/vec3.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace ohm
{
    class OccupancyMap;
    class RayMapperOccupancy;
    class Heightmap;
}  // namespace ohm

namespace ohm_mapping
{

    /// Everything the node reads from the parameter server, resolved once at construction.
    ///
    /// Grouped rather than left as loose members so the (many) values that only matter to one
    /// stage stay visibly attached to that stage.
    struct Config
    {
        // --- Input plumbing -------------------------------------------------------------
        /// Registered, motion-compensated cloud, already in `world_frame`. BIEVR-LIO publishes
        /// exactly this on /bievr_lio/points/registered, unconditionally, as
        /// `T_W_I * undistorted` (Pipeline::publishFrame). Feeding OHM the raw /livox/lidar
        /// instead would mean deskewing it here for no reason.
        std::string cloud_topic;
        /// Pose of `robot_frame` in `world_frame`, used only for the ray origins and as the
        /// heightmap's seed position. Taken from the LIO rather than from TF so that it is the
        /// *same* pose the cloud above was registered with -- TF's odom -> base_link comes from
        /// the EKF, which differs from the LIO's own estimate by a few centimetres.
        std::string odom_topic;
        std::string world_frame;
        std::string robot_frame;
        /// Source frame of the LiDAR, looked up against `robot_frame` once to place the ray
        /// origins at the sensor rather than at the robot's origin. A mast-mounted MID-360 is
        /// ~0.65 m up, which is the difference between carving the free space above the floor
        /// and carving a cone through the floor itself.
        std::string lidar_frame;
        /// How far apart a cloud's stamp and an odometry stamp may be and still be treated as
        /// the same scan. BIEVR publishes both from one `header`, so in practice this matches
        /// exactly and the tolerance only covers a backend that does not.
        double odom_match_tolerance_s;
        /// How long a cloud may sit in the pending buffer waiting for its odometry before it
        /// is given up on. Only reached when odometry genuinely stops.
        double pending_timeout_s;

        // --- Occupancy map --------------------------------------------------------------
        double resolution_m;
        double hit_probability;
        double miss_probability;
        /// Rays longer than this are clipped, not dropped: the sample is discarded but the free
        /// space along the truncated ray is still carved. Beyond the MID-360's honest range a
        /// "hit" is mostly noise, while the emptiness in front of it is still real.
        double ray_max_range_m;
        /// Take every Nth point. The first dial to reach for if integration cannot keep up;
        /// 1 keeps everything.
        int ray_stride;
        /// Regions whose centre is further than this from the robot are dropped from the map
        /// (OccupancyMap::removeDistanceRegions). Without it the map grows for the whole run
        /// and every heightmap build gets slower with it.
        double map_retain_radius_m;

        // --- Heightmap ------------------------------------------------------------------
        double heightmap_resolution_m;
        /// Half-extent of the box the heightmap is built over, centred on the robot. This is
        /// the single most important number for CPU: Heightmap::buildHeightmap() clears and
        /// rebuilds from scratch every call (Heightmap.cpp:534), it is unconditionally serial
        /// (Heightmap.h:146 declares setThreadCount() and nothing defines it), and its cost
        /// scales with AREA. Doubling this quadruples the per-build cost.
        double heightmap_radius_m;
        double heightmap_rate_hz;
        /// Passed to Heightmap's constructor as min_clearance: a surface with less headroom
        /// than this is not a surface the robot can occupy.
        double min_clearance_m;
        /// Vertical search limits relative to the seed position, in metres. Positive enables.
        double ceiling_m;
        double floor_m;
        /// "planar", "simple_fill", "layered_fill" or "layered_fill_unordered". Planar is
        /// cheapest and visits each column once; the layered modes keep more than one surface
        /// per column so a ramp or an overhang stays traversable instead of collapsing into an
        /// obstacle.
        std::string mode;
        /// Infer ground at the free/unknown boundary. The negative-obstacle feature.
        bool generate_virtual_surface;
        /// Suppress a virtual surface unless it has at least this many occupied neighbours.
        /// 0 disables the filter and lets isolated speculative cells through.
        int virtual_surface_filter_threshold;

        // --- Cost mapping ---------------------------------------------------------------
        /// Slope at or above this is lethal. Derived from neighbouring cell heights rather
        /// than from HeightmapVoxel's normal, which is only populated when the source map
        /// carries CovarianceVoxel.
        double max_slope_deg;
        /// Cost for a virtual surface that sits more than `virtual_drop_threshold_m` below
        /// the real ground around it -- i.e. an actual inferred hole. Deliberately high but
        /// not 100: an unconfirmed hole should repel the planner without being an immovable
        /// obstacle, and keeping it distinct from lethal makes it visible in the output.
        int virtual_surface_cost;
        /// Cost for a virtual surface that is level with the ground around it: unswept floor
        /// rather than a hole. This is the overwhelming majority of them -- 8396 of 11400
        /// known cells were virtual on rosbag2_1970_01_01-10_06_09 -- so costing these as
        /// near-lethal walls off most of the map for no measured reason.
        int virtual_flat_cost;
        /// How far below the surrounding real surface a virtual surface must sit before it is
        /// treated as a hole rather than as unswept floor.
        double virtual_drop_threshold_m;
        /// Radius over which the surrounding real surface height is measured, for that
        /// comparison.
        double virtual_reference_radius_m;
        /// Cost given to a real surface whose clearance is below min_clearance_m.
        int low_clearance_cost;
        bool publish_cloud;
    };

    /// Streams registered clouds into an OHM occupancy map and publishes its heightmap.
    class OhmMapping : public rclcpp::Node
    {
    public:
        explicit OhmMapping(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~OhmMapping() override;

    private:
        void declareParameters();
        void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void buildAndPublish();

        /// Integrate every buffered cloud whose odometry has now arrived, oldest first.
        ///
        /// Called from BOTH callbacks, and that is the whole point. BIEVR-LIO publishes the
        /// registered cloud and the odometry for a scan from one `header`, so their stamps
        /// are bit-identical -- but the cloud goes out first (measured on
        /// rosbag2_1970_01_01-10_06_09: the cloud precedes its odometry by ~0.2 ms, and in
        /// 3637 of 4345 scans the very next message in the bag is that odometry). Matching
        /// on arrival therefore finds only the PREVIOUS scan's pose, 100 ms stale, and
        /// discards the scan. Buffering instead of matching eagerly is what takes this from
        /// integrating a sixth of the scans to integrating all of them.
        void drainPending();
        /// Integrate one cloud, with ray origins at `origin`.
        void integrateCloud(const sensor_msgs::msg::PointCloud2& msg, const glm::dvec3& origin);

        /// Nearest cached odometry position to `stamp`, or nullopt if none is close enough.
        /// Returns the robot origin in `world_frame`; the sensor offset is applied separately.
        bool robotPositionAt(const rclcpp::Time& stamp, glm::dvec3* position) const;
        /// base_link -> lidar translation, looked up lazily and then cached. Returns false
        /// until TF can answer, at which point rays are still integrated -- from the robot
        /// origin rather than the sensor -- and a throttled warning is issued.
        bool lidarOffset(glm::dvec3* offset);

        void publishGrid(const glm::dvec3& centre, const rclcpp::Time& stamp);

        Config cfg_;

        std::unique_ptr<ohm::OccupancyMap> map_;
        std::unique_ptr<ohm::RayMapperOccupancy> mapper_;
        std::unique_ptr<ohm::Heightmap> heightmap_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        glm::dvec3 lidar_offset_{0.0, 0.0, 0.0};
        bool lidar_offset_valid_{false};

        /// Recent robot positions, oldest first, trimmed to `kOdomHistory`.
        struct StampedPosition
        {
            double stamp_s;
            glm::dvec3 position;
        };
        std::deque<StampedPosition> odom_history_;
        static constexpr size_t kOdomHistory = 200;

        /// Clouds waiting for the odometry that shares their stamp. Bounded: if odometry
        /// stops entirely there is nothing to match against and holding scans forever would
        /// just be a leak, so the oldest are dropped with a throttled warning.
        std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
        static constexpr size_t kMaxPendingClouds = 20;

        /// Scratch buffer for the origin/sample pairs handed to integrateRays(), kept as a
        /// member so a 20k-point scan at 10 Hz does not reallocate every callback.
        std::vector<glm::dvec3> rays_;

        bool have_pose_{false};
        glm::dvec3 last_position_{0.0, 0.0, 0.0};
        size_t scans_integrated_{0};
    };

}  // namespace ohm_mapping

#endif  // OHM_MAPPING__OHM_MAPPING_HPP_
