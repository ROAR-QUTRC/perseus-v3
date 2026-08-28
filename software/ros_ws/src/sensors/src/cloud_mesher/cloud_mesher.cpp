/**
 * Turns a point cloud map into a triangle mesh for RViz.
 *
 * Meant for the base station, at the far end of the Draco link: the rover sends
 * the compressed cloud it already sends, and the surface is reconstructed here.
 * That split matters because a mesh is far larger on the wire than the cloud it
 * comes from -- a TRIANGLE_LIST marker carries three float64 vertices per
 * triangle with no index buffer, around ten times the bytes -- so building it
 * on the rover would put all of that over the radio for no gain.
 *
 * Greedy projection triangulation, not Poisson or marching cubes. The input is
 * a LiDAR surface: a thin shell of points with a hole wherever nothing was
 * scanned, and no interior. Greedy projection connects neighbouring points into
 * triangles and leaves the holes alone, which is what you want to look at.
 * Poisson would fit a watertight surface and invent ground across every gap,
 * which reads as terrain that was never observed.
 *
 * The cost is not trivial and grows with the cloud, so nothing happens on a
 * timer: each incoming cloud is meshed once and published. Point it at a map
 * topic that updates every few seconds, not at a per-scan topic.
 */

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
// The impl header, not just the declaration, and deliberately: including it
// instantiates GreedyProjectionTriangulation here instead of pulling it from
// libpcl_surface. That library links VTK, and VTK brings a whole GUI stack --
// libGL, Qt's harfbuzz/icu/pcre2, libnuma -- that this node never touches and
// would otherwise have to be present at load time. The class is a template, so
// there is nothing in the library it actually needs.
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/gp3.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace {
/// Colour a vertex by height, so the shape reads without a light source. RViz
/// shades a TRIANGLE_LIST flat when every vertex is one colour, which makes
/// terrain hard to see.
std_msgs::msg::ColorRGBA heightColour(double z, double z_min, double z_span) {
  const double t =
      z_span > 1e-6 ? std::clamp((z - z_min) / z_span, 0.0, 1.0) : 0.5;
  std_msgs::msg::ColorRGBA c;
  // A plain blue-through-green-to-red ramp: cheap, and the eye reads height
  // from it immediately without needing a legend.
  c.r = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * t - 3.0), 0.0, 1.0));
  c.g = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * t - 2.0), 0.0, 1.0));
  c.b = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * t - 1.0), 0.0, 1.0));
  c.a = 1.0f;
  return c;
}
} // namespace

class CloudMesher : public rclcpp::Node {
public:
  CloudMesher() : Node("cloud_mesher") {
    cloud_topic_ = declare_parameter<std::string>(
        "cloud_topic", "/Laser_map/downsampled/decompressed");
    mesh_topic_ = declare_parameter<std::string>("mesh_topic", "mesh");
    // Empty means "whatever frame the cloud arrived in", which is almost always
    // right.
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    // Points closer than this are merged first. The dominant cost knob:
    // triangulation is superlinear in point count, and a LiDAR map is far
    // denser than a mesh needs.
    leaf_size_ = declare_parameter<double>("leaf_size", 0.15);
    // Neighbours used to estimate each normal. Too few and the normals are
    // noisy, too many and thin structure gets smoothed into the ground.
    normal_k_ = declare_parameter<int>("normal_k", 20);
    // The furthest apart two points may be and still be joined. This is what
    // decides whether a gap is a hole or gets bridged; scale it with leaf_size.
    search_radius_ = declare_parameter<double>("search_radius", 0.6);
    // Triangles are rejected when a vertex is much further away than its
    // nearest neighbour, which stops long slivers reaching across gaps.
    mu_ = declare_parameter<double>("mu", 2.5);
    max_nearest_neighbours_ =
        declare_parameter<int>("max_nearest_neighbours", 100);
    // Surface angle beyond which points are not connected, so a wall does not
    // get stitched to the floor behind it.
    max_surface_angle_deg_ =
        declare_parameter<double>("max_surface_angle_deg", 45.0);
    min_angle_deg_ = declare_parameter<double>("min_angle_deg", 10.0);
    max_angle_deg_ = declare_parameter<double>("max_angle_deg", 120.0);
    // false lets triangles form regardless of which way the normals point.
    // LiDAR normals flip freely on a surface seen from one side, and
    // consistency is not worth the holes.
    normal_consistency_ = declare_parameter<bool>("normal_consistency", false);
    colour_by_height_ = declare_parameter<bool>("colour_by_height", true);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CloudMesher::onCloud, this, std::placeholders::_1));
    // Latched: the mesh is large and slow to change, and a viewer that connects
    // between updates should not have to wait for the next one.
    pub_ = create_publisher<visualization_msgs::msg::Marker>(
        mesh_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    RCLCPP_INFO(get_logger(), "Meshing '%s' into '%s'.", cloud_topic_.c_str(),
                mesh_topic_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto started = std::chrono::steady_clock::now();

    // Converted through PointXYZ deliberately: a map cloud is geometry, and
    // asking PCL for a type with an intensity field it does not have logs a
    // warning per message and leaves that channel uninitialised.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    const size_t raw_points = cloud->points.size();
    if (raw_points < 3) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr thinned = thin(cloud);
    if (thinned->points.size() < 3) {
      return;
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(thinned);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(thinned);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setKSearch(normal_k_);
    normal_estimation.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*thinned, *normals, *with_normals);
    auto normal_tree =
        std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    normal_tree->setInputCloud(with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    gp3.setSearchRadius(search_radius_);
    gp3.setMu(mu_);
    gp3.setMaximumNearestNeighbors(max_nearest_neighbours_);
    gp3.setMaximumSurfaceAngle(max_surface_angle_deg_ * M_PI / 180.0);
    gp3.setMinimumAngle(min_angle_deg_ * M_PI / 180.0);
    gp3.setMaximumAngle(max_angle_deg_ * M_PI / 180.0);
    gp3.setNormalConsistency(normal_consistency_);
    gp3.setInputCloud(with_normals);
    gp3.setSearchMethod(normal_tree);
    gp3.reconstruct(mesh);

    publish(mesh, thinned, msg->header);

    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    RCLCPP_INFO(get_logger(),
                "%zu points -> %zu after thinning -> %zu triangles in %.2f s",
                raw_points, thinned->points.size(), mesh.polygons.size(),
                elapsed);
  }

  /// Merge points closer together than leaf_size, by keeping one per grid cell.
  /// Done by hand rather than with pcl::VoxelGrid because the grid's centroid
  /// averaging drags points off the surface at edges, and one representative
  /// point is all this needs.
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  thin(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in) const {
    if (leaf_size_ <= 0.0) {
      return in;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    out->points.reserve(in->points.size());
    const double inv = 1.0 / leaf_size_;
    std::unordered_set<int64_t> seen;
    seen.reserve(in->points.size() * 2);
    for (const auto &p : in->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      // Three 21-bit cell indices packed into one integer: a map is nowhere
      // near 2^21 cells across in any axis, and one key hashes faster than a
      // tuple.
      const int64_t gx = static_cast<int64_t>(std::floor(p.x * inv)) & 0x1FFFFF;
      const int64_t gy = static_cast<int64_t>(std::floor(p.y * inv)) & 0x1FFFFF;
      const int64_t gz = static_cast<int64_t>(std::floor(p.z * inv)) & 0x1FFFFF;
      if (seen.insert((gx << 42) | (gy << 21) | gz).second) {
        out->points.push_back(p);
      }
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return out;
  }

  void publish(const pcl::PolygonMesh &mesh,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
               const std_msgs::msg::Header &header) {
    if (mesh.polygons.empty()) {
      RCLCPP_WARN(get_logger(),
                  "No triangles produced. search_radius (%.2f) is the usual "
                  "cause: it has to "
                  "exceed the spacing left by leaf_size (%.2f).",
                  search_radius_, leaf_size_);
      return;
    }

    // gp3 re-orders and re-indexes the cloud it was given, so vertices come
    // from the mesh's own cloud rather than the one passed in.
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);

    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();
    if (colour_by_height_) {
      for (const auto &v : vertices.points) {
        z_min = std::min(z_min, static_cast<double>(v.z));
        z_max = std::max(z_max, static_cast<double>(v.z));
      }
    }
    const double z_span = z_max - z_min;

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    if (!frame_id_.empty()) {
      marker.header.frame_id = frame_id_;
    }
    marker.ns = "cloud_mesher";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.r = marker.color.g = marker.color.b = 0.8f;
    marker.color.a = 1.0f;
    marker.points.reserve(mesh.polygons.size() * 3);
    if (colour_by_height_) {
      marker.colors.reserve(mesh.polygons.size() * 3);
    }

    const size_t n_vertices = vertices.points.size();
    for (const auto &polygon : mesh.polygons) {
      if (polygon.vertices.size() != 3) {
        continue;
      }
      bool in_range = true;
      for (const auto idx : polygon.vertices) {
        in_range = in_range && static_cast<size_t>(idx) < n_vertices;
      }
      if (!in_range) {
        continue;
      }
      for (const auto idx : polygon.vertices) {
        const auto &v = vertices.points[idx];
        geometry_msgs::msg::Point p;
        p.x = v.x;
        p.y = v.y;
        p.z = v.z;
        marker.points.push_back(p);
        if (colour_by_height_) {
          marker.colors.push_back(heightColour(v.z, z_min, z_span));
        }
      }
    }
    if (marker.points.empty()) {
      return;
    }
    pub_->publish(marker);
    (void)points;
  }

  std::string cloud_topic_, mesh_topic_, frame_id_;
  double leaf_size_ = 0.15;
  int normal_k_ = 20;
  double search_radius_ = 0.6;
  double mu_ = 2.5;
  int max_nearest_neighbours_ = 100;
  double max_surface_angle_deg_ = 45.0;
  double min_angle_deg_ = 10.0;
  double max_angle_deg_ = 120.0;
  bool normal_consistency_ = false;
  bool colour_by_height_ = true;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMesher>());
  rclcpp::shutdown();
  return 0;
}
