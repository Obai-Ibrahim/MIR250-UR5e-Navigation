/**
 * arm_footprint_projector.cpp
 *
 * Subscribes to /joint_states (used as a trigger), looks up every UR5e
 * arm-link TF frame relative to mir/base_footprint, projects each link
 * capsule onto the XY plane, and publishes the convex hull of the combined
 * MiR base polygon + arm cross-sections.
 *
 * Published topic : /merged/arm_footprint  (geometry_msgs/PolygonStamped)
 * Reference frame : mir/base_footprint
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 2-D convex hull — Andrew's Monotone Chain
// ---------------------------------------------------------------------------
namespace geom {

struct Pt { double x, y; };

static double cross(const Pt & O, const Pt & A, const Pt & B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

static std::vector<Pt> convex_hull(std::vector<Pt> pts)
{
  const std::size_t n = pts.size();
  if (n < 3) return pts;

  std::sort(pts.begin(), pts.end(),
    [](const Pt & a, const Pt & b){ return a.x < b.x || (a.x == b.x && a.y < b.y); });

  std::vector<Pt> hull;
  hull.reserve(2 * n);

  // lower hull
  for (std::size_t i = 0; i < n; ++i) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull[hull.size()-1], pts[i]) <= 0.0)
      hull.pop_back();
    hull.push_back(pts[i]);
  }

  // upper hull
  const std::size_t lower_size = hull.size();
  for (std::size_t i = n - 1; ; --i) {
    while (hull.size() > lower_size && cross(hull[hull.size()-2], hull[hull.size()-1], pts[i]) <= 0.0)
      hull.pop_back();
    hull.push_back(pts[i]);
    if (i == 0) break;
  }

  hull.pop_back();   // remove the closing duplicate of the first point
  return hull;
}

} // namespace geom

// ---------------------------------------------------------------------------

class ArmFootprintProjector : public rclcpp::Node
{
public:
  ArmFootprintProjector()
  : Node("arm_footprint_projector"),
    tf_buf_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
    tf_listener_(tf_buf_)
  {
    pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/merged/arm_footprint", 10);
    global_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/global_costmap/footprint", 10);
    local_pub_  = create_publisher<geometry_msgs::msg::Polygon>(
      "/local_costmap/footprint", 10);

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg){ on_joint_states(msg); });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this](){ publish_footprint(); });
  }

private:
  // -------------------------------------------------------------------------
  // Static MiR 250 base footprint corners  (mir_nav_params.yaml)
  // [ x_m, y_m ]  in  mir/base_footprint frame.
  // Edit these constants when the base footprint is updated.
  // -------------------------------------------------------------------------
  static constexpr std::array<std::array<double,2>, 4> MIR_FOOTPRINT {{
    {{ 0.50,  0.25}},
    {{-0.40,  0.25}},
    {{-0.40, -0.25}},
    {{ 0.50, -0.25}},
  }};

  // -------------------------------------------------------------------------
  // UR5e arm segments: { start_frame, end_frame, radius_m, n_samples }
  // Radii from ur_description/config/ur5e/physical_parameters.yaml.
  // n_samples controls how finely the long links are discretised along
  // their axis before the per-sample radius circle is added.
  // -------------------------------------------------------------------------
  struct Segment {
    const char * start_frame;
    const char * end_frame;
    double       radius;
    int          n_samples;
  };

  static constexpr std::array<Segment, 7> UR_SEGMENTS {{
    {"ur_base_link",      "ur_shoulder_link",   0.060, 3},
    {"ur_shoulder_link",  "ur_upper_arm_link",  0.060, 4},
    {"ur_upper_arm_link", "ur_forearm_link",    0.054, 6},  // 0.425 m long
    {"ur_forearm_link",   "ur_wrist_1_link",    0.040, 5},  // 0.392 m long
    {"ur_wrist_1_link",   "ur_wrist_2_link",    0.045, 3},
    {"ur_wrist_2_link",   "ur_wrist_3_link",    0.045, 3},
    {"ur_wrist_3_link",   "ur_flange",          0.040, 2},
  }};

  static constexpr int    CIRCLE_STEPS    = 16;
  static constexpr double TWO_PI          = 2.0 * M_PI;
  static constexpr const char * REF_FRAME = "mir/base_footprint";

  // -------------------------------------------------------------------------

  bool lookup_xy(const std::string & frame, double & x, double & y)
  {
    try {
      auto tf = tf_buf_.lookupTransform(REF_FRAME, frame, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void add_disc(std::vector<geom::Pt> & cloud,
                double cx, double cy, double radius) const
  {
    for (int i = 0; i < CIRCLE_STEPS; ++i) {
      const double a = TWO_PI * i / CIRCLE_STEPS;
      cloud.push_back({cx + radius * std::cos(a),
                       cy + radius * std::sin(a)});
    }
  }

  // -------------------------------------------------------------------------

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr &)
  {
    std::vector<geom::Pt> cloud;
    cloud.reserve(4 + UR_SEGMENTS.size() * CIRCLE_STEPS * 6);

    for (const auto & c : MIR_FOOTPRINT)
      cloud.push_back({c[0], c[1]});

    for (const auto & seg : UR_SEGMENTS) {
      double x0, y0, x1, y1;
      if (!lookup_xy(seg.start_frame, x0, y0) ||
          !lookup_xy(seg.end_frame,   x1, y1))
        return;   // TF not yet available — skip this cycle

      for (int k = 0; k < seg.n_samples; ++k) {
        const double t  = (seg.n_samples > 1)
                          ? static_cast<double>(k) / (seg.n_samples - 1)
                          : 0.0;
        const double cx = x0 + t * (x1 - x0);
        const double cy = y0 + t * (y1 - y0);
        add_disc(cloud, cx, cy, seg.radius);
      }
    }

    last_hull_ = geom::convex_hull(cloud);
  }

  void publish_footprint()
  {
    if (last_hull_.empty()) return;

    geometry_msgs::msg::Polygon msg;
    for (const auto & p : last_hull_) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(p.x);
      pt.y = static_cast<float>(p.y);
      pt.z = 0.0f;
      msg.points.push_back(pt);
    }

    pub_->publish(msg);
    global_pub_->publish(msg);
    local_pub_->publish(msg);
  }

  // -------------------------------------------------------------------------

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    sub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;
  tf2_ros::Buffer            tf_buf_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<geom::Pt>      last_hull_;
};

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmFootprintProjector>());
  rclcpp::shutdown();
  return 0;
}
