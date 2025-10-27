#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <deque>

class TemporalPointCloudFilter : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  TemporalPointCloudFilter() : rclcpp::Node("temporal_pc_filter") {
    // Topics
    input_topic_  = this->declare_parameter<std::string>("input_topic",  "/slam/pointcloud");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/slam/pointcloud_filtered");

    // Core scaling + band-pass (post-scale distances)
    scale_factor_ = this->declare_parameter<double>("scale_factor", 1.0);      // shrink map ~5.5x by default
    range_min_    = this->declare_parameter<double>("range_min",    0.1);       // keep 1–3 m band (post-scale)
    range_max_    = this->declare_parameter<double>("range_max",    10.0);

    // Downsampling + denoising
    leaf_size_    = this->declare_parameter<double>("leaf_size",     0.00);     // stricter than 0.05
    mean_k_       = this->declare_parameter<int>("sor_mean_k",       20);       // neighbor count for SOR
    stddev_mul_   = this->declare_parameter<double>("sor_stddev",    2.05);     // lower = stricter
    radius_       = this->declare_parameter<double>("ror_radius",    0.06);     // neighborhood radius
    min_neighbors_= this->declare_parameter<int>("ror_min_neighbors",2);

    // Temporal windowing
    use_window_   = this->declare_parameter<bool>("use_window",      true);
    window_size_  = this->declare_parameter<int>("window_size",      2);        // ~last 5 frames

    // Misc
    max_points_   = this->declare_parameter<int>("max_points",       120000);   // clamp to keep viz snappy

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TemporalPointCloudFilter::callback, this, std::placeholders::_1)
    );
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "temporal_pc_filter: input='%s' -> output='%s'  scale=%.3f  band=[%.2f, %.2f]  leaf=%.3f  SOR(k=%d,σ=%.2f)  ROR(r=%.2f,k=%d)  window=%d",
      input_topic_.c_str(), output_topic_.c_str(), scale_factor_, range_min_, range_max_,
      leaf_size_, mean_k_, stddev_mul_, radius_, min_neighbors_, window_size_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    // 1) Scale down (simulate mono scale) BEFORE band-pass
    if (scale_factor_ > 0.0 && scale_factor_ != 1.0) {
      for (auto &p : cloud->points) {
        p.x *= scale_factor_;
        p.y *= scale_factor_;
        p.z *= scale_factor_;
      }
    }

    // 2) Distance band-pass in post-scale space (keep 1–3 m "happy middle")
    CloudT::Ptr band(new CloudT);
    band->points.reserve(cloud->points.size());
    const double r2_min = range_min_ * range_min_;
    const double r2_max = range_max_ * range_max_;
    for (const auto &p : cloud->points) {
      const double r2 = p.x*p.x + p.y*p.y + p.z*p.z;
      if (r2 >= r2_min && r2 <= r2_max && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        band->points.push_back(p);
      }
    }
    band->width = static_cast<uint32_t>(band->points.size());
    band->height = 1;
    band->is_dense = false;

    if (band->empty()) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*band, out);
      out.header = msg->header;
      pub_->publish(out);
      return;
    }

    // 3) Temporal window (accumulate a small history to favour persistent structure)
    if (use_window_ && window_size_ > 1) {
      window_.push_back(band);
      if ((int)window_.size() > window_size_) window_.pop_front();

      CloudT::Ptr merged(new CloudT);
      for (const auto &c : window_) *merged += *c;
      band.swap(merged);
    }

    // 4) VoxelGrid downsample (structure-preserving)
    CloudT::Ptr ds(new CloudT);
    if (leaf_size_ > 1e-6) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(band);
      vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      vg.filter(*ds);
    } else {
      ds = band;
    }

    if (ds->empty()) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*ds, out);
      out.header = msg->header;
      pub_->publish(out);
      return;
    }

    // 5) Statistical Outlier Removal (tight)
    CloudT::Ptr sor(new CloudT);
    {
      pcl::StatisticalOutlierRemoval<PointT> f;
      f.setInputCloud(ds);
      f.setMeanK(mean_k_);
      f.setStddevMulThresh(stddev_mul_);
      f.filter(*sor);
    }

    // 6) Radius Outlier Removal (ensure local support)
    CloudT::Ptr ror(new CloudT);
    {
      pcl::RadiusOutlierRemoval<PointT> f;
      f.setInputCloud(sor);
      f.setRadiusSearch(radius_);
      f.setMinNeighborsInRadius(min_neighbors_);
      f.filter(*ror);
    }

    // 7) Final clamp on total points
    if (max_points_ > 0 && (int)ror->size() > max_points_) {
      CloudT::Ptr limited(new CloudT);
      limited->reserve(max_points_);
      double step = static_cast<double>(ror->size()) / static_cast<double>(max_points_);
      double idx = 0.0;
      for (int i = 0; i < max_points_; ++i) {
        limited->push_back((*ror)[static_cast<size_t>(idx)]);
        idx += step;
      }
      limited->width = static_cast<uint32_t>(limited->size());
      limited->height = 1;
      limited->is_dense = false;
      ror.swap(limited);
    }

    // 8) Publish
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*ror, out);
    out.header = msg->header;  // preserve timing/frame_id
    pub_->publish(out);
  }

  // Params
  std::string input_topic_, output_topic_;
  double scale_factor_, range_min_, range_max_;
  double leaf_size_, stddev_mul_, radius_;
  int mean_k_, min_neighbors_, window_size_;
  bool use_window_;
  int max_points_;

  // State
  std::deque<CloudT::Ptr> window_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemporalPointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
