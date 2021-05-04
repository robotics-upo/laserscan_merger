#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <laser_geometry/laser_geometry.h>
#include <laserscan_merger/laserscan_mergerConfig.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace pcl;
using namespace laserscan_merger;

class LaserscanMerger {
public:
  LaserscanMerger(ros::NodeHandle &n, tf2_ros::Buffer *tf);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan,
                    std::string topic);
  void pointcloud_to_laserscan(Eigen::MatrixXf points,
                               pcl::PCLPointCloud2 *merged_cloud);
  void reconfigureCallback(laserscan_mergerConfig &config, uint32_t level);

private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector_;
  tf2_ros::Buffer *tf_buffer_;
  // tf::TransformListener tf_listener_;

  ros::Publisher point_cloud_publisher_;
  ros::Publisher laser_scan_publisher_;
  vector<ros::Subscriber> scan_subscribers_;
  vector<bool> clouds_modified_;

  vector<pcl::PCLPointCloud2> clouds_;
  vector<string> input_topics_;

  void laserscan_topic_parser();

  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double time_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;

  string destination_frame_;
  string cloud_destination_topic_;
  string scan_destination_topic_;
  string laserscan_topics_;
};

void LaserscanMerger::reconfigureCallback(laserscan_mergerConfig &config,
                                          uint32_t level) {
  this->angle_min_ = config.angle_min;
  this->angle_max_ = config.angle_max;
  this->angle_increment_ = config.angle_increment;
  this->time_increment_ = config.time_increment;
  this->scan_time_ = config.scan_time;
  this->range_min_ = config.range_min;
  this->range_max_ = config.range_max;
}

LaserscanMerger::LaserscanMerger(ros::NodeHandle &n, tf2_ros::Buffer *tf) {
  ros::NodeHandle nh("~");
  node_ = n;
  tf_buffer_ = tf;
  // tf2_ros::TransformListener tf_listener(tf_buffer_);

  nh.param<std::string>("destination_frame", destination_frame_, "cart_frame");
  nh.param<std::string>("cloud_destination_topic", cloud_destination_topic_,
                        "/merged_cloud");
  nh.param<std::string>("scan_destination_topic", scan_destination_topic_,
                        "/scan_multi");
  nh.param<std::string>("laserscan_topics", laserscan_topics_, "");
  nh.param("angle_min", angle_min_, -2.36);
  nh.param("angle_max", angle_max_, 2.36);
  nh.param("angle_increment", angle_increment_, 0.0058);
  nh.param("scan_time", scan_time_, 0.0333333);
  nh.param("range_min", range_min_, 0.45);
  nh.param("range_max", range_max_, 25.0);

  this->laserscan_topic_parser();

  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
      cloud_destination_topic_.c_str(), 1, false);
  laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(
      scan_destination_topic_.c_str(), 1, false);
}

void LaserscanMerger::laserscan_topic_parser() {
  // LaserScan topics to subscribe
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);

  istringstream iss(laserscan_topics_);
  vector<string> tokens;
  copy(istream_iterator<string>(iss), istream_iterator<string>(),
       back_inserter<vector<string>>(tokens));
  vector<string> tmp_input_topics;
  for (int i = 0; i < tokens.size(); ++i) {
    for (int j = 0; j < topics.size(); ++j) {
      if ((tokens[i].compare(topics[j].name) == 0) &&
          (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0)) {
        tmp_input_topics.push_back(topics[j].name);
      }
    }
  }

  sort(tmp_input_topics.begin(), tmp_input_topics.end());
  std::vector<string>::iterator last =
      std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
  tmp_input_topics.erase(last, tmp_input_topics.end());

  // Do not re-subscribe if the topics are the same
  if ((tmp_input_topics.size() != input_topics_.size()) ||
      !equal(tmp_input_topics.begin(), tmp_input_topics.end(),
             input_topics_.begin())) {

    // Unsubscribe from previous topics
    for (int i = 0; i < scan_subscribers_.size(); ++i)
      scan_subscribers_[i].shutdown();

    input_topics_ = tmp_input_topics;
    if (input_topics_.size() > 0) {
      scan_subscribers_.resize(input_topics_.size());
      clouds_modified_.resize(input_topics_.size());
      clouds_.resize(input_topics_.size());
      ROS_INFO("Subscribing to topics\t%ld", scan_subscribers_.size());
      for (int i = 0; i < input_topics_.size(); ++i) {
        scan_subscribers_[i] = node_.subscribe<sensor_msgs::LaserScan>(
            input_topics_[i].c_str(), 1,
            boost::bind(&LaserscanMerger::scanCallback, this, _1,
                        input_topics_[i]));
        clouds_modified_[i] = false;
        cout << input_topics_[i] << " ";
      }
    } else
      ROS_INFO("Not subscribed to any topic.");
  }
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan,
                                   std::string topic) {

  // Verify that TF knows how to transform from the received scan to the
  // destination scan frame
  /*tfListener_.waitForTransform(scan->header.frame_id.c_str(),
                               destination_frame_.c_str(), scan->header.stamp,
                               ros::Duration(1));
  projector_.transformLaserScanToPointCloud(
      scan->header.frame_id, *scan, tmpCloud1, tfListener_,
      laser_geometry::channel_option::Distance);
  try {
    tfListener_.transformPointCloud(destination_frame_.c_str(), tmpCloud1,
                                    tmpCloud2);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }*/

  // Verify that TF knows how to transform from the received scan to the
  // destination scan frame
  // tf_buffer_.waitForTransform(destination_frame_.c_str(),
  // scan->header.frame_id.c_str(), scan->header.stamp, ros::Duration(1));
  // printf("Received topic %s in the frame %s\n", topic.c_str(),
  //       scan->header.frame_id.c_str());
  bool ok = true;
  try {
    bool ok = tf_buffer_->canTransform(
        destination_frame_.c_str(), scan->header.frame_id.c_str(),
        scan->header.stamp +
            ros::Duration().fromSec(scan->ranges.size() * scan->time_increment),
        ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("tranformException. Could NOT transform from %s to %s: %s",
              scan->header.frame_id.c_str(), destination_frame_.c_str(),
              ex.what());
    return;
  } catch (tf2::LookupException &ex) {
    ROS_ERROR("LookupException. Could NOT transform from %s to %s: %s",
              scan->header.frame_id.c_str(), destination_frame_.c_str(),
              ex.what());
  }
  if (!ok) {
    ROS_ERROR("Could NOT transform from %s to %s.",
              scan->header.frame_id.c_str(), destination_frame_.c_str());
    return;
  }

  // The concatenation of pointclouds (pcl::concatena) is giving me problems
  // because of different fiels in the pointclouds. And using the
  // channel_options in the transformLaserScanToPointCloud is not working
  // properly. So, I'm filling the intensity field in all the scans.
  sensor_msgs::LaserScan laser = *scan;
  if (scan->intensities.empty())
    laser.intensities.resize(scan->ranges.size(), 0.0);

  sensor_msgs::PointCloud2 cloud;
  // sensor_msgs::PointCloud cloud;
  if (scan->header.frame_id != destination_frame_) {
    try {
      projector_.transformLaserScanToPointCloud(
          destination_frame_, laser, cloud,
          *tf_buffer_); // laser_geometry::channel_option::Distance
      // | None | Intensity
      // projector_.transformLaserScanToPointCloud(destination_frame_, laser,
      //                                          cloud, tf_listener_);
    } catch (const std::exception &ex) {
      ROS_ERROR("[%s] Topic: %s. Error transforming laser to point cloud in "
                "other frame: %s",
                topic.c_str(), ros::this_node::getName().c_str(), ex.what());
      return;
    }
  } else {
    try {
      projector_.projectLaser(laser, cloud);
    } catch (const std::exception &ex) {
      ROS_ERROR("[%s] Error projecting laser to point cloud: %s",
                ros::this_node::getName().c_str(), ex.what());
      return;
    }
  }

  for (int i = 0; i < input_topics_.size(); ++i) {
    if (topic.compare(input_topics_[i]) == 0) {
      // sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
      pcl_conversions::toPCL(cloud, clouds_[i]);
      clouds_modified_[i] = true;
    }
  }

  // Count how many scans we have
  int totalClouds = 0;
  for (int i = 0; i < clouds_modified_.size(); ++i)
    if (clouds_modified_[i])
      ++totalClouds;

  // Go ahead only if all subscribed scans have arrived
  if (totalClouds == clouds_modified_.size()) {
    pcl::PCLPointCloud2 merged_cloud = clouds_[0];
    clouds_modified_[0] = false;

    for (int i = 1; i < clouds_modified_.size(); ++i) {
      pcl::concatenatePointCloud(merged_cloud, clouds_[i],
                                 merged_cloud); // Deprecated

      // if (!pcl::concatenate(merged_cloud, clouds[i],
      // merged_cloud))
      //  ROS_ERROR("[%s] Concatenate pointclouds failed!",
      //            ros::this_node::getName().c_str());
      // merged_cloud += clouds_[i];
      clouds_modified_[i] = false;
    }

    point_cloud_publisher_.publish(merged_cloud);

    Eigen::MatrixXf points;
    getPointCloudAsEigen(merged_cloud, points);

    pointcloud_to_laserscan(points, &merged_cloud);
  }
}

void LaserscanMerger::pointcloud_to_laserscan(
    Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud) {
  sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
  output->header = pcl_conversions::fromPCL(merged_cloud->header);
  output->header.frame_id = destination_frame_.c_str();
  output->header.stamp = ros::Time::now(); // fixes #265
  output->angle_min = this->angle_min_;
  output->angle_max = this->angle_max_;
  output->angle_increment = this->angle_increment_;
  output->time_increment = this->time_increment_;
  output->scan_time = this->scan_time_;
  output->range_min = this->range_min_;
  output->range_max = this->range_max_;

  uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) /
                                   output->angle_increment);
  output->ranges.assign(ranges_size, output->range_max + 1.0);

  for (int i = 0; i < points.cols(); i++) {
    const float &x = points(0, i);
    const float &y = points(1, i);
    const float &z = points(2, i);

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
      continue;
    }

    double range_sq = y * y + x * x;
    double range_min_sq_ = output->range_min * output->range_min;
    if (range_sq < range_min_sq_) {
      ROS_DEBUG(
          "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
          range_sq, range_min_sq_, x, y, z);
      continue;
    }

    double angle = atan2(y, x);
    if (angle < output->angle_min || angle > output->angle_max) {
      ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle,
                output->angle_min, output->angle_max);
      continue;
    }
    int index = (angle - output->angle_min) / output->angle_increment;

    if (output->ranges[index] * output->ranges[index] > range_sq)
      output->ranges[index] = sqrt(range_sq);
  }

  laser_scan_publisher_.publish(output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_multi_merger");

  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  LaserscanMerger _laser_merger(nh, &tf_buffer);

  dynamic_reconfigure::Server<laserscan_mergerConfig> server;
  dynamic_reconfigure::Server<laserscan_mergerConfig>::CallbackType f;

  f = boost::bind(&LaserscanMerger::reconfigureCallback, &_laser_merger, _1,
                  _2);
  server.setCallback(f);

  ros::spin();

  return 0;
}
