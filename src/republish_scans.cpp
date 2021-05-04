
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

class LaserscanThrottle {
public:
  LaserscanThrottle(ros::NodeHandle *n);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan,
                    std::string topic);

  void publishScans();

private:
  ros::NodeHandle *nh_;
  std::vector<ros::Publisher> scan_publishers_;
  std::vector<ros::Subscriber> scan_subscribers_;
  std::vector<std::string> scan_topics_;
  std::vector<sensor_msgs::LaserScan> scans_;
  std::vector<bool> received_;
};

LaserscanThrottle::LaserscanThrottle(ros::NodeHandle *n) {

  nh_ = n;
  ros::NodeHandle nh;
  int i = 0;
  bool ok = true;
  while (ok) {
    char buf[25];
    sprintf(buf, "scan_topic_%u", (i + 1));
    std::string st = std::string(buf);

    if (nh_->hasParam(st.c_str())) {
      std::string rt;
      nh_->getParam(st.c_str(), rt);
      scan_topics_.push_back(rt);

      // Subcribe to topic
      ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>(
          rt.c_str(), 1,
          boost::bind(&LaserscanThrottle::scanCallback, this, _1, rt));
      scan_subscribers_.push_back(sub);
      printf("%s. subscribed to topic: %s\n", ros::this_node::getName().c_str(),
             rt.c_str());

      // Declare the publishers
      // std::string topic = rt + "_throttle";
      ros::Publisher pub =
          nh_->advertise<sensor_msgs::LaserScan>(rt.c_str(), 1, false);
      scan_publishers_.push_back(pub);
      i++;
    } else
      ok = false;
  }
  received_.resize(scan_topics_.size(), false);
  scans_.resize(scan_topics_.size());
}

void LaserscanThrottle::publishScans() {
  for (bool b : received_)
    if (!b)
      return;

  for (unsigned int i = 0; i < scans_.size(); i++) {
    scans_[i].header.stamp = ros::Time::now();
    scan_publishers_[i].publish(scans_[i]);
  }
  received_.resize(scan_topics_.size(), false);
}

void LaserscanThrottle::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan, std::string topic) {

  for (unsigned int i = 0; i < scan_topics_.size(); i++) {
    if (topic == scan_topics_[i]) {
      scans_[i] = *scan;
      received_[i] = true;
    }
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "republish_scans");

  ros::NodeHandle n("~");

  LaserscanThrottle laser_throttle(&n);

  float freq;
  n.param<float>("publish_frequency", freq, 10.0);

  ros::NodeHandle nh;
  ros::Rate r(freq);
  while (nh.ok()) {
    laser_throttle.publishScans();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}