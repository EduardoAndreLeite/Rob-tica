#include <iostream>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MappingNode : public rclcpp::Node
{
public:
  MappingNode(Perception &perception)
      : Node("mapping_p3dx"), perception_(perception)
  {
    // Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize subscribers
    sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                           std::bind(&Perception::receiveLaser, &perception, _1));
    sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
                                                                         std::bind(&Perception::receiveSonar, &perception, _1));

    // Initialize publishers
    pub_mapLaserLogOdds_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_laser_log_odds", 1);
    pub_mapLaserHIMM_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_laser_HIMM", 1);
    pub_mapSonar_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_sonar", 1);

    // Initialize timer callback
    timer_ = this->create_wall_timer(100ms, std::bind(&MappingNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Get latest sensor readings
    std::vector<float> lasers = perception_.getLatestLaserRanges();
    std::vector<float> sonars = perception_.getLatestSonarRanges();
    RCLCPP_INFO(this->get_logger(), "Read %ld laser measurements", lasers.size());
    RCLCPP_INFO(this->get_logger(), "Read %ld laser measurements", sonars.size());

    // Get current robot pose given by ODOM
    Pose2D robotPose;
    geometry_msgs::msg::TransformStamped transformStamped;
    bool validPose=true;
    try{ 
      transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero); 
    } catch (const tf2::TransformException & ex) { 
      RCLCPP_INFO(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
      validPose=false;
    }

    if(validPose){
      // Update robotPose from robot transformation
      robotPose.x = transformStamped.transform.translation.x;
      robotPose.y = transformStamped.transform.translation.y;

      // Convert quaternion to euler angles
      tf2::Quaternion q4(transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y, 
                        transformStamped.transform.rotation.z, 
                        transformStamped.transform.rotation.w);
      tf2::Matrix3x3 m4(q4);
      double roll, pitch, yaw;
      m4.getRPY(roll,pitch,yaw);

      // Update orientation with yaw
      robotPose.theta = RAD2DEG(yaw);
      RCLCPP_INFO(this->get_logger(), "Robot pose (%f %f %f)", robotPose.x, robotPose.y, robotPose.theta);

      // Update maps
      nav_msgs::msg::OccupancyGrid& msg_mapLaserLogOdds = perception_.updateMapLaserWithLogOdds(lasers, robotPose);
      nav_msgs::msg::OccupancyGrid& msg_mapLaserHIMM = perception_.updateMapLaserWithHIMM(lasers, robotPose);
      nav_msgs::msg::OccupancyGrid& msg_mapSonar = perception_.updateMapSonar(sonars, robotPose);

      // Publish maps
      pub_mapLaserLogOdds_->publish(msg_mapLaserLogOdds);
      pub_mapLaserHIMM_->publish(msg_mapLaserHIMM);
      pub_mapSonar_->publish(msg_mapSonar);
      RCLCPP_INFO(this->get_logger(), "Published maps");
    }
  }

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapLaserLogOdds_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapLaserHIMM_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapSonar_;

  Perception &perception_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  Perception perception(100, 100, 0.1);

  std::shared_ptr<MappingNode> nh = std::make_shared<MappingNode>(perception);
  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
