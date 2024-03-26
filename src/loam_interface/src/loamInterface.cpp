#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
nav_msgs::msg::Odometry odomData;
tf2::Stamped<tf2::Transform> odomTrans;
geometry_msgs::msg::TransformStamped transformTfGeom ; 
std::vector<int> scanInd;
class LoamInterface : public rclcpp::Node
{
public:
  LoamInterface()
  : Node("loamInterface"),
  tfBuffer(std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>())), 
  tfListener(std::make_shared<tf2_ros::TransformListener>(*tfBuffer))
  {
    // Declare Parameters
    this->declare_parameter<std::string>("stateEstimationTopic", stateEstimationTopic);
    this->declare_parameter<std::string>("registeredScanTopic", registeredScanTopic);
    this->declare_parameter<bool>("flipStateEstimation", flipStateEstimation);
    this->declare_parameter<bool>("flipRegisteredScan", flipRegisteredScan);
    this->declare_parameter<bool>("combinedOdom", combinedOdom);
    this->declare_parameter<bool>("sendTF", sendTF);
    this->declare_parameter<bool>("reverseTF", reverseTF);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame);

    // Initialize Parameters
    this->get_parameter("stateEstimationTopic", stateEstimationTopic);
    this->get_parameter("registeredScanTopic", registeredScanTopic);
    this->get_parameter("flipStateEstimation", flipStateEstimation);
    this->get_parameter("flipRegisteredScan", flipRegisteredScan);
    this->get_parameter("combinedOdom", combinedOdom);
    this->get_parameter("sendTF", sendTF);
    this->get_parameter("reverseTF", reverseTF);

    tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 5);
    pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);

    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      registeredScanTopic, 5, std::bind(&LoamInterface::laserCloudHandler, this, std::placeholders::_1));
    subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      stateEstimationTopic, 5, std::bind(&LoamInterface::odometryHandler, this, std::placeholders::_1));

  }

private:

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn) const
  {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);
    pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, scanInd);

    if (flipRegisteredScan) {
      int laserCloudSize = laserCloud->points.size();
      for (int i = 0; i < laserCloudSize; i++) {
        float temp = laserCloud->points[i].x;
        laserCloud->points[i].x = laserCloud->points[i].z;
        laserCloud->points[i].z = laserCloud->points[i].y;
        laserCloud->points[i].y = temp;
      }
    }
    if (combinedOdom){
        // Transform point cloud from sensor frame to odom frame
        sensor_msgs::msg::PointCloud2 transformedCloud;
        geometry_msgs::msg::TransformStamped transformStamped;

        try {
            // Replace "odom" with the name of your robot's odometry frame
            // and "laser_frame_id" with the frame ID of your laser sensor
            transformStamped = tfBuffer->lookupTransform(this->odom_frame, laserCloudIn->header.frame_id,
                                                         laserCloudIn->header.stamp, rclcpp::Duration::from_seconds(1.0));
            tf2::doTransform(*laserCloudIn, transformedCloud, transformStamped);

            // Convert transformed ROS message back to PCL for further processing
            laserCloud->clear();
            pcl::PointCloud<pcl::PointXYZ> finalCloud;
            pcl::fromROSMsg(transformedCloud, *laserCloud);
            pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, scanInd);
            // Now finalCloud contains the point cloud transformed into the odometry frame
            // You can continue processing finalCloud as needed
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("laserCloudHandler"), "Could NOT transform laser scan: %s", ex.what());
            return;
        }
    }
    // publish registered scan messages
    sensor_msgs::msg::PointCloud2 laserCloud2;
    pcl::toROSMsg(*laserCloud, laserCloud2);
    laserCloud2.header.stamp = laserCloudIn->header.stamp;
    laserCloud2.header.frame_id = this->odom_frame;
    pubLaserCloud->publish(laserCloud2);
  }

  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom) const
  {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    odomData = *odom;

    if (flipStateEstimation) {
      tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

      pitch = -pitch;
      yaw = -yaw;

      tf2::Quaternion quat_tf;
      quat_tf.setRPY(roll, pitch, yaw);
      tf2::convert(quat_tf, geoQuat);

      odomData.pose.pose.orientation = geoQuat;
      odomData.pose.pose.position.x = odom->pose.pose.position.z;
      odomData.pose.pose.position.y = odom->pose.pose.position.x;
      odomData.pose.pose.position.z = odom->pose.pose.position.y;
    }

    // publish odometry messages
    odomData.header.frame_id = this->odom_frame;
    odomData.child_frame_id = "base_link";
    pubOdometry->publish(odomData);

    // // publish tf messages
    odomTrans.frame_id_ = this->odom_frame;
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

    if (sendTF) {
      if (!reverseTF) {
        transformTfGeom = tf2::toMsg(odomTrans);
        transformTfGeom.child_frame_id = "base_link";
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      } 
      else{
        transformTfGeom.transform = tf2::toMsg(odomTrans.inverse());
        transformTfGeom.header.frame_id = this->odom_frame;
        transformTfGeom.child_frame_id = "base_link";
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      }
    }

    // //for registered scan
    // odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    // odomTimeStack[odomSendIDPointer] = odomTime.seconds();
    // vehicleXStack[odomSendIDPointer] = vehicleX;
    // vehicleYStack[odomSendIDPointer] = vehicleY;
    // vehicleZStack[odomSendIDPointer] = vehicleZ;
    // vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    // vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    // vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    // terrainRollStack[odomSendIDPointer] = terrainRoll;
    // terrainPitchStack[odomSendIDPointer] = terrainPitch;


  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    // tf2 Buffer and Listener to listen to transforms
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;
  const double PI = 3.1415926;

  string stateEstimationTopic = "/integrated_to_init";
  string registeredScanTopic = "/velodyne_cloud_registered";
  string odom_frame;
  bool flipStateEstimation = true;
  bool flipRegisteredScan = true;
  bool combinedOdom = true;
  bool sendTF = true;
  bool reverseTF = false;
  // static const int stackNum = 400;
  // float vehicleX = 0;
  // float vehicleY = 0;
  // float vehicleZ = 0;
  // float vehicleRoll = 0;
  // float vehiclePitch = 0;
  // float vehicleYaw = 0;
  // float vehicleXStack[stackNum];
  // float vehicleYStack[stackNum];
  // float vehicleZStack[stackNum];
  // float vehicleRollStack[stackNum];
  // float vehiclePitchStack[stackNum];
  // float vehicleYawStack[stackNum];
  // float terrainRollStack[stackNum];
  // float terrainPitchStack[stackNum];
  // double odomTimeStack[stackNum];
  // int odomSendIDPointer = -1;
  // int odomRecIDPointer = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoamInterface>());
  rclcpp::shutdown();
  
  return 0;
}
