#include "pm_proj2.h"
// Variables to store the IMU and odometry data
geometry_msgs::Quaternion orientation;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Point position;
geometry_msgs::Vector3 velocity;
ros::Publisher ttc_pub;
ros::Publisher ttc_marker_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  orientation = msg->orientation;
  angular_velocity = msg->angular_velocity;
  linear_acceleration = msg->linear_acceleration;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  position = msg->pose.pose.position;
  velocity = msg->twist.twist.linear;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert the ROS point cloud message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Check if the point cloud is empty (no obstacle detected)
  if (cloud->points.empty())
  {
    ROS_INFO("No obstacle detected");
    return;
  }

  // Get the position of the obstacle
  pcl::PointXYZ obstacle_position = cloud->points[0]; // Assume the obstacle is represented by the first point in the cloud

  // Estimate the position of the vehicle using the IMU and odometry data
  // Convert the quaternion orientation to a 3x3 rotation matrix
  tf::Matrix3x3 rot(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  // Get the angular velocity of the vehicle in the inertial frame
  tf::Vector3 angular_velocity_inertial = rot * tf::Vector3(angular_velocity.x, angular_velocity.y, angular_velocity.z);
  // Estimate the position of the vehicle in the inertial frame
  tf::Vector3 position_inertial(position.x, position.y, position.z);
  // Estimate the velocity of the vehicle in the inertial frame
  tf::Vector3 velocity_inertial(velocity.x, velocity.y, velocity.z);
  // Estimate the acceleration of the vehicle in the inertial frame
  tf::Vector3 acceleration_inertial = rot * tf::Vector3(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z) - angular_velocity_inertial.cross(velocity_inertial);

  // Compute the relative position and velocity of the obstacle with respect to the vehicle
  tf::Vector3 obstacle_position_rel = position_inertial - tf::Vector3(obstacle_position.x, obstacle_position.y, obstacle_position.z);
  tf::Vector3 obstacle_velocity_rel = velocity_inertial; // Assume the obstacle is static

  // Compute the time to collision (TTC) using the relative position and velocity
  double ttc = (obstacle_position_rel.length()) / (obstacle_velocity_rel - velocity_inertial).length(); // 2.0 is a safety distance to account for vehicle size and reaction time

  ROS_INFO("TTC: %f seconds", ttc);

  std::string ttc_file = ros::package::getPath("pm_proj2") + "/file/ttc.txt";

  FILE* file = fopen(ttc_file.c_str(), "a");

  if (file == NULL)
  {
    ROS_ERROR("Unable to open file: %s", ttc_file.c_str());
    return;
  }

  fprintf(file, "%f %f\n", ttc, msg->header.stamp.toSec());

  fclose(file);

  // Publish the TTC on the "risk_assessor/time_to_collision" topic
  std_msgs::Float32 ttc_msg;
  ttc_msg.data = ttc;
  ttc_pub.publish(ttc_msg);

  // Create a marker message to display the TTC value as text
  visualization_msgs::Marker ttc_marker;
  ttc_marker.header.frame_id = msg->header.frame_id;
  ttc_marker.ns = "os_sensor";
  ttc_marker.id = 0;
  ttc_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  ttc_marker.action = visualization_msgs::Marker::ADD;
  ttc_marker.text = "TTC = " + std::to_string(ttc) + " s";
  ttc_marker.scale.z = 0.1;  // Set the font size
  ttc_marker.color.r = 1.0;
  ttc_marker.color.g = 1.0;
  ttc_marker.color.b = 1.0;
  ttc_marker.color.a = 1.0;
  ttc_marker.lifetime = ros::Duration();

  // Publish the marker message
  ttc_marker_pub.publish(ttc_marker);
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "risk_assessor");
  ros::NodeHandle nh;

  // Subscribe to the "/imu_nav/data" topic
  ros::Subscriber imu_sub = nh.subscribe("/imu_nav/data", 1000, imuCallback);
  // Subscribe to the "/gps/rtkfix" topic
  ros::Subscriber odometry_sub = nh.subscribe("/gps/rtkfix", 1000, odometryCallback);
  // Subscribe to the "detector/obstacle" topic
  ros::Subscriber cloud_sub = nh.subscribe("detector/obstacle", 1000, cloudCallback);
  // Create a publisher for the "risk_assessor/time_to_collision" topic
  ttc_pub = nh.advertise<std_msgs::Float32>("risk_assessor/time_to_collision", 1000);
  // Create a publisher for the ttc_marker topic
  ttc_marker_pub = nh.advertise<visualization_msgs::Marker>("risk_assessor/ttc_marker", 1);

  // Spin and process messages
  ros::spin();

  return 0;
}
