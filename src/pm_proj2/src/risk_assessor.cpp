#include "pm_proj2.h"


ros::Publisher ttc_pub;
ros::Publisher ttc_marker_pub;



void callback_dist(const geometry_msgs::Point &dist) {
  distance = dist;
}

void callback_bb(const visualization_msgs::Marker &marker) {
  p_min = marker.points.at(0); //min min
  p_max = marker.points.at(10); //max max

  ang1 = atan(p_min.y/p_min.x);
  ang2 = atan(p_min.y/p_max.x);
  ang3 = atan(p_max.y/p_min.x);
  ang4 = atan(p_max.y/p_max.x);

  ang_max = std::max({ang1,ang2,ang3,ang4});
  ang_min = std::min({ang1,ang2,ang3,ang4});


  if (vel_ang <= ang_max && vel_ang >= ang_min) {
    ttc = (tf::Vector3(distance.x,distance.y,0).length()) / tf::Vector3(velocity.x,velocity.y,0).length();
  }
  else {
    //ROS_INFO("No collision");
    ttc = 0;
  }


  file.open("ttc.txt", std::ios::app);
  file << ttc << " " << ros::Time::now() << "\n";
  file.close();

  // Publish the TTC on the "risk_assessor/time_to_collision" topic
  std::cout << ttc << "\n";
  std_msgs::Float32 ttc_msg;
  ttc_msg.data = ttc;
  ttc_pub.publish(ttc_msg);
}



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

  vel_ang = atan(velocity.y/velocity.x);
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

  std::vector<double> all_x,all_y,all_z;

  for (int i = 0; i < cloud->points.size(); i++) {
    all_x.push_back(cloud->points[i].x);
    all_y.push_back(cloud->points[i].y);
    all_z.push_back(cloud->points[i].z);
  }

  double min_x = *min_element(all_x.begin(), all_x.end());
  double max_x = *max_element(all_x.begin(), all_x.end());
  double min_y = *min_element(all_y.begin(), all_y.end());
  double max_y = *max_element(all_y.begin(), all_y.end());
  //double min_z = *min_element(all_z.begin(), all_z.end());
  double max_z = *max_element(all_z.begin(), all_z.end());
  


  
  // Create a marker message to display the TTC value as text
  ttc_marker.header.frame_id = msg->header.frame_id;
  ttc_marker.ns = "os_sensor";
  ttc_marker.id = 0;
  ttc_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  ttc_marker.action = visualization_msgs::Marker::ADD;
  ttc_marker.text = "TTC = " + std::to_string(ttc) + " s";
  if (ttc!= 0) {
    ttc_marker.scale.z = 3/ttc;  // Set the font size
  }
  else {
    ttc_marker.scale.z = 0.1;  // Set the font size
  }
  ttc_marker.color.r = 1.0;
  ttc_marker.color.g = 1.0;
  ttc_marker.color.b = 1.0;
  ttc_marker.color.a = 1.0;
  ttc_marker.pose.position.x = (max_x-min_x)/2;
  ttc_marker.pose.position.y = (max_y-min_y)/2;
  ttc_marker.pose.position.z = max_z+1;
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

  ros::Subscriber sub_bb = nh.subscribe("detector/bbox", 1, callback_bb);

  ros::Subscriber sub_dist = nh.subscribe("detector/dist", 1, callback_dist);
  // Create a publisher for the "risk_assessor/time_to_collision" topic
  ttc_pub = nh.advertise<std_msgs::Float32>("risk_assessor/time_to_collision", 1000);
  // Create a publisher for the ttc_marker topic
  ttc_marker_pub = nh.advertise<visualization_msgs::Marker>("risk_assessor/ttc_marker", 1);

  // Spin and process messages
  ros::spin();

  return 0;
}
