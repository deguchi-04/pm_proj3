#include "pm_proj2.h"

ros::Publisher pub_obs; 
ros::Publisher pub_dist;
ros::Publisher pub_bb;

void callback_point_cloud(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{

    // ref: https://pcl.readthedocs.io/en/latest/cluster_extraction.html#compiling-and-running-the-program
    pcl::PCLPointCloud2 point_cloud;
    pcl_conversions::toPCL(*pcl_msg, point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(point_cloud, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_pcl(new pcl::PointCloud<pcl::PointXYZ>);
   
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    //std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.15f, 0.15f, 0.15f);
    vg.filter(*cloud_filtered);
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointXYZ closest_center, center;
    pcl::PointXYZ origin(0, 0, 0);
    float min_dist, dist;
    std_msgs::Float32 Dist;
    min_dist = INFINITY;
    for (const auto &cluster : cluster_indices)
    {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
            centroid.add((*cloud_filtered)[idx]);
        }
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //ref: https://pointclouds.org/documentation/classpcl_1_1_centroid_point.html
        centroid.get(center);
        std::cout << center.x << " " << center.y << " " << center.z << std::endl;
        dist = pcl::euclideanDistance(origin, center);
        if (dist < min_dist)
        {
            min_dist = dist;
            *result_pcl = *cloud_cluster;
        }
    }


    // ref: https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
    // ref: https://stackoverflow.com/questions/49688940/point-cloud-library-rotation-of-axis-alligned-bounding-box
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(result_pcl);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ position_OBB;

    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    Eigen::Vector3f p1(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p2(min_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p3(max_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p4(max_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p5(min_point_AABB.x, max_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p6(min_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p7(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p8(max_point_AABB.x, max_point_AABB.y, min_point_AABB.z);

    // ref: http://wiki.ros.org/rviz/DisplayTypes/Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/os_sensor";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = "bbox";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point P_1, P_2, P_3, P_4, P_5, P_6, P_7, P_8;

    P_1.x = p1.x();
    P_1.y = p1.y();
    P_1.z = p1.z();
    P_2.x = p2.x();
    P_2.y = p2.y();
    P_2.z = p2.z();
    P_3.x = p3.x();
    P_3.y = p3.y();
    P_3.z = p3.z();
    P_4.x = p4.x();
    P_4.y = p4.y();
    P_4.z = p4.z();
    P_5.x = p5.x();
    P_5.y = p5.y();
    P_5.z = p5.z();
    P_6.x = p6.x();
    P_6.y = p6.y();
    P_6.z = p6.z();
    P_7.x = p7.x();
    P_7.y = p7.y();
    P_7.z = p7.z();
    P_8.x = p8.x();
    P_8.y = p8.y();
    P_8.z = p8.z();

    marker.points.push_back(P_1);
    marker.points.push_back(P_2);
    marker.points.push_back(P_3);
    marker.points.push_back(P_4);
    marker.points.push_back(P_1);
    marker.points.push_back(P_4);
    marker.points.push_back(P_2);
    marker.points.push_back(P_3);

    marker.points.push_back(P_5);
    marker.points.push_back(P_6);
    marker.points.push_back(P_7);
    marker.points.push_back(P_8);
    marker.points.push_back(P_5);
    marker.points.push_back(P_8);
    marker.points.push_back(P_6);
    marker.points.push_back(P_7);

    marker.points.push_back(P_1);
    marker.points.push_back(P_5);
    marker.points.push_back(P_2);
    marker.points.push_back(P_6);
    marker.points.push_back(P_3);
    marker.points.push_back(P_7);
    marker.points.push_back(P_4);
    marker.points.push_back(P_8);

    

    sensor_msgs::PointCloud2 output;
    result_pcl->header.frame_id = "os_sensor";
    pcl::toROSMsg(*result_pcl, output);
    pub_bb.publish(marker);
    pub_obs.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, callback_point_cloud);

    // Publishers
    pub_obs = n.advertise<sensor_msgs::PointCloud2>("detector/obstacle", 1);
    pub_bb = n.advertise<visualization_msgs::Marker>("detector/bbox", 1);

    
        ros::spin();
    
    return 0;
}
