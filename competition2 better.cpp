#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <fstream>

int id = 1;
int loop_counts = 0;
int lidar_count = 0;
float yaw = -0.1;
// try using madwick or complementary filter to get quaternion
int get_lidar_data = 0;
bool get_odom = false;
bool bad_fitness = false;
int fit_flag = 0;
//bool get_imu_data = false;
geometry_msgs::PointStamped gps_data;
geometry_msgs::PointStamped later_gps_data;
sensor_msgs::Imu imu_data;
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_data(new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::Odometry odom;

void gpsCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::cout << "gps_callback...";
  if(loop_counts == 0){
    loop_counts = 1;
    gps_data = *msg;
    std::cout << "get gps data";
  }
  later_gps_data = *msg;
  std::cout << "\n";
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  lidar_count += 1;
  std::cout << "lidar counts: " << lidar_count << std::endl;
  // Convert pointcloud2 msg to pointcloud2
  pcl::PCLPointCloud2::Ptr temp_pc2 (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr filtered_pc2 (new pcl::PCLPointCloud2 ());
  pcl_conversions::toPCL(*msg,*temp_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_lidar_data(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(*temp_pc2,*downsampled_lidar_data);  // pc2 to pointxyz
  // Radius Outlier Removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(downsampled_lidar_data);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (20);//18
  outrem.setKeepOrganized(true);
  // apply filter
  outrem.filter (*lidar_data);

  // //Create the filtering object
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (lidar_data);
  // // pass.setFilterFieldName ("x");
  // // pass.setFilterLimits (0.0, 0.1);
  // // pass.setFilterFieldName ("y");
  // // pass.setFilterLimits (0, 0.1);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (-1, 300); //0.9
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*lidar_filtered);

  pcl::toPCLPointCloud2(*lidar_data,*temp_pc2);  // pointxyz to pc2
  // Downsample pointxyz data
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (temp_pc2);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*filtered_pc2);

  // Convert pointcloud2 to pointxyz              
  pcl::fromPCLPointCloud2(*filtered_pc2,*lidar_data);  // pc2 to pointxyz

  get_lidar_data = 1;
}

// void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   std::cout << "imu_callback... get imu data" << std::endl;
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//   imu_data = *msg;
//   get_imu_data = true;
// }

int main (int argc, char** argv)
{
  std::ofstream outFile;
  outFile.open("data.csv", std::ios::out);
  outFile << "id" << ',' << "x" << ',' << "y" << ',' << "z" << ','
                    << "yaw" << ',' << "pitch" << ',' << "roll" << std::endl;

  // Initialize ROS
  ros::init (argc, argv, "competition2_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1.0);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber gps_sub = nh.subscribe ("gps", 1, gpsCallback);
  ros::Subscriber lidar_sub = nh.subscribe ("lidar_points", 1, lidarCallback);

  //ros::Subscriber imu_sub = nh.subscribe ("imu/data", 8, imuCallback);
  
  // Create a ROS publisher for the output data
  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("match", 1);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("lidar_odom", 1);

  // Transform /lidar_points from velodyne to base_link
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster br;

  // Use initial guess for first ICP
  Eigen::Matrix4f initial_guess;
  Eigen::Matrix4f updated_guess;
  tf2::Matrix3x3 rotation_matrix;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  geometry_msgs::PointStamped processing_gps_data;

  // Load map
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud3 (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2 ());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/alex/catkin_ws/src/midterm_competition2/src/downsampled.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file downsampled.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from nuscenes_map.pcd with the following fields: "
            << std::endl;

  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());

  // Radius Outlier Removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem2;
  // build the filter
  outrem2.setInputCloud(cloud);
  outrem2.setRadiusSearch(0.8);
  outrem2.setMinNeighborsInRadius (18); //<-map18
  outrem2.setKeepOrganized(true);
  // apply filter
  outrem2.filter (*cloud2);

  // Convert downsampled data to pointcloud2 msg
  pcl::toPCLPointCloud2(*cloud2, *cloud_filtered2);
  pcl_conversions::fromPCL(*cloud_filtered2, *output);
  output->header.frame_id = "world";
  pcl::fromPCLPointCloud2 (*cloud_filtered2,*cloud);
  std::cout << "map downsample complete..." << std::endl;

  //======================

  // Load map
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCLPointCloud2::Ptr cloud3 (new pcl::PCLPointCloud2 ());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2 ());
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/alex/catkin_ws/src/midterm_competition2/src/nuscenes_map.pcd", *cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file itri_map.pcd \n");
  //   return (-1);
  // }
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height
  //           << " data points from nuscenes_map.pcd with the following fields: "
  //           << std::endl;

  // // Radius Outlier Removal
  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem2;
  // // build the filter
  // outrem2.setInputCloud(cloud);
  // outrem2.setRadiusSearch(1);
  // outrem2.setMinNeighborsInRadius (8);//5
  // outrem2.setKeepOrganized(true);
  // // apply filter
  // outrem2.filter (*cloud2);

  // // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud2);
  // // pass.setFilterFieldName ("x");
  // // pass.setFilterLimits (0.0, 0.1);
  // // pass.setFilterFieldName ("y");
  // // pass.setFilterLimits (0, 0.1);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (1.3, 300); //0.9
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered);

  // // Downsample map
  // pcl::toPCLPointCloud2(*cloud_filtered, *cloud3);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud3);
  // sor.setLeafSize (0.5f, 0.5f, 0.5f);
  // sor.filter (*cloud_filtered2);
  // sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());

  // // Convert downsampled data to pointcloud2 msg
  // pcl_conversions::fromPCL(*cloud_filtered2, *output);
  // output->header.frame_id = "world";
  // //pcl::fromPCLPointCloud2 (*cloud_filtered2,*cloud);
  // std::cout << "map downsample complete..." << std::endl;

  // pcl::PCDWriter writer;
  // writer.write ("downsampled.pcd", *cloud_filtered2, 
  //        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  //=======================

  // Spin once to get initial guess
  while (nh.ok() && loop_counts == 0) 
  {
    std::cout << "waiting for initial gps data..." << std::endl;
    ros::spinOnce();
    loop_rate.sleep();  // this would make the program stop here..
  }

  // Disable gps subscriber (used for initial only)
  //gps_sub.shutdown();
  //imu_sub.shutdown();

  while(nh.ok())
  {
    
    map_pub.publish(output);  // publish map point cloud
    // tf transform
    geometry_msgs::TransformStamped transformStamped;
    
    try{
      transformStamped = tfBuffer.lookupTransform("car", "nuscenes_lidar", ros::Time(0));
      // loop_rate.sleep();
    }
    catch (tf2::TransformException &ex){
      ROS_WARN("%s",ex.what());
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_lidar_data (new pcl::PointCloud<pcl::PointXYZ>);
    if(get_lidar_data == 1){  // && get_imu_data == true
      std::cout << "get lidar data true..." << std::endl;
      pcl_ros::transformPointCloud(*lidar_data, *tf_lidar_data, transformStamped.transform);
      // tf_lidar_data = lidar_data;
      if(loop_counts == 1){
        std::cout << "--> initial guess" << std::endl;
        loop_counts = 2;
        tf2::Quaternion tf_quat (0,0,sin(yaw/2),cos(yaw/2));
        //tf2::Quaternion tf_quat (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
        tf2::Vector3 tf_vector (gps_data.point.x,gps_data.point.y,gps_data.point.z);
        tf2::Transform tftransform (tf_quat, tf_vector);
        rotation_matrix = tftransform.getBasis();
        // Build a Homogeneous Transformation Matrix (Rotaional matrix + Position vector)
        initial_guess << rotation_matrix[0][0] ,rotation_matrix[0][1] ,rotation_matrix[0][2] ,gps_data.point.x ,
                	       rotation_matrix[1][0] ,rotation_matrix[1][1] ,rotation_matrix[1][2] ,gps_data.point.y ,
           		           rotation_matrix[2][0] ,rotation_matrix[2][1] ,rotation_matrix[2][2] ,gps_data.point.z ,
            	   	       0 ,0 ,0 ,1;
        std::cout << initial_guess;
      }
      else{
        processing_gps_data = later_gps_data;
        std::cout << "--> update guess" << std::endl;
        initial_guess = updated_guess;
      }

      if(loop_counts ==2){
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored) 
        icp.setMaxCorrespondenceDistance (4);
        std::cout << "fuck" << std::endl;
        loop_counts = 3;
      }
      else if(bad_fitness == 1){
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distansces will be ignored) 
        icp.setMaxCorrespondenceDistance (3);
        std::cout << "shit" << std::endl;
        loop_counts = 3;
      }
      else{
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored) 
        icp.setMaxCorrespondenceDistance (2);
      }
      // Set the maximum number of iterations (criterion 1)  
      icp.setMaximumIterations (500);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (1e-10); 
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (0.001);
      // Set the input source and target
      icp.setInputSource (tf_lidar_data);
      icp.setInputTarget (cloud);

      // Perform the alignment
      pcl::PointCloud<pcl::PointXYZ> align_xyz;
      icp.align(align_xyz, initial_guess);

      float fit_score = 1;//icp.getFitnessScore();
      std::cout << "score: " << fit_score << std::endl;
       
      if(fit_score >= 9){
        bad_fitness = true;
        fit_flag += 1;
      }
      else{
        bad_fitness = false;
      }
      std::cout << "bad fitness score: " << fit_flag << "..." << std::endl;

      updated_guess = icp.getFinalTransformation();

      // Publish align data (Point cloud 2 msg) *PointXYZ->PCL2->PointCloud2 msg
      pcl::PCLPointCloud2::Ptr align_pcl2 (new pcl::PCLPointCloud2 ());
      pcl::toPCLPointCloud2 (align_xyz, *align_pcl2);
      sensor_msgs::PointCloud2::Ptr align_msg (new sensor_msgs::PointCloud2()); 
      pcl_conversions::fromPCL(*align_pcl2, *align_msg);
      align_msg->header.frame_id = "world";
      pcl_pub.publish(align_msg);

      // Publish Odometry results as nav msgs/Odometry.msg
      
      odom.header.frame_id = "world";
      odom.child_frame_id = "car";
      if(bad_fitness == true && fit_flag == 1){
        std::cout << "bad ICP results, use GPS data..." << std::endl;
        odom.pose.pose.position.x = processing_gps_data.point.x;
        odom.pose.pose.position.y = processing_gps_data.point.y;
        odom.pose.pose.position.z = processing_gps_data.point.z;
      }
      else{
        odom.pose.pose.position.x = updated_guess(0,3);
        odom.pose.pose.position.y = updated_guess(1,3);
        odom.pose.pose.position.z = updated_guess(2,3);
        if(fit_flag >= 3){
          fit_flag = 0;
        }
      }
      rotation_matrix.setValue(updated_guess(0,0) ,updated_guess(0,1) ,updated_guess(0,2) ,
                               updated_guess(1,0) ,updated_guess(1,1) ,updated_guess(1,2) ,
                               updated_guess(2,0) ,updated_guess(2,1) ,updated_guess(2,2));

      tf2::Quaternion tf_quat2;
      rotation_matrix.getRotation(tf_quat2);

      odom.pose.pose.orientation.x = tf_quat2[0];
      odom.pose.pose.orientation.y = tf_quat2[1];
      odom.pose.pose.orientation.z = tf_quat2[2];
      odom.pose.pose.orientation.w = tf_quat2[3];

      odom_pub.publish(odom);
      
      get_lidar_data = 0; 
      get_odom = true;
      //get_imu_data = true;

    }
    if(get_odom == true){
      std::cout << "save data... number: " << id <<std::endl;
      tf2::Quaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      outFile << id << ',' << updated_guess(0,3) << ',' << updated_guess(1,3) << ',' << updated_guess(2,3) << ','
                          << yaw << ',' << pitch << ',' << roll << std::endl;
      id++; 
      if(id > 396){
        outFile.close();
      }
      get_odom = false;
    }
    else{
      std::cout << "no odom data..." << std::endl;
    }
    std::cout << "=================" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

