#include <ros/ros.h>
#include <fstream> 
#include <string> 
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
using namespace std;

ros::Publisher pub;


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  vector<int> indices;
  vector<int> indices2;
  //en la linea de abajo hay forma de darle 2 argumentos mas, arreglar eso para verlo en rviz
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/donaxi/catkin_ws/src/registration_pcl/src/cloud1.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
   
  }
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/donaxi/catkin_ws/src/registration_pcl/src/cloud2.pcd", *cloud2) == -1) //* load the 2nd file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
   
  }


  pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
  pcl::removeNaNFromPointCloud(*cloud2, *outputCloud2, indices2);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(outputCloud);
  icp.setInputTarget(outputCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);

  pcl::io::savePCDFile("/home/donaxi/catkin_ws/src/registration_pcl/src/final.pcd", *Final);
  pcl::io::savePCDFile("/home/donaxi/catkin_ws/src/registration_pcl/src/aCloud1.pcd", *outputCloud);
  pcl::io::savePCDFile("/home/donaxi/catkin_ws/src/registration_pcl/src/aCloud2.pcd", *outputCloud2);
  pcl::visualization::CloudViewer viewer ("final");
  viewer.showCloud (Final);
  while (!viewer.wasStopped ())
   {
   }

  //Conversion y salida
  sensor_msgs::PointCloud2 output_read;
  pcl::toROSMsg (*cloud, output_read);
  

  // Publish the data
  pub.publish (output_read);
}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "registration_pcl");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_read", 1);

  // Spin
  ros::spin();
  
}
