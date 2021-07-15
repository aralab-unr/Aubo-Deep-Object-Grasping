#include <ros/ros.h>
#include <iostream>

// PCL specific includes
#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree_search.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/filters/voxel_grid.h>
//using namespace message_filters;

ros::Publisher pub;

int minX;
int minY;
int maxX;
int maxY;
int YoloCenterPointX;
int YoloCenterPointY;
int endCallback2 = 0;
int endCallback1 = 0;


void cloud_cb_2(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{

    if(endCallback2 == 0){
        std::string my_choice = "";
        std::cout << "Choose what object you want to grasp: ";
        std::cin >> my_choice;
        std::cout << std::endl;
    

    for(int i = 0; i < 5; i++){
        
            if(msg->bounding_boxes[i].Class == my_choice)
            {
                
                minX = msg->bounding_boxes[i].xmin;
                minY = msg->bounding_boxes[i].ymin;
                maxX = msg->bounding_boxes[i].xmax;
                maxY = msg->bounding_boxes[i].ymax;

                std::cout << "bounding box_boxes: " << msg->bounding_boxes[i].Class << std::endl;
                std::cout << "bounding box minX" << minX << std::endl;
                std::cout << "bounding box minY" << minY << std::endl;
                std::cout << "bounding box maxX" << maxX << std::endl;
                std::cout << "bounding box maxY" << maxY << std::endl;

                YoloCenterPointX = (maxX + minX)/2;
                YoloCenterPointY = (maxY + minY)/2;
               

            }
        
    }
    endCallback2 = 1;
    }
    

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
if(endCallback1 == 0){
    
 // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloudFilteredPtr);


  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.78, 1.1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);
  

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.005);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);



  // perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);
  sensor_msgs::PointCloud2 output; 


  pcl::PCLPointCloud2 outputPCL;

  int EuclideanDistance, distance_x, distance_y;
  int threshold = 30;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); 
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) 
{ 
    Eigen::Vector4f centroid3D;
    Eigen::Vector2i centroid2D;
    Eigen::Matrix3f camera_matrix;
    camera_matrix <<  547.471175, 0.000000, 313.045026, 0.000000, 547.590335, 237.016225, 0.000000, 0.000000, 1.000000;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        cloud_cluster->points.push_back (xyzCloudPtrRansacFiltered->points[*pit]); 
    }    
    cloud_cluster->width = cloud_cluster->points.size (); 
    cloud_cluster->height = 1; 
    cloud_cluster->is_dense = true; 
    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl; 
    pcl::compute3DCentroid(*xyzCloudPtrRansacFiltered, *it, centroid3D);
    
    Eigen::Vector2i pixel_position;

    pixel_position(0) = (int)(centroid3D(0)*camera_matrix(0,0)/centroid3D(2) + camera_matrix(0,2));
    pixel_position(1) = (int)(centroid3D(1)*camera_matrix(1,1)/centroid3D(2) + camera_matrix(1,2));
    
    centroid2D = pixel_position;


    distance_x = abs(YoloCenterPointX - centroid2D(0));
    std::cout << "YOlo centerx " << YoloCenterPointX << std::endl;
    std::cout << "controidx " << centroid2D(0) << std::endl;
    std::cout <<  "The distance in x axis: " << distance_x << std::endl;
    distance_y = abs(YoloCenterPointY - centroid2D(1));
    std::cout <<  "The distance in y axis: " << distance_y << std::endl;
    EuclideanDistance = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
    std::cout <<  "The aggregated distance: " << EuclideanDistance << std::endl;
    
    if(EuclideanDistance < threshold){
        pcl::toPCLPointCloud2 (*cloud_cluster, outputPCL); 
        pcl_conversions::fromPCL(outputPCL, output);
        output.header.frame_id = "/camera_rgb_optical_frame";

        ros::Rate loop_rate(1);
        while(ros::ok()){
            pub.publish(output);
            ros::spinOnce();
            loop_rate.sleep();
        }

    }
    
    //centroid2D = point3Dto_pixel(centroid3D, camera_matrix);


}
endCallback1++;

}
}




int main (int argc, char** argv){
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");
ros::NodeHandle nh;
ros::NodeHandle m_nh;

//subscribe to the pointcloud
ros::Subscriber sub = nh.subscribe ("/camera_remote/depth_registered/points", 1, cloud_cb);
//subscribe to the bouding boxes from darknet_ros
ros::Subscriber object_detection = m_nh.subscribe("/darknet_ros/bounding_boxes", 1, cloud_cb_2);
   
pub = nh.advertise<sensor_msgs::PointCloud2> ("output_filtered_cloud", 10);

ros::spin();




}
