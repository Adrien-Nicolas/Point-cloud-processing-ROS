#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class PlaneModelSegmentation {
public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    PlaneModelSegmentation() {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("SEGM_OUT",1);
        sub_ = nh_.subscribe("point_cloud_in", 1, &PlaneModelSegmentation::loudCallback, this);
    }

    ~PlaneModelSegmentation() {}

    void loudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    PointCloud::Ptr cloud (new PointCloud);
    pcl::fromROSMsg(*cloud_in, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.5);


    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    { 
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);

    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud, cloud_out);
    pub_.publish(cloud_out);
  }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    pcl::SACSegmentation<Point> seg_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_model_segmentation_node");
    PlaneModelSegmentation pms;
    ros::spin();
    return 0;
}


