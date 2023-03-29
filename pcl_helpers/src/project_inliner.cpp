#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



class ProjectInliersNode 
{
public:
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;


    ProjectInliersNode() {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("INLINE_OUT",1);
        sub_ = nh_.subscribe("point_cloud_in", 1, &ProjectInliersNode::cloudCallback, this);
    }

  ~ProjectInliersNode() {}

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in) {



    PointCloud::Ptr cloud (new PointCloud);
    PointCloud::Ptr cloud_projected (new PointCloud);


    pcl::fromROSMsg(*cloud_in, *cloud);

    pcl::ProjectInliers<pcl::PointXYZ> proj;


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Convert the result to a ROS message
    sensor_msgs::PointCloud2 result;
    pcl::toROSMsg(*cloud_projected, result);
    pub_.publish(result);
    

  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_inliers_node");
  ProjectInliersNode node;
  ros::spin();
  return 0;
}