#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class StatisticalOutlierRemovalNode
{
public:
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;

  StatisticalOutlierRemovalNode()
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("STAT_OUT",1);
    sub_ = nh_.subscribe("point_cloud_in", 1, &StatisticalOutlierRemovalNode::cloudCallback, this);
  }

  ~StatisticalOutlierRemovalNode() {}

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    PointCloud::Ptr cloud (new PointCloud);
    pcl::fromROSMsg(*cloud_in, *cloud);

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud);

    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud, cloud_out);
    pub_.publish(cloud_out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  pcl::StatisticalOutlierRemoval<Point> sor_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "statistical_outlier_removal_node");
  StatisticalOutlierRemovalNode sor;
  ros::spin();
  return 0;
}