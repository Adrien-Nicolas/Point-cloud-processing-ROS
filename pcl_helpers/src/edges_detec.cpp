#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ConvexHullFilter
{
public:
  ConvexHullFilter()
  {
    pub_ = nh_.advertise<PointCloud>("HULL_OUT", 1);
    sub_ = nh_.subscribe("point_cloud_in", 1, &ConvexHullFilter::cloudCallback, this);
  }

  ~ConvexHullFilter() {}

  void cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    PointCloud::Ptr cloud_hull (new PointCloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_in);
    hull.reconstruct(*cloud_hull);

    pub_.publish(cloud_hull);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "convex_hull_filter_node");

  ConvexHullFilter chf;

  ros::spin();

  return 0;
}