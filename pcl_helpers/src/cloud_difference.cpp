#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudDifference
{
public:
  CloudDifference()
  {
    pub_ = nh_.advertise<PointCloud>("void_out",1);
    sub_ = nh_.subscribe("point_cloud_in", 1, &CloudDifference::cloudCallback, this);
  }

  void cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // set input cloud for convex hull
    chull.setInputCloud(cloud_in);

    // perform reconstruction
    chull.reconstruct(*cloud_hull);

    // Create the filtering object
    extract.setInputCloud(cloud_in);
    extract.setIndices(chull.getIndices());
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    pub_.publish(cloud_filtered);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_difference");
  CloudDifference cd;
  ros::spin();
  return 0;
}
