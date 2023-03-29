#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_helpers/passthrough_filter_nodeConfig.h>
#include <pcl_helpers/voxel_filter_nodeConfig.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


class VoxelFilterNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  VoxelFilterNode()
  {
    pub_ = nh_.advertise<PointCloud>("/streaming_pointcloud_registered",1);
    //sub_ = nh_.subscribe ("/camera/depth_registered/pointas", 1,  &VoxelFilterNode::cloudCallback, this);
    sub_ = nh_.subscribe ("/camera/depth_registered/points", 1,  &VoxelFilterNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&VoxelFilterNode::dynReconfCallback, this, _1, _2));

    double leafsize;
    double upper_limit, lower_limit;

    // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
    ros::NodeHandle private_nh("~");

    //read parameters with default value
    private_nh.param("leafsize", leafsize, 0.015);
    private_nh.param("lower_limit", lower_limit, 0.5);
    private_nh.param("upper_limit", upper_limit, 2.5);

    vg_.setLeafSize (leafsize,leafsize,leafsize);
    pt_.setFilterFieldName ("z");
    pt_.setFilterLimits (lower_limit, upper_limit);
  }

  ~VoxelFilterNode() {}

  void
  dynReconfCallback(matrix_transform::voxel_filter_nodeConfig &config, uint32_t level)
  {
    vg_.setLeafSize(config.leafsize, config.leafsize, config.leafsize);
    pt_.setFilterLimits(config.lower_limit, config.upper_limit);
  }
  /*
  void rotatePointcloud()
  {

  }*/

  void
  cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    vg_.setInputCloud(cloud_in);
    PointCloud cloud_out;
    vg_.filter(cloud_out);
    pub_.publish(cloud_out);
    //PointCloud::ConstPtr cloud_intermed;
    //cloud_intermed = &cloud_out;

    //pt_.setInputCloud(intermed);
    //PointCloud cloud_out2;
    //pt_.filter(cloud_out2);
    //pub_.publish(cloud_out2);//*/
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<matrix_transform::voxel_filter_nodeConfig> config_server_;

  pcl::VoxelGrid<Point> vg_;
  pcl::PassThrough<Point> pt_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "real_time_computed_points");

  VoxelFilterNode vf;

  ros::spin();
}
